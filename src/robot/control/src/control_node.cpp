#include "control_node.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
  control_.current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  control_.robot_odom_ = msg;
}

void ControlNode::controlLoop(){
  if (!control_.current_path_ || !control_.robot_odom_ || control_.current_path_->poses.empty()){
    return;
  }

  const auto &goal_pos = control_.current_path_->poses.back().pose.position;
  double dist_to_goal = computeDistance(control_.robot_odom_->pose.pose.position, goal_pos);

  if (dist_to_goal < goal_tolerance_) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "STOP");
    return; 
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point){
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint(){
  if (control_.current_path_->poses.empty()){
    return std::nullopt;
  }

  for (const auto &pose_stamped : control_.current_path_->poses){
    double distance = computeDistance(control_.robot_odom_->pose.pose.position, pose_stamped.pose.position);
    if (distance >= lookahead_distance_){
      return pose_stamped;
    }
  }

  return control_.current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target){
  geometry_msgs::msg::Twist cmd_vel;

  double dx = target.pose.position.x - control_.robot_odom_->pose.pose.position.x;
  double dy = target.pose.position.y - control_.robot_odom_->pose.pose.position.y;

  double robot_yaw = extractYaw(control_.robot_odom_->pose.pose.orientation);
  double local_x = dx * std::cos(-robot_yaw) - dy * std::sin(-robot_yaw);
  double local_y = dx * std::sin(-robot_yaw) + dy * std::cos(-robot_yaw);

  double L_sq = (local_x * local_x) + (local_y * local_y);
  if (L_sq < 0.001) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  double curvature = (2.0 * local_y) / L_sq;

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = cmd_vel.linear.x * curvature;

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b){
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat){
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
