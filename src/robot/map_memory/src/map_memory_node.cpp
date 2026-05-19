#include <chrono>
#include <memory>
#include <cmath>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MapMemoryNode::updateMap, this));

  map_memory_.initializeGlobalMap();
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  // Compute vehicle Heading Angle (Yaw)
  current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  checkDistanceThreshold(current_x, current_y);

}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){

  map_memory_.latest_costmap_ = *msg;
  costmap_updated_ = true;

}

void MapMemoryNode::updateMap(){
  if (should_update_map_ && costmap_updated_){
    integrateCostmap();

    map_memory_.global_map_.header.stamp = this->get_clock()->now();
    map_memory_.global_map_.header.frame_id = "sim_world";
    
    map_pub_->publish(map_memory_.global_map_);
    should_update_map_ = false;
  }
  else{
    map_memory_.global_map_.header.stamp = this->get_clock()->now();
    map_memory_.global_map_.header.frame_id = "sim_world";
    
    map_pub_->publish(map_memory_.global_map_); // for the planner node to work properly
  }
}

void MapMemoryNode::integrateCostmap(){
  int costmap_w = static_cast<int>(map_memory_.latest_costmap_.info.width);
  int costmap_h = static_cast<int>(map_memory_.latest_costmap_.info.height);
  double costmap_res = map_memory_.latest_costmap_.info.resolution;

  double cos_yaw = std::cos(current_yaw_);
  double sin_yaw = std::sin(current_yaw_);

  double center_x = static_cast<double>(costmap_w) / 2.0;
  double center_y = static_cast<double>(costmap_h) / 2.0;

  for (int y = 0; y < costmap_h; ++y){
    for (int x = 0; x < costmap_w; ++x){
      int8_t cost = map_memory_.latest_costmap_.data[y * costmap_w + x];

      if (cost == -1){
        continue; // skip unknown cells
      }

      double local_x = (static_cast<double>(x) - center_x) * costmap_res;
      double local_y = (static_cast<double>(y) - center_y) * costmap_res;

      double global_x = current_x + local_x * cos_yaw - local_y * sin_yaw;
      double global_y = current_y + local_x * sin_yaw + local_y * cos_yaw;

      // global grid indicies
      int gx = static_cast<int>((global_x - map_memory_.global_map_.info.origin.position.x) / map_memory_.global_map_.info.resolution);
      int gy = static_cast<int>((global_y - map_memory_.global_map_.info.origin.position.y) / map_memory_.global_map_.info.resolution);

      if (gx < 0 || gx >= static_cast<int>(map_memory_.global_map_.info.width) || gy < 0 || gy >= static_cast<int>(map_memory_.global_map_.info.height)) {
        continue;
      }

      int global_index = gy * map_memory_.global_map_.info.width + gx;
      map_memory_.global_map_.data[global_index] = cost;
    }
  }
} 

bool MapMemoryNode::checkDistanceThreshold(double x, double y){
  if (!has_odom_){
    last_x = x;
    last_y = y;
    has_odom_ = true;
    should_update_map_ = true;
  }

  double distance = std::hypot((x - last_x), (y - last_y));
  if (distance >= distance_threshold){
    last_x = x;
    last_y = y;
    should_update_map_ = true;
  }
  return should_update_map_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
