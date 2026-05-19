#include <chrono>
#include <memory>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  planner_.current_map_ = msg;
  map_updated_since_last_plan_ = true;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  planner_.current_odom_ = msg;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  planner_.goal_point_ = msg;

  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::timerCallback(){
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL){
    if (goalReached()){
      state_ = State::WAITING_FOR_GOAL;
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->get_clock()->now();
      empty_path.header.frame_id = "map";
      path_pub_->publish(empty_path);
      return;
    }

    if (map_updated_since_last_plan_){
      RCLCPP_INFO(this->get_logger(), "Map structural layout updated. Recalculating path...");
      planPath();
    }
  }
}

void PlannerNode::planPath(){
  if (!planner_.current_map_ || !planner_.current_odom_ || !planner_.goal_point_){
    RCLCPP_WARN(this->get_logger(), "A* blocked: Incomplete Map, Odometry or Goal states.");
    return;
  }

  // Convert frames from spatial coordinate metric system into logical array matrix coordinates
  CellIndex start = worldToGrid(planner_.current_odom_->pose.pose.position.x, planner_.current_odom_->pose.pose.position.y);
  CellIndex goal = worldToGrid(planner_.goal_point_->point.x, planner_.goal_point_->point.y);

  if (!isCellValid(start) || !isCellValid(goal)) {
    RCLCPP_ERROR(this->get_logger(), "Start or Destination point falls inside an obstacle/out of map range.");
    return;
  }

  // Prepare priority containers
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, getHeuristic(start, goal)));

  bool path_found = false;

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      path_found = true;
      break;
    }

    for (const auto &neighbor : getNeighbors(current)) {
      // Calculate geometric edge weights (diagonal vs linear adjustments)
      double movement_cost = std::hypot((neighbor.x - current.x), (neighbor.y - current.y));
      double tentative_g_score = g_score[current] + movement_cost;

      if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        double f_score = tentative_g_score + getHeuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f_score));
      }
    }
  }

  if (!path_found) {
    RCLCPP_WARN(this->get_logger(), "A* algorithm could not resolve a continuous path to target.");
    return;
  }

  // Trace sequence backward
  std::vector<CellIndex> grid_path;
  CellIndex current_cell = goal;
  while (current_cell != start) {
    grid_path.push_back(current_cell);
    current_cell = came_from[current_cell];
  }
  grid_path.push_back(start);
  std::reverse(grid_path.begin(), grid_path.end());

  // Format output mapping msg array
  nav_msgs::msg::Path ros_path;
  ros_path.header.stamp = this->get_clock()->now();
  ros_path.header.frame_id = "sim_world";

  for (const auto &cell : grid_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = ros_path.header;
        
    gridToWorld(cell, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
        
    ros_path.poses.push_back(pose);
  }

  path_pub_->publish(ros_path);
  map_updated_since_last_plan_ = false;

}

bool PlannerNode::goalReached(){
  if (!planner_.current_odom_ || !planner_.goal_point_) return false;

  double dx = planner_.goal_point_->point.x - planner_.current_odom_->pose.pose.position.x;
  double dy = planner_.goal_point_->point.y - planner_.current_odom_->pose.pose.position.y;
  double distance = std::hypot(dx, dy);

  return distance <= goal_tolerance;
}

CellIndex PlannerNode::worldToGrid(double wx, double wy){
  double origin_x = planner_.current_map_->info.origin.position.x;
  double origin_y = planner_.current_map_->info.origin.position.y;
  double res = planner_.current_map_->info.resolution;

  int gx = static_cast<int>((wx - origin_x) / res);
  int gy = static_cast<int>((wy - origin_y) / res);
  return CellIndex(gx, gy);
}

void PlannerNode::gridToWorld(const CellIndex &cell, double &wx, double &wy) {
  double origin_x = planner_.current_map_->info.origin.position.x;
  double origin_y = planner_.current_map_->info.origin.position.y;
  double res = planner_.current_map_->info.resolution;

  wx = origin_x + (cell.x + 0.5) * res;
  wy = origin_y + (cell.y + 0.5) * res;
}

int PlannerNode::gridToIndex(const CellIndex &cell) {
  return cell.y * planner_.current_map_->info.width + cell.x;
}

bool PlannerNode::isCellValid(const CellIndex &cell) {
  if (cell.x < 0 || cell.x >= static_cast<int>(planner_.current_map_->info.width) || cell.y < 0 || cell.y >= static_cast<int>(planner_.current_map_->info.height)) {
    return false;
  }
  int index = gridToIndex(cell);
  int cost = planner_.current_map_->data[index];
      
  // choosing cost < 50 to act as safe clear cell threshold, and -1 represents unknown cells
  return (cost >= 0 && cost < 50); 
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex& cell) {
  std::vector<CellIndex> neighbors;
  int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

  for (int i = 0; i < 8; ++i) {
    CellIndex neighbor(cell.x + dx[i], cell.y + dy[i]);
    if (isCellValid(neighbor)) {
      neighbors.push_back(neighbor);
    }
  }
  return neighbors;
}

double PlannerNode::getHeuristic(const CellIndex &a, const CellIndex &b) {
  return std::hypot((a.x - b.x), (a.y - b.y));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
