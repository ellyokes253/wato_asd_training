#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){

  // steps from pseudocode 
  // Step 1: Initialize costmap
  initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      // Calculate grid coordinates
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }
 
  // Step 3: Inflate obstacles
  inflateObstacles();
 
  // Step 4: Publish costmap
  publishCostmap();
  
}

void CostmapNode::initializeCostmap(){
  std::fill(costmap_.costmap_grid.data.begin(), costmap_.costmap_grid.data.end(), 0);
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid){

  double x_coord = range * std::cos(angle);
  double y_coord = range * std::sin(angle);

  auto &info = costmap_.costmap_grid.info;

  x_grid = (info.width / 2) + static_cast<int>(std::round(x_coord / info.resolution));
  y_grid = (info.height / 2) + static_cast<int>(std::round(y_coord / info.resolution));
 
}

void CostmapNode::markObstacle(int x_grid, int y_grid){
  auto &info = costmap_.costmap_grid.info;

  if (x_grid >= 0 && x_grid < static_cast<int>(info.width) && y_grid >= 0 && y_grid < static_cast<int>(info.height)){

    // making a 2D array into a 1D array -> index = (y * width) + x
    int index = (y_grid * info.width) + x_grid;
    costmap_.costmap_grid.data[index] = 100; //occupied cost
  }
}

void CostmapNode::inflateObstacles(){
  const int inflation_radius = 1;
  const int max_cost = 100;

  auto temp = costmap_.costmap_grid.data;

  double res = costmap_.costmap_grid.info.resolution;
  int width = costmap_.costmap_grid.info.width;
  int height = costmap_.costmap_grid.info.height;

  int cell_radius = static_cast<int>(std::ceil(inflation_radius / res));

  for (int y = 0; y < height; y ++){
    for (int x = 0; x < width; x ++){
      if (temp[(y * width) + x] == 100){
        for (int dy = -cell_radius; dy <= cell_radius; ++dy){
          for (int dx = -cell_radius; dx <= cell_radius; ++dx){

            int adj_x = x + dx;
            int adj_y = y + dy;

            if (adj_x >= 0 && adj_x < width && adj_y >= 0 && adj_y < height){

              double distance = std::hypot(dx, dy) * res;

              if (distance <= inflation_radius){
                int calc_cost = static_cast<int>(max_cost * (1.0 - (distance / inflation_radius)));

                int target_index = (adj_y * width) + adj_x;

                // makes sure a high cosst value isn't overwritten by mistake due to obstacles being too close to each other
                auto &active_cell = costmap_.costmap_grid.data[target_index];
                active_cell = std::max(static_cast<int>(active_cell), calc_cost);
              }
            }
          }
        }
      }
    }
  }
}

void CostmapNode::publishCostmap(){
  costmap_.costmap_grid.header.stamp = this->get_clock()->now();
  costmap_.costmap_grid.header.frame_id = "robot/chassis/lidar";

  costmap_pub_->publish(costmap_.costmap_grid);
  //RCLCPP_INFO(this->get_logger(), "Costmap publishing!");
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}