#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateMap();
    void integrateCostmap();

    // extra helper method
    bool checkDistanceThreshold(double x, double y);

    double last_x, last_y = 0.0;
    double current_x, current_y = 0.0;
    const double distance_threshold = 5.0;
    double current_yaw_ = 0;
  
    //flags
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
    bool has_odom_ = false;

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 
