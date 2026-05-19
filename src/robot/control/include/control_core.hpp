#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    nav_msgs::msg::Path::SharedPtr current_path_;
  
  private:
    rclcpp::Logger logger_;
};

} 

#endif 
