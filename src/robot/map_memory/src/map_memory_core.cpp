#include "map_memory_core.hpp"

#include <vector>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

  void MapMemoryCore::initializeGlobalMap(){
      global_map_.header.frame_id = "robot/chassis/lidar";
      global_map_.info.resolution = resolution;
      global_map_.info.width = width;
      global_map_.info.height = height;
      global_map_.info.origin.position.x = -(width * resolution) / 2.0;
      global_map_.info.origin.position.y = -(height * resolution) / 2.0;
      global_map_.info.origin.orientation.w = 1.0;
      global_map_.data.assign(width * height, -1); 
    }

} 
