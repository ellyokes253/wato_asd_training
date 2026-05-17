#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger){

    // initial params
    costmap_grid.info.resolution = 0.1;
    costmap_grid.info.width = 1000;
    costmap_grid.info.height = 1000;

    // for convertToGrid()
    costmap_grid.info.origin.position.x = -((costmap_grid.info.width * costmap_grid.info.resolution) / 2.0);
    costmap_grid.info.origin.position.y = -((costmap_grid.info.height * costmap_grid.info.resolution) / 2.0);
    costmap_grid.info.origin.orientation.w = 1.0;

    // data is a 1D array, have to convert 2D grid to 1D
    costmap_grid.data.resize(costmap_grid.info.width * costmap_grid.info.height, 0);
}

}