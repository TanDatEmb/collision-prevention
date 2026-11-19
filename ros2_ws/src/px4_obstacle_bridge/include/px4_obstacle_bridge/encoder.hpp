#pragma once

#include <optional>

#include "obstacle_sector_map/types.hpp"
#include "px4_msgs/msg/obstacle_distance.hpp"

namespace px4_obstacle_bridge
{

[[nodiscard]] std::optional<px4_msgs::msg::ObstacleDistance> encodeObstacleDistance(
  const obstacle_sector_map::SectorMap & sector_map,
  const obstacle_sector_map::SectorMapConfig & config);

}  // namespace px4_obstacle_bridge
