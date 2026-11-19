#pragma once

#include <string>

#include "obstacle_sector_map/types.hpp"

namespace obstacle_sector_node
{

/// Reads the sector map values of a platform YAML file: `sensor.collision_elevation_min_rad`,
/// `sensor.collision_elevation_max_rad` and `sector_map.coverage_min_rays_per_sector`.
/// Throws std::invalid_argument when the file cannot be read, a value is null, missing, not a
/// number (the coverage count must be a positive integer) or the result is not a valid config.
[[nodiscard]] obstacle_sector_map::SectorMapConfig loadSectorMapConfig(
  const std::string & yaml_path);

}  // namespace obstacle_sector_node
