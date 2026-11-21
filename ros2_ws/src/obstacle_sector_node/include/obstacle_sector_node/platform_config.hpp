#pragma once

#include <string>

#include "obstacle_sector_map/types.hpp"

namespace obstacle_sector_node
{

/// Reads the sector map values of a platform YAML file: `sector_map.height_up_m`, `height_down_m`,
/// `obstacle_window_m`, `obstacle_min_returns` and `free_min_rays` (ADR 0020 D2). None has a default.
/// Throws std::invalid_argument when the file cannot be read, a value is null, missing, not a
/// number (the coverage count must be a positive integer) or the result is not a valid config.
[[nodiscard]] obstacle_sector_map::SectorMapConfig loadSectorMapConfig(
  const std::string & yaml_path);

}  // namespace obstacle_sector_node
