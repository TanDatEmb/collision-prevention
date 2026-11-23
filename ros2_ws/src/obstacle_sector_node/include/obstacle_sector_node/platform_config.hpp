#pragma once

#include <string>

#include "livox_ray_source/pose_history.hpp"
#include "livox_ray_source/ray_leveling.hpp"
#include "obstacle_sector_map/types.hpp"

namespace obstacle_sector_node
{

/// Reads the sector map values of a platform YAML file: `sector_map.height_up_m`, `height_down_m`,
/// `obstacle_window_m`, `obstacle_min_returns` and `free_min_rays` (ADR 0020 D2). None has a default.
/// Throws std::invalid_argument when the file cannot be read, a value is null, missing, not a
/// number (the coverage count must be a positive integer) or the result is not a valid config.
[[nodiscard]] obstacle_sector_map::SectorMapConfig loadSectorMapConfig(
  const std::string & yaml_path);

/// What the leveling step needs from a platform YAML (ADR 0020 D1, ADR 0022): the mount of the sensor and the
/// size of the pose history.
struct LevelingSettings
{
  livox_ray_source::LevelingConfig leveling;
  livox_ray_source::PoseHistoryConfig history;
};

/// Reads `sensor.mount_pose_in_base_link.xyz_m` (the lever arm, from base_link, FLU) and `rpy_rad` (roll about
/// forward, pitch about left with positive = boresight down, yaw about up with positive = turned left), and
/// `pose_history.depth` and `pose_history.max_bracket_gap_ms`. None has a default. Throws std::invalid_argument
/// when the file cannot be read, a value is null, missing or not a number, or a result is not valid.
[[nodiscard]] LevelingSettings loadLevelingSettings(const std::string & yaml_path);

}  // namespace obstacle_sector_node
