#include "obstacle_sector_node/platform_config.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <stdexcept>

namespace obstacle_sector_node
{
namespace
{

YAML::Node child(const YAML::Node & parent, const std::string & name)
{
  return parent ? parent[name] : YAML::Node();
}

double number(const YAML::Node & node, const std::string & what)
{
  if (!node || !node.IsScalar()) {
    throw std::invalid_argument("platform config: " + what + " is missing or not a number");
  }

  double value{0.0};
  try {
    value = node.as<double>();
  }
  catch (const YAML::Exception &) {
    throw std::invalid_argument("platform config: " + what + " is null or not a number");
  }

  if (!std::isfinite(value)) {
    throw std::invalid_argument("platform config: " + what + " is not finite");
  }
  return value;
}

std::array<double, 3> vector3(const YAML::Node & node, const std::string & what)
{
  if (!node || !node.IsSequence() || node.size() != 3U) {
    throw std::invalid_argument("platform config: " + what + " is missing or not three numbers");
  }
  return {number(node[0], what), number(node[1], what), number(node[2], what)};
}

}  // namespace

obstacle_sector_map::SectorMapConfig loadSectorMapConfig(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  }
  catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string("platform config: cannot read file: ") + error.what());
  }

  const YAML::Node map = child(root, "sector_map");
  const double height_up = number(child(map, "height_up_m"), "height up");
  const double height_down = number(child(map, "height_down_m"), "height down");
  const double window = number(child(map, "obstacle_window_m"), "obstacle window");
  const double obstacle_returns = number(child(map, "obstacle_min_returns"), "obstacle returns");
  const double free_rays = number(child(map, "free_min_rays"), "free rays");

  for (const double count : {obstacle_returns, free_rays}) {
    if (count < 1.0 || count != std::floor(count) || count > 4.0e9) {
      throw std::invalid_argument("platform config: a ray count must be a positive integer");
    }
  }

  obstacle_sector_map::SectorMapConfig config{
    static_cast<float>(height_up), static_cast<float>(height_down), static_cast<float>(window),
    static_cast<uint32_t>(obstacle_returns), static_cast<uint32_t>(free_rays)};

  // Optional: the start of the rear class (issue #141). Absent or null keeps the default, no rear class.
  const YAML::Node rear_start = child(map, "rear_start_rad");
  if (rear_start && !rear_start.IsNull()) {
    config.rear_start_rad = static_cast<float>(number(rear_start, "rear start"));
  }

  if (!config.isValid()) {
    throw std::invalid_argument("platform config: sector map values are not valid");
  }
  return config;
}

LevelingSettings loadLevelingSettings(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  }
  catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string("platform config: cannot read file: ") + error.what());
  }

  const YAML::Node mount = child(child(root, "sensor"), "mount_pose_in_base_link");
  const auto lever_arm = vector3(child(mount, "xyz_m"), "mount xyz");
  const auto rpy = vector3(child(mount, "rpy_rad"), "mount rpy");
  const YAML::Node history = child(root, "pose_history");
  const double depth = number(child(history, "depth"), "pose history depth");
  const double gap_ms = number(child(history, "max_bracket_gap_ms"), "pose history gap");
  if (depth < 2.0 || depth != std::floor(depth) || depth > 1.0e6 || gap_ms <= 0.0 || gap_ms > 1.0e6) {
    throw std::invalid_argument(
      "platform config: pose history depth must be an integer of at least 2 and the gap positive");
  }

  LevelingSettings settings;
  settings.leveling.roll_rad = rpy[0];
  settings.leveling.pitch_down_rad = rpy[1];
  settings.leveling.yaw_left_rad = rpy[2];
  settings.leveling.lever_arm_flu_m = lever_arm;
  settings.history.depth = static_cast<std::size_t>(depth);
  settings.history.max_bracket_gap_ns = static_cast<uint64_t>(gap_ms * 1.0e6);
  if (!settings.leveling.isValid() || !settings.history.isValid()) {
    throw std::invalid_argument("platform config: mount or pose history values are not valid");
  }
  return settings;
}

}  // namespace obstacle_sector_node
