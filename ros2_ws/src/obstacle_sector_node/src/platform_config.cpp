#include "obstacle_sector_node/platform_config.hpp"

#include <yaml-cpp/yaml.h>

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

  const YAML::Node sensor = child(root, "sensor");
  const double min_elevation =
    number(child(sensor, "collision_elevation_min_rad"), "elevation min");
  const double max_elevation =
    number(child(sensor, "collision_elevation_max_rad"), "elevation max");
  const double coverage =
    number(child(child(root, "sector_map"), "coverage_min_rays_per_sector"), "coverage count");

  if (coverage < 1.0 || coverage != std::floor(coverage) || coverage > 4.0e9) {
    throw std::invalid_argument("platform config: coverage count must be a positive integer");
  }

  const obstacle_sector_map::SectorMapConfig config{
    static_cast<float>(min_elevation), static_cast<float>(max_elevation),
    static_cast<uint32_t>(coverage)};

  if (!config.isValid()) {
    throw std::invalid_argument("platform config: sector map values are not valid");
  }
  return config;
}

}  // namespace obstacle_sector_node
