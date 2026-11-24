#include "livox_ray_source/self_mask.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace livox_ray_source
{
namespace
{

using obstacle_sector_map::RayReturnKind;

bool inside(const MaskCylinder & c, double x, double y, double z)
{
  const double dx = x - c.center_x_m;
  const double dy = y - c.center_y_m;
  return dx * dx + dy * dy <= c.radius_m * c.radius_m && z >= c.z_min_m && z <= c.z_max_m;
}

// True when the segment o + t * d, t in [0, t_max], meets the cylinder.
bool segmentCrosses(const MaskCylinder & c, const std::array<double, 3> & o, double dx, double dy, double dz,
  double t_max)
{
  double lo = 0.0;
  double hi = t_max;

  // Horizontal condition: |(o_xy - c_xy) + t * (dx, dy)|^2 <= r^2, a quadratic in t.
  const double px = o[0] - c.center_x_m;
  const double py = o[1] - c.center_y_m;
  const double a = dx * dx + dy * dy;
  const double b = 2.0 * (px * dx + py * dy);
  const double q = px * px + py * py - c.radius_m * c.radius_m;

  if (a < 1.0e-12) {
    if (q > 0.0) {
      return false;
    }
  }
  else {
    const double disc = b * b - 4.0 * a * q;
    if (disc < 0.0) {
      return false;
    }
    const double root = std::sqrt(disc);
    lo = std::max(lo, (-b - root) / (2.0 * a));
    hi = std::min(hi, (-b + root) / (2.0 * a));
  }

  // Vertical condition: z_min <= o_z + t * dz <= z_max.
  if (std::fabs(dz) < 1.0e-12) {
    if (o[2] < c.z_min_m || o[2] > c.z_max_m) {
      return false;
    }
  }
  else {
    const double t1 = (c.z_min_m - o[2]) / dz;
    const double t2 = (c.z_max_m - o[2]) / dz;
    lo = std::max(lo, std::min(t1, t2));
    hi = std::min(hi, std::max(t1, t2));
  }

  return lo <= hi;
}

double finiteNumber(const YAML::Node & node, const std::string & what)
{
  if (!node || !node.IsScalar()) {
    throw std::invalid_argument("self_mask: " + what + " is missing or not a number");
  }

  double value{0.0};
  try {
    value = node.as<double>();
  }
  catch (const YAML::Exception &) {
    throw std::invalid_argument("self_mask: " + what + " is null or not a number");
  }

  if (!std::isfinite(value)) {
    throw std::invalid_argument("self_mask: " + what + " is not finite");
  }
  return value;
}

}  // namespace

SelfMask::SelfMask(std::vector<MaskCylinder> cylinders, const LevelingConfig & mount)
: _cylinders(std::move(cylinders)), _rotation{}, _origin_m(mount.lever_arm_flu_m)
{
  if (!mount.isValid()) {
    throw std::invalid_argument("self_mask: the mount is not valid");
  }
  _rotation = mountRotation(mount);

  if (_cylinders.empty()) {
    throw std::invalid_argument("self_mask: no cylinder declared");
  }

  for (const auto & c : _cylinders) {
    const bool finite = std::isfinite(c.center_x_m) && std::isfinite(c.center_y_m) &&
                        std::isfinite(c.radius_m) && std::isfinite(c.z_min_m) &&
                        std::isfinite(c.z_max_m);
    if (!finite || c.radius_m <= 0.0 || c.z_min_m >= c.z_max_m) {
      throw std::invalid_argument("self_mask: invalid cylinder");
    }
  }
}

bool SelfMask::isMasked(const obstacle_sector_map::RaySample & ray, double max_distance_m) const
{
  if (!std::isfinite(ray.azimuth_rad) || !std::isfinite(ray.elevation_rad)) {
    return false;
  }

  // the ray direction in the sensor frame, then in the body frame (the origin is the lever arm)
  const double cos_el = std::cos(static_cast<double>(ray.elevation_rad));
  const double sx = cos_el * std::cos(static_cast<double>(ray.azimuth_rad));
  const double sy = cos_el * std::sin(static_cast<double>(ray.azimuth_rad));
  const double sz = std::sin(static_cast<double>(ray.elevation_rad));
  const auto & r = _rotation;
  const double dx = r[0] * sx + r[1] * sy + r[2] * sz;
  const double dy = r[3] * sx + r[4] * sy + r[5] * sz;
  const double dz = r[6] * sx + r[7] * sy + r[8] * sz;
  const auto & o = _origin_m;

  if (ray.return_kind == RayReturnKind::kReturn) {
    if (!std::isfinite(ray.range_m)) {
      return false;
    }
    const double range = static_cast<double>(ray.range_m);
    return std::any_of(_cylinders.begin(), _cylinders.end(), [&](const MaskCylinder & c) {
      return inside(c, o[0] + range * dx, o[1] + range * dy, o[2] + range * dz);
    });
  }

  if (ray.return_kind == RayReturnKind::kQualifiedNoReturn) {
    if (!std::isfinite(max_distance_m) || max_distance_m <= 0.0) {
      return false;
    }
    return std::any_of(_cylinders.begin(), _cylinders.end(), [&](const MaskCylinder & c) {
      return segmentCrosses(c, o, dx, dy, dz, max_distance_m);
    });
  }

  return false;
}

void SelfMask::apply(obstacle_sector_map::RaySample & ray, double max_distance_m) const
{
  if (isMasked(ray, max_distance_m)) {
    ray.masked = true;
  }
}

SelfMask loadSelfMask(const std::string & yaml_path, const LevelingConfig & mount)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  }
  catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string("self_mask: cannot read file: ") + error.what());
  }

  const YAML::Node list = root["self_mask"] ? root["self_mask"]["cylinders"] : YAML::Node();
  if (!list || !list.IsSequence() || list.size() == 0U) {
    throw std::invalid_argument("self_mask: cylinders list is missing or empty");
  }

  std::vector<MaskCylinder> cylinders;
  for (const auto & entry : list) {
    const YAML::Node center = entry["center_m"];
    if (!center || !center.IsSequence() || center.size() != 2U) {
      throw std::invalid_argument("self_mask: center_m needs two numbers [x, y]");
    }
    const double cx = finiteNumber(center[0], "center_m x");
    const double cy = finiteNumber(center[1], "center_m y");
    cylinders.push_back(MaskCylinder{
      cx, cy, finiteNumber(entry["radius_m"], "radius_m"),
      finiteNumber(entry["z_min_m"], "z_min_m"), finiteNumber(entry["z_max_m"], "z_max_m")});
  }

  return SelfMask(std::move(cylinders), mount);
}

}  // namespace livox_ray_source
