#pragma once

#include <array>
#include <string>
#include <vector>

#include "livox_ray_source/ray_leveling.hpp"
#include "obstacle_sector_map/types.hpp"

namespace livox_ray_source
{

/// Upright cylinder in the body frame, base_link FLU (ADR 0006, OPEN-046): axis through
/// (center_x_m, center_y_m) parallel to the body z axis, radius radius_m, z range z_min_m to z_max_m.
struct MaskCylinder
{
  double center_x_m;
  double center_y_m;
  double radius_m;
  double z_min_m;
  double z_max_m;
};

/// Static geometric self-mask (ADR 0006, ICD-CP-06). Rays come in the sensor frame with the sensor at
/// the origin, before leveling; the mount (rotation and lever arm, the same values the leveler uses)
/// carries each ray into the body frame, where the cylinders are. There is no temporal filter.
class SelfMask
{
public:
  /// Throws std::invalid_argument for no cylinder, a non-finite value, a radius that is not
  /// positive, an empty z range or a mount that is not valid.
  SelfMask(std::vector<MaskCylinder> cylinders, const LevelingConfig & mount);

  /// A return is masked when its end point is inside a cylinder. A no-return is masked when its
  /// ray segment from the sensor up to `max_distance_m` crosses a cylinder. Other kinds are not
  /// masked here (the sector map rejects them).
  [[nodiscard]] bool isMasked(
    const obstacle_sector_map::RaySample & ray, double max_distance_m) const;

  /// Sets `ray.masked` when isMasked() is true; never clears it.
  void apply(obstacle_sector_map::RaySample & ray, double max_distance_m) const;

private:
  std::vector<MaskCylinder> _cylinders;
  std::array<double, 9> _rotation;  // sensor FLU to body FLU, row-major
  std::array<double, 3> _origin_m;  // the sensor position in the body frame (the lever arm)
};

/// Reads `self_mask.cylinders` (center_m, radius_m, z_min_m, z_max_m, body frame) from a platform YAML
/// file; `mount` is the sensor mount of the same file. Throws std::invalid_argument when the file
/// cannot be read, the list is missing or empty, or any value is null, missing or not a finite number.
[[nodiscard]] SelfMask loadSelfMask(const std::string & yaml_path, const LevelingConfig & mount);

}  // namespace livox_ray_source
