#pragma once

#include <string>
#include <vector>

#include "obstacle_sector_map/types.hpp"

namespace livox_ray_source
{

/// Upright cylinder in the sensor frame (ADR 0006): axis through (center_x_m, center_y_m),
/// extent z_min_m to z_max_m, radius radius_m.
struct MaskCylinder
{
  double center_x_m;
  double center_y_m;
  double radius_m;
  double z_min_m;
  double z_max_m;
};

/// Static geometric self-mask (ADR 0006, ICD-CP-06). Rays must be in the sensor frame with the
/// sensor at the origin. There is no temporal filter.
class SelfMask
{
public:
  /// Throws std::invalid_argument for no cylinder, a non-finite value, a radius that is not
  /// positive or an empty z range.
  explicit SelfMask(std::vector<MaskCylinder> cylinders);

  /// A return is masked when its end point is inside a cylinder. A no-return is masked when its
  /// ray segment from the sensor up to `max_distance_m` crosses a cylinder. Other kinds are not
  /// masked here (the sector map rejects them).
  [[nodiscard]] bool isMasked(
    const obstacle_sector_map::RaySample & ray, double max_distance_m) const;

  /// Sets `ray.masked` when isMasked() is true; never clears it.
  void apply(obstacle_sector_map::RaySample & ray, double max_distance_m) const;

private:
  std::vector<MaskCylinder> _cylinders;
};

/// Reads `self_mask.cylinders` (center_m, radius_m, z_min_m, z_max_m) from a platform YAML file.
/// `center_m` needs three finite numbers; its z entry is checked but the z extent comes from
/// z_min_m and z_max_m. Throws std::invalid_argument when the file cannot be read, the list is
/// missing or empty, or any value is null, missing or not a finite number.
[[nodiscard]] SelfMask loadSelfMask(const std::string & yaml_path);

}  // namespace livox_ray_source
