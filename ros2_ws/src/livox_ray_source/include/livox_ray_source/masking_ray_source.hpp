#pragma once

#include <memory>
#include <optional>

#include "livox_ray_source/ray_source.hpp"
#include "livox_ray_source/self_mask.hpp"

namespace livox_ray_source
{

/// Decorator that applies the static self-mask to every ray of another source (ICD-CP-06).
/// The inner source must deliver rays in the sensor frame, before any frame change or deskew.
class MaskingRaySource : public RaySource
{
public:
  /// Throws std::invalid_argument when `inner` is null or `max_distance_m` is not positive and
  /// finite.
  MaskingRaySource(std::unique_ptr<RaySource> inner, SelfMask mask, double max_distance_m);

  [[nodiscard]] std::optional<RayFrame> nextFrame() override;

private:
  std::unique_ptr<RaySource> _inner;
  SelfMask _mask;
  double _max_distance_m;
};

}  // namespace livox_ray_source
