#include "livox_ray_source/masking_ray_source.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace livox_ray_source
{

MaskingRaySource::MaskingRaySource(
  std::unique_ptr<RaySource> inner, SelfMask mask, double max_distance_m)
: _inner(std::move(inner)), _mask(std::move(mask)), _max_distance_m(max_distance_m)
{
  if (!_inner) {
    throw std::invalid_argument("masking source needs an inner source");
  }

  if (!std::isfinite(max_distance_m) || max_distance_m <= 0.0) {
    throw std::invalid_argument("masking source needs a positive finite max distance");
  }
}

std::optional<RayFrame> MaskingRaySource::nextFrame()
{
  auto frame = _inner->nextFrame();

  if (frame.has_value()) {
    for (auto & ray : frame->rays) {
      _mask.apply(ray, _max_distance_m);
    }
  }

  return frame;
}

}  // namespace livox_ray_source
