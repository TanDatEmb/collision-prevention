#include "livox_ray_source/leveling_ray_source.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace livox_ray_source
{

LevelingRaySource::LevelingRaySource(
  std::unique_ptr<RaySource> inner, const PoseHistory & poses, const LevelingConfig & config)
: _inner(std::move(inner)), _leveler(poses, config)
{
  if (!_inner) {
    throw std::invalid_argument("leveling source needs an inner source");
  }
}

std::optional<RayFrame> LevelingRaySource::nextFrame()
{
  auto frame = _inner->nextFrame();
  if (!frame.has_value()) {
    return frame;
  }

  // The reference time is the earliest ray time; a ray without a time (0) cannot be a reference
  // and the leveler reports it as invalid.
  uint64_t reference_ns = 0U;
  for (const auto & ray : frame->rays) {
    if (ray.timestamp_ns != 0U && (reference_ns == 0U || ray.timestamp_ns < reference_ns)) {
      reference_ns = ray.timestamp_ns;
    }
  }
  if (reference_ns == 0U) {
    return frame;  // nothing to level; the sector map rejects these rays as before
  }

  if (_leveler.beginFrame(reference_ns) != LevelingStatus::kOk) {
    frame->rays.clear();
    frame->packet_continuity_valid = false;
    return frame;
  }

  std::size_t kept = 0U;
  double rise_bound_m = 0.0;
  double fall_bound_m = 0.0;
  for (auto & ray : frame->rays) {
    double origin_rise_m = 0.0;
    const LevelingStatus status = _leveler.level(ray, origin_rise_m);
    if (breaksFrameContinuity(status)) {
      frame->packet_continuity_valid = false;
      continue;
    }
    if (status == LevelingStatus::kOk) {
      rise_bound_m = std::max(rise_bound_m, origin_rise_m);
      fall_bound_m = std::max(fall_bound_m, -origin_rise_m);
    }
    if (&frame->rays[kept] != &ray) {
      frame->rays[kept] = ray;
    }
    ++kept;
  }
  frame->rays.resize(kept);
  frame->origin_rise_bound_m = rise_bound_m;
  frame->origin_fall_bound_m = fall_bound_m;
  return frame;
}

}  // namespace livox_ray_source
