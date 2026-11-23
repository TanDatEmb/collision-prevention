#include "livox_ray_source/leveling_ray_source.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "obstacle_sector_map/builder.hpp"

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

  // The reference time is the map timestamp: the earliest time among the rays the sector map takes
  // (ICD-CP-06). A masked or invalid ray is rejected there and must not set the reference.
  uint64_t reference_ns = 0U;
  for (const auto & ray : frame->rays) {
    if (obstacle_sector_map::SectorMapBuilder::accepts(ray) &&
      (reference_ns == 0U || ray.timestamp_ns < reference_ns))
    {
      reference_ns = ray.timestamp_ns;
    }
  }
  if (reference_ns == 0U) {
    return frame;  // nothing to level; the sector map rejects these rays as before
  }

  if (_leveler.beginFrame(reference_ns) != LevelingStatus::kOk) {
    ++_stats.frames_without_reference;
    _stats.rays_dropped += frame->rays.size();
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
      ++_stats.rays_dropped;
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
