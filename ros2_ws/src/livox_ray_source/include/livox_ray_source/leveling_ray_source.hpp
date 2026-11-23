#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "livox_ray_source/pose_history.hpp"
#include "livox_ray_source/ray_leveling.hpp"
#include "livox_ray_source/ray_source.hpp"

namespace livox_ray_source
{

/// Decorator that levels every ray of another source (ADR 0020 D1, ICD-CP-06) with RayLeveler.
/// The inner source must deliver rays in the sensor frame. For each frame the reference time is
/// the earliest ray time (the map timestamp). A ray the leveler drops (breaksFrameContinuity)
/// is removed and the frame becomes discontinuous. When there is no reference pose, every ray is
/// removed and the frame is discontinuous, so no sector of it becomes clear.
/// `poses` must outlive the source.
class LevelingRaySource : public RaySource
{
public:
  /// Throws std::invalid_argument when `inner` is null or `config` is not valid.
  LevelingRaySource(
    std::unique_ptr<RaySource> inner, const PoseHistory & poses, const LevelingConfig & config);

  [[nodiscard]] std::optional<RayFrame> nextFrame() override;

  /// Rays removed so far because the leveler dropped them (no pose, reset, ...); a frame with no reference pose
  /// counts all its rays. It is what ADR 0022 decision 2 asks to measure before a wait for a newer pose is added.
  /// Not thread safe: read it from the thread that calls nextFrame().
  [[nodiscard]] uint64_t raysDropped() const {return _rays_dropped;}

private:
  uint64_t _rays_dropped{0U};
  std::unique_ptr<RaySource> _inner;
  RayLeveler _leveler;
};

}  // namespace livox_ray_source
