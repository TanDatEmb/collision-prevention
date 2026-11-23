#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "livox_ray_source/pose_history.hpp"
#include "livox_ray_source/ray_leveling.hpp"
#include "livox_ray_source/ray_source.hpp"

namespace livox_ray_source
{

/// Counts for the log of the node: what the decorator dropped, so that the need for a wait for a newer pose
/// (ADR 0022 decision 2, "measure first") can be read from a run.
struct LevelingStats
{
  uint64_t frames_without_reference{0U};  ///< frames emptied because no pose exists at the reference time
  uint64_t rays_dropped{0U};              ///< rays removed because the leveler dropped them (no pose, reset, ...)
};

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

  /// Not thread safe: read it from the thread that calls nextFrame().
  [[nodiscard]] const LevelingStats & stats() const {return _stats;}

private:
  LevelingStats _stats;
  std::unique_ptr<RaySource> _inner;
  RayLeveler _leveler;
};

}  // namespace livox_ray_source
