#pragma once

#include <utility>

#include "livox_ray_source/pose_history.hpp"

namespace livox_ray_source
{

/// Test double of PoseProvider: returns the same result for every query time. With a valid pose
/// the vehicle is at rest (no origin motion, no interpolation); with a status other than kOk it
/// stands for a missing pose. It is meant for tests and for code that has no PX4 pose yet; a
/// real node uses PoseHistory.
class FixedPoseProvider : public PoseProvider
{
public:
  explicit FixedPoseProvider(PoseLookupResult result)
  : _result(std::move(result)) {}

  /// Always returns kOk with `pose` (its time is replaced by the query time).
  explicit FixedPoseProvider(const PoseSample & pose)
  : _result{PoseLookupStatus::kOk, pose} {}

  [[nodiscard]] PoseLookupResult poseAt(uint64_t time_ns) const override
  {
    PoseLookupResult result = _result;
    result.pose.timestamp_sample_ns = time_ns;
    return result;
  }

private:
  PoseLookupResult _result;
};

}  // namespace livox_ray_source
