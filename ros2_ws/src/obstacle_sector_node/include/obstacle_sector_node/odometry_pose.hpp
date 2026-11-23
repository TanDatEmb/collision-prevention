#pragma once

#include <cstdint>
#include <optional>

#include "livox_ray_source/pose_history.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

namespace obstacle_sector_node
{

/// Converts a PX4 `VehicleOdometry` into a pose sample in the time base of the rays (ICD-CP-06, ADR 0003,
/// ADR 0019). `timestamp_sample` is PX4 microseconds; in both timestamp profiles of ADR 0003 (simulation with
/// `UXRCE_DDS_SYNCT=0`, target with `UXRCE_DDS_SYNCT=1` and PTP) it is already in the clock of the rays, so it is
/// scaled to nanoseconds and not mapped.
///
/// Returns std::nullopt, so that the sample is not stored at all, when `timestamp_sample` is 0 (unknown) or is
/// ahead of `now_ns` (the node's own clock): a pose from the future cannot be a real sample. A sample whose pose
/// frame is unknown is returned with a NaN quaternion, which PoseHistory stores as an invalid sample, so that a
/// query across it gets no pose. The quaternion and position of a valid sample are copied unchanged; NaN values
/// from PX4 make the sample invalid in the same way.
[[nodiscard]] std::optional<livox_ray_source::PoseSample> poseFromOdometry(
  const px4_msgs::msg::VehicleOdometry & odometry, int64_t now_ns);

}  // namespace obstacle_sector_node
