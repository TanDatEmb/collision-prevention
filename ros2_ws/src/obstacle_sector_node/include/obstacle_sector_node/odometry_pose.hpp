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
/// ahead of `now_ns` (the node's own clock): a pose from the future cannot be a real sample. The quaternion and
/// position are copied unchanged; the velocity only when `velocity_frame` is NED, otherwise it stays NaN
/// (unknown), which the leveler accepts only for rays at the reference time (ADR 0023). A NaN from PX4 makes the sample invalid, and PoseHistory stores it as an invalid
/// sample, so that a query across it gets no pose. The pose frame (NED or FRD) is not read: the leveling step only
/// uses heading and position differences, which a constant heading offset does not change (PoseSample).
[[nodiscard]] std::optional<livox_ray_source::PoseSample> poseFromOdometry(
  const px4_msgs::msg::VehicleOdometry & odometry, int64_t now_ns);

}  // namespace obstacle_sector_node
