#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "livox_ray_source/pose_history.hpp"
#include "obstacle_sector_map/types.hpp"

namespace livox_ray_source
{

/// Frame conventions of the leveling and deskew transform (ADR 0020 D1, ICD-CP-06). A mix-up of
/// these mirrors lateral obstacles or flips the height band, so they are fixed here once.
///
/// - Sensor frame (FLU): x forward along the sensor boresight, y left, z up. This is the frame
///   of `decodeSphericalPacket` and of the self-mask: azimuth is counter-clockwise from +x
///   (towards +y, the left), elevation is positive upwards. A point is
///   `range * (cos(el) cos(az), cos(el) sin(az), sin(el))`.
/// - Body frame: FLU for the mount (x forward, y left, z up) and FRD for the pose (x forward,
///   y right, z down). They differ by negating y and z: FRD = (x, -y, -z).
/// - Pose: `PoseSample.q_wxyz` rotates body FRD to local NED (PX4 `VehicleOdometry.q`).
/// - Output frame (FLU, ICD-CP-06): origin at the vehicle position at the reference time, x
///   forward along the reference heading, y LEFT, z UP (against gravity), and azimuth
///   counter-clockwise from +x (left is positive), elevation positive upwards. Roll and pitch are
///   removed (z is gravity-aligned); only the heading at the reference time remains.
///
/// Mount rotation (sensor FLU to body FLU): R = Rz(yaw) * Ry(pitch) * Rx(roll) with right-hand
/// rotations about the FLU axes of the body: Rx about forward (positive: left side up, right
/// side down), Ry about left (positive: the boresight tilts DOWN, so a sensor ray along +x
/// points below the body horizon), Rz about up (positive: the boresight turns to the left).
/// The lever arm between the body origin and the sensor is not modelled (the ray origin is the
/// body origin).
struct MountRotation
{
  /// Rotation about the forward axis, radians. Sentinel NaN.
  double roll_rad{std::numeric_limits<double>::quiet_NaN()};
  /// Rotation about the left axis, radians, positive = boresight tilted down. Sentinel NaN.
  double pitch_down_rad{std::numeric_limits<double>::quiet_NaN()};
  /// Rotation about the up axis, radians, positive = boresight turned left. Sentinel NaN.
  double yaw_left_rad{std::numeric_limits<double>::quiet_NaN()};

  /// Valid: every angle is finite and within [-pi, pi].
  [[nodiscard]] bool isValid() const;
};

/// Required parameters of RayLeveler. There are no defaults: a default-constructed config holds
/// the sentinels (NaN) and is not valid. The mount comes from the calibrated platform
/// configuration (ADR 0020 names 10 degrees pitched down for the first platform); the
/// origin-motion threshold is OPEN-031 and has no approved value yet.
struct LevelingConfig
{
  MountRotation mount{};
  /// A ray is dropped when the distance between the vehicle position at the ray time and at the
  /// reference time exceeds this, metres. Sentinel NaN. Valid: finite and greater than 0.
  double origin_motion_threshold_m{std::numeric_limits<double>::quiet_NaN()};

  [[nodiscard]] bool isValid() const
  {
    return mount.isValid() && std::isfinite(origin_motion_threshold_m) &&
           origin_motion_threshold_m > 0.0;
  }
};

enum class LevelingStatus : uint8_t
{
  kOk,
  kNoReference,           ///< beginFrame() was not called or did not succeed
  kNoPose,                ///< no pose for the ray time (or, from beginFrame, the reference time)
  kResetSinceReference,   ///< the pose reset counter differs from that of the reference pose
  kOriginMotionExceeded,  ///< the vehicle moved more than the threshold since the reference time
  kInvalidRay,            ///< no time, non-finite direction, a return without a finite range, or
                          ///< return kind kInvalid: left unchanged, the map rejects it anyway
};

/// True when a ray with this status was dropped for lack of evidence, so the caller must mark the
/// frame discontinuous (`packet_continuity_valid` false, ICD-CP-06) and no sector of the frame
/// becomes clear. kInvalidRay is not included: the sector map counts it as rejected as before.
/// kOriginMotionExceeded is included as the conservative choice: a dropped ray may have been the
/// obstacle (ADR 0018); the owner has not decided it (OPEN-031).
[[nodiscard]] constexpr bool breaksFrameContinuity(LevelingStatus status)
{
  return status != LevelingStatus::kOk && status != LevelingStatus::kInvalidRay;
}

/// The leveling and deskew transform of ADR 0020 D1 for the rays of one frame.
///
/// For a ray at time t_i and a reference time t_ref (the map timestamp: the earliest contributing
/// ray time):
///   1. sensor point from range, azimuth, elevation (sensor FLU);
///   2. calibrated mount rotation to the body frame (FLU), then FLU to FRD;
///   3. q(t_i) rotates body FRD to NED;
///   4. for a return, add the origin motion x(t_i) - x(t_ref) (NED); a no-return has no end point
///      and keeps its direction only;
///   5. rotate about down by minus the heading at t_ref (heading-aligned FRD);
///   6. negate right and down (FRD to FLU).
/// The ray then carries azimuth, elevation and range in the output frame documented at
/// MountRotation; `timestamp_ns` stays the original sensor time. Nothing else is changed.
///
/// Not thread safe; use one instance per frame-building thread. `poses` must outlive it.
class RayLeveler
{
public:
  /// Throws std::invalid_argument when `config` is not valid.
  RayLeveler(const PoseProvider & poses, const LevelingConfig & config);

  /// Sets the reference time (the map timestamp) and looks up the reference pose: position and
  /// heading. Returns kOk, or kNoPose when there is no usable pose at that time (no pose, or an
  /// attitude whose forward axis is vertical so that it has no heading). On failure every
  /// level() call returns kNoReference until a call succeeds.
  LevelingStatus beginFrame(uint64_t reference_time_ns);

  /// Transforms `ray` in place. On any status other than kOk the ray is left unchanged and must
  /// be dropped by the caller (see breaksFrameContinuity()).
  [[nodiscard]] LevelingStatus level(obstacle_sector_map::RaySample & ray) const;

private:
  const PoseProvider & _poses;
  LevelingConfig _config;
  std::array<double, 9> _mount_rotation;  // sensor FLU to body FLU, row-major
  bool _has_reference{false};
  std::array<double, 3> _reference_position_ned_m{};
  double _reference_heading_rad{0.0};
  uint8_t _reference_reset_counter{0U};
};

}  // namespace livox_ray_source
