#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "livox_ray_source/gyro_track.hpp"
#include "livox_ray_source/pose_history.hpp"
#include "obstacle_sector_map/types.hpp"

namespace livox_ray_source
{

// Frame conventions of the leveling and deskew transform (ADR 0020 D1, ICD-CP-06). A mix-up of
// these mirrors lateral obstacles or flips the height band, so they are fixed here once.
//
// - Sensor frame (FLU): x forward along the sensor boresight, y left, z up. This is the frame
//   of `decodeSphericalPacket` and of the self-mask: azimuth is counter-clockwise from +x
//   (towards +y, the left), elevation is positive upwards. A point is
//   `range * (cos(el) cos(az), cos(el) sin(az), sin(el))`.
// - Body frame: FLU for the mount (x forward, y left, z up) and FRD for the pose (x forward,
//   y right, z down). They differ by negating y and z: FRD = (x, -y, -z).
// - Pose: `PoseSample.q_wxyz` rotates body FRD to local NED (PX4 `VehicleOdometry.q`).
// - Output frame (FLU, ICD-CP-06): origin at the sensor position at the reference time, x
//   forward along the reference heading, y LEFT, z UP (against gravity), and azimuth
//   counter-clockwise from +x (left is positive), elevation positive upwards. Roll and pitch are
//   removed (z is gravity-aligned); only the heading at the reference time remains.
//
// Mount rotation (sensor FLU to body FLU, `LevelingConfig` roll, pitch, yaw): R = Rz(yaw) * Ry(pitch) * Rx(roll) with right-hand
// rotations about the FLU axes of the body: Rx about forward (positive: left side up, right
// side down), Ry about left (positive: the boresight tilts DOWN, so a sensor ray along +x
// points below the body horizon), Rz about up (positive: the boresight turns to the left).
// The lever arm `LevelingConfig::lever_arm_flu_m` is the sensor position in the body frame. The ray origin is
// the sensor, so the origin motion between two times is x_i + R_i L - x_ref - R_ref L (ADR 0022 decision 1).
/// Required parameters of RayLeveler. There are no defaults: a default-constructed config holds
/// the sentinels (NaN) and is not valid. The mount comes from the calibrated platform
/// configuration (ADR 0020 names 10 degrees pitched down for the first platform); the
/// lever arm comes from the platform file as well. The mount rotation is
/// R = Rz(yaw) * Ry(pitch) * Rx(roll) from the sensor FLU frame to the body FLU frame (see the
/// frame conventions above for the sign of each angle).
struct LevelingConfig
{
  /// Mount rotation about the forward axis, radians, positive = left side up. Sentinel NaN.
  double roll_rad{std::numeric_limits<double>::quiet_NaN()};
  /// Mount rotation about the left axis, radians, positive = boresight tilted down. Sentinel NaN.
  double pitch_down_rad{std::numeric_limits<double>::quiet_NaN()};
  /// Mount rotation about the up axis, radians, positive = boresight turned left. Sentinel NaN.
  double yaw_left_rad{std::numeric_limits<double>::quiet_NaN()};
  /// Position of the sensor from the body origin, body FLU, metres (the lever arm, ADR 0022 decision 1 item d).
  /// Sentinel NaN. Valid: finite.
  std::array<double, 3> lever_arm_flu_m{
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  /// Valid: the three mount angles are finite within [-pi, pi] and the lever arm is finite.
  [[nodiscard]] bool isValid() const;
};

enum class LevelingStatus : uint8_t
{
  kOk,
  kNoReference,           ///< beginFrame() was not called or did not succeed
  kNoPose,                ///< no FMU pose at the reference time, or no gyro cover or velocity for the ray time
  kResetSinceReference,   ///< the gyro restarted (gap or time step) between the reference and the ray time
  kDegenerateReturn,      ///< the origin motion moved a return's end point onto the reference origin
                          ///< (range 0): the ray has no direction
  kInvalidRay,            ///< no time, non-finite direction, a return without a finite positive range, or
                          ///< return kind kInvalid: left unchanged, the map rejects it anyway
};

/// True when a ray with this status was dropped for lack of evidence, so the caller must mark the
/// frame discontinuous (`packet_continuity_valid` false, ICD-CP-06) and no sector of the frame
/// becomes clear. kInvalidRay is not included: the sector map counts it as rejected as before.
/// Every status except kOk and kInvalidRay drops the ray: a dropped ray may have been the obstacle
/// (ADR 0018, ADR 0022 decision 1 item f).
[[nodiscard]] constexpr bool breaksFrameContinuity(LevelingStatus status)
{
  return status != LevelingStatus::kOk && status != LevelingStatus::kInvalidRay;
}

/// The leveling and deskew transform of ADR 0020 D1 for the rays of one frame, with the pose sources of
/// ADR 0023: the FMU pose only at the reference time t_ref (the map timestamp: the earliest contributing
/// ray time) and the Livox gyro for the rotation inside the frame.
///
/// For a ray at time t_i:
///   1. sensor point from range, azimuth, elevation (sensor FLU);
///   2. dR, the rotation of the sensor from t_ref to t_i from the gyro (the identity when t_i == t_ref);
///   3. calibrated mount rotation M to the body frame (FLU), then F, FLU to FRD;
///   4. q_fmu(t_ref) rotates body FRD to NED: p_ned = q_fmu(t_ref) * F * M * dR * p_sensor, so the body
///      attitude at t_i is R_i = q_fmu(t_ref) * F * M * dR * M^T * F;
///   5. for a return, add the origin motion d_i = x_i + R_i L - x_ref - R_ref L (NED, L the lever arm), with
///      x_i = x_ref + v_ref * (t_i - t_ref) from the FMU velocity at t_ref; a no-return has no end point and
///      keeps its direction only. The vertical part of d_i is reported to the caller (`origin_rise_m`),
///      which bounds it for the sector map;
///   6. rotate about down by minus the heading at t_ref (heading-aligned FRD);
///   7. negate right and down (FRD to FLU).
/// The ray then carries azimuth, elevation and range in the output frame documented at the top of this
/// file; `timestamp_ns` stays the original sensor time. Nothing else is changed.
///
/// A ray with t_i != t_ref needs the gyro (orientations at t_ref and t_i in one segment) and a finite FMU
/// velocity at t_ref; otherwise it gets kNoPose (or kResetSinceReference when the gyro restarted between
/// the two times).
///
/// Not thread safe; use one instance per frame-building thread. `poses` and `gyro` must outlive it.
class RayLeveler
{
public:
  /// Throws std::invalid_argument when `config` is not valid. `gyro` may be null for a source whose rays
  /// of one frame all carry the reference time (the Gazebo cloud).
  RayLeveler(
    const PoseHistory & poses, const LevelingConfig & config, const GyroTrack * gyro = nullptr);

  /// Sets the reference time (the map timestamp) and looks up the FMU pose there: attitude, position,
  /// velocity and heading. Returns kOk, or kNoPose when there is no usable pose at that time (no pose, or
  /// an attitude whose forward axis is vertical so that it has no heading). On failure every level() call
  /// returns kNoReference until a call succeeds.
  LevelingStatus beginFrame(uint64_t reference_time_ns);

  /// Transforms `ray` in place. On any status other than kOk the ray is left unchanged and must
  /// be dropped by the caller (see breaksFrameContinuity()). On kOk `origin_rise_m` is set to how
  /// far the sensor at the ray time is above the sensor at the reference time, metres (negative
  /// when it is below).
  [[nodiscard]] LevelingStatus level(
    obstacle_sector_map::RaySample & ray, double & origin_rise_m) const;

private:
  const PoseHistory & _poses;
  const GyroTrack * _gyro;
  LevelingConfig _config;
  std::array<double, 9> _mount_rotation;  // sensor FLU to body FLU, row-major
  std::array<double, 9> _sensor_to_frd;   // F * M: sensor FLU to body FRD, row-major
  bool _has_reference{false};
  uint64_t _reference_time_ns{0U};
  std::array<double, 9> _reference_attitude{};  // body FRD to NED at t_ref, row-major
  std::array<double, 3> _reference_position_ned_m{};
  std::array<double, 3> _reference_velocity_ned_m_s{};
  std::array<double, 3> _reference_sensor_position_ned_m{};
  double _reference_heading_rad{0.0};
};

}  // namespace livox_ray_source
