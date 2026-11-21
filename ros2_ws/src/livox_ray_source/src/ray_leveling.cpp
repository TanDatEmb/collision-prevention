#include "livox_ray_source/ray_leveling.hpp"

#include <stdexcept>

namespace livox_ray_source
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// Numerical guard, not a safety threshold: the horizontal part of the body forward axis is below
// the single precision accuracy of the PX4 quaternion, so the heading is noise. Listed for
// approval in the pull request.
constexpr double kMinForwardHorizontalProjection = 1.0e-6;

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;  // row-major

Mat3 multiply(const Mat3 & a, const Mat3 & b)
{
  Mat3 c{};
  for (std::size_t i = 0U; i < 3U; ++i) {
    for (std::size_t j = 0U; j < 3U; ++j) {
      for (std::size_t k = 0U; k < 3U; ++k) {
        c[3U * i + j] += a[3U * i + k] * b[3U * k + j];
      }
    }
  }
  return c;
}

Vec3 multiply(const Mat3 & a, const Vec3 & v)
{
  return {
    a[0] * v[0] + a[1] * v[1] + a[2] * v[2], a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
    a[6] * v[0] + a[7] * v[1] + a[8] * v[2]};
}

// Right-hand rotations about the x, y and z axis of a right-handed frame.
Mat3 rotationX(double a)
{
  return {1.0, 0.0, 0.0, 0.0, std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a)};
}

Mat3 rotationY(double a)
{
  return {std::cos(a), 0.0, std::sin(a), 0.0, 1.0, 0.0, -std::sin(a), 0.0, std::cos(a)};
}

Mat3 rotationZ(double a)
{
  return {std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a), 0.0, 0.0, 0.0, 1.0};
}

// Rotation matrix of a Hamilton quaternion (w, x, y, z); normalized here.
Mat3 rotationOf(const std::array<double, 4> & q_in)
{
  const double n = std::sqrt(q_in[0] * q_in[0] + q_in[1] * q_in[1] + q_in[2] * q_in[2] +
    q_in[3] * q_in[3]);
  const double w = q_in[0] / n;
  const double x = q_in[1] / n;
  const double y = q_in[2] / n;
  const double z = q_in[3] / n;
  return {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
    2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
    2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)};
}

bool angleIsValid(double a)
{
  return std::isfinite(a) && a >= -kPi && a <= kPi;
}

}  // namespace

bool MountRotation::isValid() const
{
  return angleIsValid(roll_rad) && angleIsValid(pitch_down_rad) && angleIsValid(yaw_left_rad);
}

RayLeveler::RayLeveler(const PoseProvider & poses, const LevelingConfig & config)
: _poses(poses), _config(config), _mount_rotation{}
{
  if (!config.isValid()) {
    throw std::invalid_argument(
            "LevelingConfig: the mount angles must be finite within [-pi, pi] and "
            "origin_motion_threshold_m finite and above 0");
  }
  // Sensor FLU to body FLU: Rz(yaw) * Ry(pitch) * Rx(roll).
  _mount_rotation = multiply(
    rotationZ(config.mount.yaw_left_rad),
    multiply(rotationY(config.mount.pitch_down_rad), rotationX(config.mount.roll_rad)));
}

LevelingStatus RayLeveler::beginFrame(uint64_t reference_time_ns)
{
  _has_reference = false;
  const PoseLookupResult lookup = _poses.poseAt(reference_time_ns);
  if (!lookup.ok() || !lookup.pose.isValid()) {
    return LevelingStatus::kNoPose;
  }
  // Heading: direction of the body forward axis (the first column of the body FRD to NED
  // rotation) projected on the horizontal plane, clockwise from north (the Euler yaw).
  const Mat3 r = rotationOf(lookup.pose.q_wxyz);
  const double north = r[0];
  const double east = r[3];
  if (std::hypot(north, east) < kMinForwardHorizontalProjection) {
    return LevelingStatus::kNoPose;
  }
  _reference_heading_rad = std::atan2(east, north);
  _reference_position_ned_m = lookup.pose.position_ned_m;
  _reference_reset_counter = lookup.pose.reset_counter;
  _has_reference = true;
  return LevelingStatus::kOk;
}

LevelingStatus RayLeveler::level(obstacle_sector_map::RaySample & ray) const
{
  using obstacle_sector_map::RayReturnKind;

  const bool is_return = ray.return_kind == RayReturnKind::kReturn;
  const bool is_no_return = ray.return_kind == RayReturnKind::kQualifiedNoReturn;
  if (ray.timestamp_ns == 0U || (!is_return && !is_no_return) ||
    !std::isfinite(ray.azimuth_rad) || !std::isfinite(ray.elevation_rad) ||
    (is_return && !std::isfinite(ray.range_m)))
  {
    return LevelingStatus::kInvalidRay;
  }
  if (!_has_reference) {
    return LevelingStatus::kNoReference;
  }

  const PoseLookupResult lookup = _poses.poseAt(ray.timestamp_ns);
  if (!lookup.ok() || !lookup.pose.isValid()) {
    return LevelingStatus::kNoPose;
  }
  if (lookup.pose.reset_counter != _reference_reset_counter) {
    return LevelingStatus::kResetSinceReference;
  }
  // Origin motion x(t_i) - x(t_ref), NED.
  const Vec3 motion_ned_m{
    lookup.pose.position_ned_m[0] - _reference_position_ned_m[0],
    lookup.pose.position_ned_m[1] - _reference_position_ned_m[1],
    lookup.pose.position_ned_m[2] - _reference_position_ned_m[2]};
  const double motion_m =
    std::sqrt(motion_ned_m[0] * motion_ned_m[0] + motion_ned_m[1] * motion_ned_m[1] +
      motion_ned_m[2] * motion_ned_m[2]);
  if (motion_m > _config.origin_motion_threshold_m) {
    return LevelingStatus::kOriginMotionExceeded;
  }

  // 1. Sensor point (FLU); a no-return has a direction only (unit length).
  const double range_m = is_return ? static_cast<double>(ray.range_m) : 1.0;
  const double az = static_cast<double>(ray.azimuth_rad);
  const double el = static_cast<double>(ray.elevation_rad);
  const Vec3 p_sensor_flu{
    range_m * std::cos(el) * std::cos(az), range_m * std::cos(el) * std::sin(az),
    range_m * std::sin(el)};
  // 2. Mount: sensor FLU to body FLU, then body FLU to body FRD (negate left -> right, up -> down).
  const Vec3 p_body_flu = multiply(_mount_rotation, p_sensor_flu);
  const Vec3 p_body_frd{p_body_flu[0], -p_body_flu[1], -p_body_flu[2]};
  // 3. Attitude at the ray time: body FRD to NED.
  Vec3 p_ned_m = multiply(rotationOf(lookup.pose.q_wxyz), p_body_frd);
  // 4. Move the ray origin to the reference position (an end point only; a direction has none).
  if (is_return) {
    for (std::size_t i = 0U; i < 3U; ++i) {
      p_ned_m[i] += motion_ned_m[i];
    }
  }
  // 5. Heading-aligned FRD: rotate about down by minus the reference heading.
  const double c = std::cos(_reference_heading_rad);
  const double s = std::sin(_reference_heading_rad);
  const Vec3 p_heading_frd{c * p_ned_m[0] + s * p_ned_m[1], -s * p_ned_m[0] + c * p_ned_m[1],
    p_ned_m[2]};
  // 6. Output frame FLU: negate right and down.
  const Vec3 p_flu{p_heading_frd[0], -p_heading_frd[1], -p_heading_frd[2]};

  ray.azimuth_rad = static_cast<float>(std::atan2(p_flu[1], p_flu[0]));
  ray.elevation_rad = static_cast<float>(std::atan2(p_flu[2], std::hypot(p_flu[0], p_flu[1])));
  if (is_return) {
    ray.range_m = static_cast<float>(
      std::sqrt(p_flu[0] * p_flu[0] + p_flu[1] * p_flu[1] + p_flu[2] * p_flu[2]));
  }
  return LevelingStatus::kOk;
}

}  // namespace livox_ray_source
