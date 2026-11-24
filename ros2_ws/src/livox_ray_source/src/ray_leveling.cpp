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

Mat3 transpose(const Mat3 & a)
{
  return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
}

// F: body FLU to body FRD (negate left and up).
Mat3 fluToFrd()
{
  return {1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0};
}

bool angleIsValid(double a)
{
  return std::isfinite(a) && a >= -kPi && a <= kPi;
}

}  // namespace

bool LevelingConfig::isValid() const
{
  return angleIsValid(roll_rad) && angleIsValid(pitch_down_rad) && angleIsValid(yaw_left_rad) &&
         std::isfinite(lever_arm_flu_m[0]) && std::isfinite(lever_arm_flu_m[1]) &&
         std::isfinite(lever_arm_flu_m[2]);
}

std::array<double, 9> mountRotation(const LevelingConfig & config)
{
  // Sensor FLU to body FLU: Rz(yaw) * Ry(pitch) * Rx(roll).
  return multiply(
    rotationZ(config.yaw_left_rad),
    multiply(rotationY(config.pitch_down_rad), rotationX(config.roll_rad)));
}

RayLeveler::RayLeveler(
  const PoseHistory & poses, const LevelingConfig & config, const GyroTrack * gyro)
: _poses(poses), _gyro(gyro), _config(config), _mount_rotation{}, _sensor_to_frd{}
{
  if (!config.isValid()) {
    throw std::invalid_argument(
            "LevelingConfig: the mount angles must be finite within [-pi, pi] and the lever arm finite");
  }
  _mount_rotation = mountRotation(config);
  _sensor_to_frd = multiply(fluToFrd(), _mount_rotation);
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
  _reference_time_ns = reference_time_ns;
  _reference_attitude = r;
  _reference_position_ned_m = lookup.pose.position_ned_m;
  _reference_velocity_ned_m_s = lookup.pose.velocity_ned_m_s;
  _reference_heading_rad = std::atan2(east, north);
  // x_ref + R_ref L, with the lever arm FLU -> FRD.
  const Vec3 l_frd{_config.lever_arm_flu_m[0], -_config.lever_arm_flu_m[1], -_config.lever_arm_flu_m[2]};
  const Vec3 l_ned = multiply(r, l_frd);
  for (std::size_t i = 0U; i < 3U; ++i) {
    _reference_sensor_position_ned_m[i] = _reference_position_ned_m[i] + l_ned[i];
  }
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
    (is_return && !(std::isfinite(ray.range_m) && ray.range_m > 0.0F)))
  {
    return LevelingStatus::kInvalidRay;
  }
  if (!_has_reference) {
    return LevelingStatus::kNoReference;
  }

  // Attitude (body FRD to NED) and body position at the ray time (ADR 0023 decisions 1 and 3).
  Mat3 attitude = _reference_attitude;
  Vec3 position_ned_m = _reference_position_ned_m;
  if (ray.timestamp_ns != _reference_time_ns) {
    if (_gyro == nullptr) {
      return LevelingStatus::kNoPose;
    }
    const PoseLookupResult at_reference = _gyro->orientationAt(_reference_time_ns);
    const PoseLookupResult at_ray = _gyro->orientationAt(ray.timestamp_ns);
    if (!at_reference.ok() || !at_ray.ok()) {
      return LevelingStatus::kNoPose;
    }
    if (at_reference.pose.reset_counter != at_ray.pose.reset_counter) {
      return LevelingStatus::kResetSinceReference;
    }
    for (const double v : _reference_velocity_ned_m_s) {
      if (!std::isfinite(v)) {
        return LevelingStatus::kNoPose;
      }
    }
    // dR maps the sensor frame at t_i to the sensor frame at t_ref; R_i = R_ref (F M) dR (F M)^T.
    const Mat3 rotation_inside_frame = multiply(
      transpose(rotationOf(at_reference.pose.q_wxyz)), rotationOf(at_ray.pose.q_wxyz));
    attitude = multiply(
      _reference_attitude,
      multiply(_sensor_to_frd, multiply(rotation_inside_frame, transpose(_sensor_to_frd))));
    // Signed time difference: a masked ray can be earlier than the reference.
    const double dt_s = ray.timestamp_ns > _reference_time_ns ?
      static_cast<double>(ray.timestamp_ns - _reference_time_ns) * 1.0e-9 :
      -static_cast<double>(_reference_time_ns - ray.timestamp_ns) * 1.0e-9;
    for (std::size_t i = 0U; i < 3U; ++i) {
      position_ned_m[i] += _reference_velocity_ned_m_s[i] * dt_s;
    }
  }

  // Origin motion of the sensor, d_i = x_i + R_i L - x_ref - R_ref L, NED. A return is shifted exactly below,
  // whatever the motion; it is never dropped for it (ADR 0022 decision 1 item a).
  const Vec3 l_frd{_config.lever_arm_flu_m[0], -_config.lever_arm_flu_m[1], -_config.lever_arm_flu_m[2]};
  const Vec3 l_ned = multiply(attitude, l_frd);
  const Vec3 motion_ned_m{
    position_ned_m[0] + l_ned[0] - _reference_sensor_position_ned_m[0],
    position_ned_m[1] + l_ned[1] - _reference_sensor_position_ned_m[1],
    position_ned_m[2] + l_ned[2] - _reference_sensor_position_ned_m[2]};

  // 1. Sensor point (FLU); a no-return has a direction only (unit length).
  const double range_m = is_return ? static_cast<double>(ray.range_m) : 1.0;
  const double az = static_cast<double>(ray.azimuth_rad);
  const double el = static_cast<double>(ray.elevation_rad);
  const Vec3 p_sensor_flu{
    range_m * std::cos(el) * std::cos(az), range_m * std::cos(el) * std::sin(az),
    range_m * std::sin(el)};
  // 2-4. Mount and FLU to FRD (F M), then the attitude at the ray time: body FRD to NED.
  Vec3 p_ned_m = multiply(attitude, multiply(_sensor_to_frd, p_sensor_flu));
  // 5. Move the ray origin to the reference position (an end point only; a direction has none).
  if (is_return) {
    for (std::size_t i = 0U; i < 3U; ++i) {
      p_ned_m[i] += motion_ned_m[i];
    }
  }
  // 6. Heading-aligned FRD: rotate about down by minus the reference heading.
  const double c = std::cos(_reference_heading_rad);
  const double s = std::sin(_reference_heading_rad);
  const Vec3 p_heading_frd{c * p_ned_m[0] + s * p_ned_m[1], -s * p_ned_m[0] + c * p_ned_m[1],
    p_ned_m[2]};
  // 7. Output frame FLU: negate right and down.
  const Vec3 p_flu{p_heading_frd[0], -p_heading_frd[1], -p_heading_frd[2]};

  if (is_return) {
    const float out_range_m = static_cast<float>(
      std::sqrt(p_flu[0] * p_flu[0] + p_flu[1] * p_flu[1] + p_flu[2] * p_flu[2]));
    // The origin motion can move the end point onto the reference origin. A range that is not
    // strictly positive has no direction and the sector map rejects it, so the evidence would be
    // lost silently: drop the ray, which breaks frame continuity. A tiny positive range stays a
    // valid return (the map reports it as too close).
    if (!(std::isfinite(out_range_m) && out_range_m > 0.0F)) {
      return LevelingStatus::kDegenerateReturn;
    }
    ray.range_m = out_range_m;
  }
  ray.azimuth_rad = static_cast<float>(std::atan2(p_flu[1], p_flu[0]));
  ray.elevation_rad = static_cast<float>(std::atan2(p_flu[2], std::hypot(p_flu[0], p_flu[1])));
  return LevelingStatus::kOk;
}

}  // namespace livox_ray_source
