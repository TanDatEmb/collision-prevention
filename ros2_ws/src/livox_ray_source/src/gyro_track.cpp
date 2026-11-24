#include "livox_ray_source/gyro_track.hpp"

#include <cmath>
#include <stdexcept>

namespace livox_ray_source
{

namespace
{

using Quaternion = std::array<double, 4>;  // w, x, y, z

Quaternion multiply(const Quaternion & a, const Quaternion & b)
{
  return {
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

// Quaternion of the rotation vector `angle` (axis times angle, radians).
Quaternion fromRotationVector(const std::array<double, 3> & angle)
{
  const double theta = std::sqrt(angle[0] * angle[0] + angle[1] * angle[1] + angle[2] * angle[2]);
  const double half = 0.5 * theta;
  // sin(half) / theta, with its limit 1/2 for a vanishing angle.
  const double k = theta > 1.0e-12 ? std::sin(half) / theta : 0.5;
  return {std::cos(half), k * angle[0], k * angle[1], k * angle[2]};
}

Quaternion normalized(const Quaternion & q)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

// The sensor turns at the body rate omega, so R(t + dt) = R(t) Exp(omega dt): the increment multiplies on
// the right.
Quaternion advance(const Quaternion & q, const std::array<double, 3> & rate_rad_s, double dt_s)
{
  return normalized(multiply(
    q, fromRotationVector({rate_rad_s[0] * dt_s, rate_rad_s[1] * dt_s, rate_rad_s[2] * dt_s})));
}

PoseSample orientationSample(uint64_t time_ns, const Quaternion & q, uint8_t segment)
{
  PoseSample sample;
  sample.timestamp_sample_ns = time_ns;
  sample.q_wxyz = q;
  sample.position_ned_m = {0.0, 0.0, 0.0};
  sample.reset_counter = segment;
  return sample;
}

}  // namespace

bool GyroSample::isValid() const
{
  return timestamp_ns != 0U && std::isfinite(rate_flu_rad_s[0]) && std::isfinite(rate_flu_rad_s[1]) &&
         std::isfinite(rate_flu_rad_s[2]);
}

GyroTrack::GyroTrack(const PoseHistoryConfig & config)
: _config(config)
{
  if (!config.isValid()) {
    throw std::invalid_argument("GyroTrack: the depth must be at least 2 and the gap positive");
  }
  _history = std::make_unique<PoseHistory>(config);
}

bool GyroTrack::push(const GyroSample & sample)
{
  if (!sample.isValid()) {
    return false;
  }
  const std::lock_guard<std::mutex> lock(_mutex);
  if (_has_newest && sample.timestamp_ns == _newest.timestamp_ns) {
    return false;
  }
  const bool goes_back = _has_newest && sample.timestamp_ns < _newest.timestamp_ns;
  const bool restart = !_has_newest || goes_back ||
    sample.timestamp_ns - _newest.timestamp_ns > _config.max_bracket_gap_ns;
  if (restart) {
    if (_has_newest) {
      ++_segment;  // wraps; compared for inequality only
    }
    if (goes_back) {
      _history = std::make_unique<PoseHistory>(_config);  // the stored times are now in the future
    }
    _newest_q_wxyz = {1.0, 0.0, 0.0, 0.0};
  } else {
    // Trapezoid rule: the mean of the two rates over the step.
    const double dt_s = static_cast<double>(sample.timestamp_ns - _newest.timestamp_ns) * 1.0e-9;
    const std::array<double, 3> mean_rate{
      0.5 * (_newest.rate_flu_rad_s[0] + sample.rate_flu_rad_s[0]),
      0.5 * (_newest.rate_flu_rad_s[1] + sample.rate_flu_rad_s[1]),
      0.5 * (_newest.rate_flu_rad_s[2] + sample.rate_flu_rad_s[2])};
    _newest_q_wxyz = advance(_newest_q_wxyz, mean_rate, dt_s);
  }
  _newest = sample;
  _has_newest = true;
  return _history->push(orientationSample(sample.timestamp_ns, _newest_q_wxyz, _segment));
}

PoseLookupResult GyroTrack::orientationAt(uint64_t time_ns) const
{
  const std::lock_guard<std::mutex> lock(_mutex);
  PoseLookupResult result = _history->poseAt(time_ns);
  if (result.status != PoseLookupStatus::kNoSampleAfter || !_has_newest ||
    time_ns <= _newest.timestamp_ns || time_ns - _newest.timestamp_ns > _config.max_bracket_gap_ns)
  {
    return result;
  }
  // After the newest sample: hold its rate (ADR 0023 decision 2).
  const double dt_s = static_cast<double>(time_ns - _newest.timestamp_ns) * 1.0e-9;
  result.status = PoseLookupStatus::kOk;
  result.pose = orientationSample(time_ns, advance(_newest_q_wxyz, _newest.rate_flu_rad_s, dt_s), _segment);
  return result;
}

}  // namespace livox_ray_source
