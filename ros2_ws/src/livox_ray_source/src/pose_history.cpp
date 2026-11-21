#include "livox_ray_source/pose_history.hpp"

#include <algorithm>
#include <stdexcept>

namespace livox_ray_source
{

namespace
{

// Numerical guard of the spherical interpolation, not a safety threshold: when the two
// quaternions are this close to parallel, sin(theta) is too small to divide by and the
// normalized linear interpolation is exact to rounding.
constexpr double kParallelDotMin = 1.0 - 1.0e-12;

std::array<double, 4> normalized(const std::array<double, 4> & q)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

std::array<double, 4> slerp(
  const std::array<double, 4> & a, const std::array<double, 4> & b_in, double t)
{
  std::array<double, 4> b = b_in;
  double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0.0) {  // q and -q are the same attitude: take the shorter arc
    for (double & v : b) {v = -v;}
    dot = -dot;
  }
  double wa{1.0 - t};
  double wb{t};
  if (dot < kParallelDotMin) {
    const double theta = std::acos(std::min(dot, 1.0));
    const double s = std::sin(theta);
    wa = std::sin((1.0 - t) * theta) / s;
    wb = std::sin(t * theta) / s;
  }
  return normalized(
    {wa * a[0] + wb * b[0], wa * a[1] + wb * b[1], wa * a[2] + wb * b[2], wa * a[3] + wb * b[3]});
}

}  // namespace

bool PoseSample::isValid() const
{
  if (timestamp_sample_ns == 0U) {
    return false;
  }
  for (double v : position_ned_m) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  double norm2 = 0.0;
  for (double v : q_wxyz) {
    if (!std::isfinite(v)) {
      return false;
    }
    norm2 += v * v;
  }
  return std::isfinite(norm2) && norm2 > 0.0;
}

PoseHistory::PoseHistory(const PoseHistoryConfig & config)
: _config(config)
{
  if (!config.isValid()) {
    throw std::invalid_argument(
            "PoseHistoryConfig: depth must be at least 2 and max_bracket_gap_ns above 0");
  }
}

bool PoseHistory::push(const PoseSample & sample)
{
  if (sample.timestamp_sample_ns == 0U) {
    return false;
  }
  const std::lock_guard<std::mutex> lock(_mutex);
  if (!_samples.empty() && sample.timestamp_sample_ns <= _samples.back().timestamp_sample_ns) {
    return false;
  }
  _samples.push_back(sample);
  while (_samples.size() > _config.depth) {
    _samples.pop_front();
  }
  return true;
}

std::size_t PoseHistory::size() const
{
  const std::lock_guard<std::mutex> lock(_mutex);
  return _samples.size();
}

PoseLookupResult PoseHistory::poseAt(uint64_t time_ns) const
{
  const std::lock_guard<std::mutex> lock(_mutex);
  PoseLookupResult result;

  // First sample at or after the query time.
  const auto after = std::lower_bound(
    _samples.begin(), _samples.end(), time_ns,
    [](const PoseSample & s, uint64_t t) {return s.timestamp_sample_ns < t;});

  if (after != _samples.end() && after->timestamp_sample_ns == time_ns) {
    // On a sample: no bracket is needed.
    if (!after->isValid()) {
      result.status = PoseLookupStatus::kInvalidSampleInBracket;
      return result;
    }
    result.status = PoseLookupStatus::kOk;
    result.pose = *after;
    return result;
  }
  if (after == _samples.begin()) {
    result.status = PoseLookupStatus::kNoSampleBefore;
    return result;
  }
  if (after == _samples.end()) {
    result.status = PoseLookupStatus::kNoSampleAfter;
    return result;
  }
  const PoseSample & lo = *(after - 1);
  const PoseSample & hi = *after;
  if (!lo.isValid() || !hi.isValid()) {
    result.status = PoseLookupStatus::kInvalidSampleInBracket;
    return result;
  }
  if (lo.reset_counter != hi.reset_counter) {
    result.status = PoseLookupStatus::kResetInBracket;
    return result;
  }
  const uint64_t gap_ns = hi.timestamp_sample_ns - lo.timestamp_sample_ns;
  if (gap_ns > _config.max_bracket_gap_ns) {
    result.status = PoseLookupStatus::kBracketGapTooLarge;
    return result;
  }
  const double t = static_cast<double>(time_ns - lo.timestamp_sample_ns) /
    static_cast<double>(gap_ns);
  result.status = PoseLookupStatus::kOk;
  result.pose.timestamp_sample_ns = time_ns;
  result.pose.reset_counter = lo.reset_counter;
  result.pose.q_wxyz = slerp(normalized(lo.q_wxyz), normalized(hi.q_wxyz), t);
  for (std::size_t i = 0U; i < 3U; ++i) {
    result.pose.position_ned_m[i] =
      lo.position_ned_m[i] + t * (hi.position_ned_m[i] - lo.position_ned_m[i]);
  }
  return result;
}

}  // namespace livox_ray_source
