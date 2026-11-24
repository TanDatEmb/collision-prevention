#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>

#include "livox_ray_source/pose_history.hpp"

namespace livox_ray_source
{

/// One sample of the Livox IMU gyro (ADR 0023): the angular rate of the sensor in the sensor FLU frame
/// (the frame of the points; the IMU axes are taken equal to it until the bench check 5a of ADR 0023),
/// with the PTP time of its packet.
struct GyroSample
{
  uint64_t timestamp_ns{0U};
  std::array<double, 3> rate_flu_rad_s{
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  /// Valid: a non-zero time and three finite rates.
  [[nodiscard]] bool isValid() const;
};

/// The rotation of the sensor integrated from the gyro, for the rotation inside one frame (ADR 0023
/// decision 1 and 2). Thread safe: the SDK thread pushes, the frame thread looks up.
///
/// - Integration: trapezoid rule between consecutive samples (exact for a rate that changes linearly).
///   The orientation of a sample maps the sensor frame at that time to the sensor frame at the start of
///   its segment.
/// - Segments: the first sample, a gap above `max_bracket_gap_ns`, or a time that goes back starts a new
///   segment from the identity and increments the segment counter, which is the `reset_counter` of the
///   looked-up orientation. Two orientations can only be combined within one segment. A sample with the
///   time of the newest one is ignored.
/// - Lookup (`orientationAt`): slerp between the two samples around the time (`PoseHistory`). After the
///   newest sample the newest rate is held, for at most `max_bracket_gap_ns`, so a ray at the end of a
///   frame does not wait for the next IMU sample.
/// The position of a looked-up sample is zero; only `q_wxyz`, `timestamp_sample_ns` and `reset_counter`
/// have a meaning.
class GyroTrack
{
public:
  /// Throws std::invalid_argument when `config` is not valid.
  explicit GyroTrack(const PoseHistoryConfig & config);

  /// Integrates one sample. False when the sample is not valid or has the time of the newest sample.
  bool push(const GyroSample & sample);

  [[nodiscard]] PoseLookupResult orientationAt(uint64_t time_ns) const;

private:
  PoseHistoryConfig _config;
  mutable std::mutex _mutex;
  std::unique_ptr<PoseHistory> _history;
  bool _has_newest{false};
  GyroSample _newest{};
  std::array<double, 4> _newest_q_wxyz{1.0, 0.0, 0.0, 0.0};
  uint8_t _segment{0U};
};

}  // namespace livox_ray_source
