#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>

namespace livox_ray_source
{

/// One vehicle pose from the pose chain of ADR 0018 and 0019 (PX4 `VehicleOdometry`).
///
/// Frames: `q_wxyz` is the Hamilton quaternion (w, x, y, z) that rotates a vector from the body
/// FRD frame (forward, right, down) to the local NED frame, as `VehicleOdometry.q` does;
/// `position_ned_m` is the body origin in the same local NED frame. A constant heading offset of
/// the local frame (`pose_frame` FRD) does not matter for the leveling transform, which only uses
/// heading differences and position differences.
///
/// `timestamp_sample_ns` is the sample time of the pose in the time base of the rays, in
/// nanoseconds. PX4 stamps in microseconds; the caller converts and, if the clocks differ, maps
/// the time base (not done here). 0 is "unknown" and never valid.
struct PoseSample
{
  uint64_t timestamp_sample_ns{0U};
  std::array<double, 4> q_wxyz{
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 3> position_ned_m{
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  uint8_t reset_counter{0U};  ///< compared for inequality only; the counter wraps

  /// Valid: a non-zero time, a finite position and a finite quaternion of non-zero norm
  /// (PX4 sets the quaternion to NaN when the attitude is invalid).
  [[nodiscard]] bool isValid() const;
};

enum class PoseLookupStatus : uint8_t
{
  kOk,
  kNoSampleBefore,       ///< no sample at or before the query time (also an empty history)
  kNoSampleAfter,        ///< no sample at or after the query time
  kBracketGapTooLarge,   ///< the two samples around the time are further apart than allowed
  kResetInBracket,       ///< the two samples have different reset counters
  kInvalidSampleInBracket,  ///< a sample around the time is not valid (NaN quaternion, ...)
};

/// Result of a pose query. `pose` is meaningful only when `status` is kOk; its
/// `timestamp_sample_ns` is the query time.
struct PoseLookupResult
{
  PoseLookupStatus status{PoseLookupStatus::kNoSampleBefore};
  PoseSample pose{};

  [[nodiscard]] bool ok() const {return status == PoseLookupStatus::kOk;}
};

/// Required parameters of PoseHistory. There are no defaults: a default-constructed config holds
/// the sentinels (0) and is not valid. The values come from the platform configuration once the
/// owner has approved them; they are not decided in code.
struct PoseHistoryConfig
{
  /// Number of samples kept; the oldest is dropped when it is exceeded. Sentinel 0.
  /// Valid: at least 2 (an interpolation needs two samples).
  std::size_t depth{0U};
  /// Largest time between the two samples that bracket a query for it to be interpolated,
  /// nanoseconds. Sentinel 0. Valid: greater than 0.
  uint64_t max_bracket_gap_ns{0U};

  [[nodiscard]] bool isValid() const {return depth >= 2U && max_bracket_gap_ns > 0U;}
};

/// History of pose samples keyed by their sample time, with interpolation (spherical for the
/// quaternion, linear for the position) between the two samples that bracket the query time.
///
/// A query returns no pose when: there is no sample on one side, the two samples are further
/// apart than `max_bracket_gap_ns`, their reset counters differ, or either one is not valid. A
/// query that equals the time of a sample returns that sample, even when its neighbours would
/// fail. Invalid samples are stored (the sample time is known) so that a bracket that touches one
/// gives no pose instead of being interpolated across it.
///
/// push() returns true when the sample is stored. It returns false, and stores nothing, for a
/// sample time of 0 and for a time that is not strictly later than the newest sample (a duplicate
/// or an out of order sample).
///
/// Thread safe: the odometry callback may push while another thread queries.
class PoseHistory
{
public:
  /// Throws std::invalid_argument when `config` is not valid.
  explicit PoseHistory(const PoseHistoryConfig & config);

  bool push(const PoseSample & sample);

  /// Pose at `time_ns`, or a status that says why there is none.
  [[nodiscard]] PoseLookupResult poseAt(uint64_t time_ns) const;

  [[nodiscard]] std::size_t size() const;

private:
  PoseHistoryConfig _config;
  mutable std::mutex _mutex;
  std::deque<PoseSample> _samples;  // strictly increasing timestamp_sample_ns
};

}  // namespace livox_ray_source
