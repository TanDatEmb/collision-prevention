#include "obstacle_sector_node/odometry_pose.hpp"

#include <limits>

namespace obstacle_sector_node
{

std::optional<livox_ray_source::PoseSample> poseFromOdometry(
  const px4_msgs::msg::VehicleOdometry & odometry, int64_t now_ns)
{
  constexpr uint64_t kNanosecondsPerMicrosecond{1'000U};
  constexpr uint64_t kLargestSampleUs{
    static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) / kNanosecondsPerMicrosecond};

  // Unknown time, or a time that would overflow int64 nanoseconds: not a sample.
  if (odometry.timestamp_sample == 0U || odometry.timestamp_sample > kLargestSampleUs) {
    return std::nullopt;
  }
  const uint64_t sample_ns = odometry.timestamp_sample * kNanosecondsPerMicrosecond;
  if (now_ns < 0 || sample_ns > static_cast<uint64_t>(now_ns)) {
    return std::nullopt;  // ahead of the node clock
  }

  livox_ray_source::PoseSample sample;
  sample.timestamp_sample_ns = sample_ns;
  sample.reset_counter = odometry.reset_counter;
  for (std::size_t i = 0U; i < 3U; ++i) {
    sample.position_ned_m[i] = static_cast<double>(odometry.position[i]);
  }
  for (std::size_t i = 0U; i < 4U; ++i) {
    sample.q_wxyz[i] = static_cast<double>(odometry.q[i]);
  }
  return sample;
}

}  // namespace obstacle_sector_node
