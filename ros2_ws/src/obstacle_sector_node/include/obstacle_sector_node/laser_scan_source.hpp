#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

#include "livox_ray_source/ray_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace obstacle_sector_node
{

/// One planar scan (`sensor_msgs/LaserScan`, for example the Gazebo 2D lidar through
/// `ros_gz_bridge`) as one ray frame.
///
/// - A finite range in `[range_min, range_max)` is a return.
/// - A range at or beyond `range_max`, or `+inf`, is a qualified no-return. This is a simulation
///   rule: an ideal simulated lidar has no dropout, so a beam that hits nothing is clear evidence
///   (the fidelity gap listed in `docs/architecture.md`, section 7).
/// - Anything else (NaN, `-inf`, a range below `range_min`, zero, negative) is invalid. It is never
///   read as free space.
/// - Azimuth is the scan angle (counter-clockwise from the front, FLU), elevation is 0.
/// - The ray time is the header time plus `i * time_increment`; a zero header time stays zero and
///   is rejected downstream (SYS-CP-016).
[[nodiscard]] livox_ray_source::RayFrame laserScanToFrame(
  const sensor_msgs::msg::LaserScan & scan, bool packet_continuity_valid);

/// Packet continuity of a scan stream: true when the scan time is non-zero and later than the
/// previous scan's time. The first scan, a repeated time and a backwards time are not continuous.
class ScanContinuityTracker
{
public:
  [[nodiscard]] bool update(uint64_t scan_time_ns);

private:
  bool _has_previous{false};
  uint64_t _previous_ns{0U};
};

/// Hand-over of the newest frame from a subscriber thread to the node thread. A frame that was not
/// read before the next one arrives is replaced and counted.
class LatestFrameQueue
{
public:
  void push(livox_ray_source::RayFrame frame);

  /// The newest unread frame, waiting up to `timeout` for one; std::nullopt on timeout.
  [[nodiscard]] std::optional<livox_ray_source::RayFrame> waitPop(
    std::chrono::milliseconds timeout);

  [[nodiscard]] uint64_t droppedCount() const;

private:
  mutable std::mutex _mutex;
  std::condition_variable _condition;
  std::optional<livox_ray_source::RayFrame> _frame;
  uint64_t _dropped{0U};
};

/// RaySource over a LatestFrameQueue. `nextFrame()` waits up to `timeout` for a frame and returns
/// std::nullopt after that, which the node reports as "source ended".
class QueuedRaySource : public livox_ray_source::RaySource
{
public:
  /// Throws std::invalid_argument for a null queue or a timeout that is not positive.
  QueuedRaySource(std::shared_ptr<LatestFrameQueue> queue, std::chrono::milliseconds timeout);

  [[nodiscard]] std::optional<livox_ray_source::RayFrame> nextFrame() override;

private:
  std::shared_ptr<LatestFrameQueue> _queue;
  std::chrono::milliseconds _timeout;
};

}  // namespace obstacle_sector_node
