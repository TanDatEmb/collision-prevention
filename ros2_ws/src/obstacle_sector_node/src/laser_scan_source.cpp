#include "obstacle_sector_node/laser_scan_source.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace obstacle_sector_node
{

namespace
{

using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;

constexpr double kNanosecondsPerSecond{1.0e9};

RayReturnKind classify(float range_m, float range_min_m, float range_max_m)
{
  if (std::isnan(range_m)) {
    return RayReturnKind::kInvalid;
  }
  if (range_m >= range_max_m) {
    return RayReturnKind::kQualifiedNoReturn;  // includes +inf
  }
  if (range_m >= range_min_m && range_m > 0.0F) {
    return RayReturnKind::kReturn;
  }
  return RayReturnKind::kInvalid;  // below range_min, zero, negative, -inf
}

}  // namespace

livox_ray_source::RayFrame laserScanToFrame(
  const sensor_msgs::msg::LaserScan & scan, bool packet_continuity_valid)
{
  livox_ray_source::RayFrame frame;
  frame.packet_continuity_valid = packet_continuity_valid;
  frame.rays.reserve(scan.ranges.size());

  const uint64_t header_ns = static_cast<uint64_t>(scan.header.stamp.sec) * 1'000'000'000U +
                             static_cast<uint64_t>(scan.header.stamp.nanosec);
  const bool has_time = header_ns != 0U;
  const double increment_s = std::isfinite(scan.time_increment) && scan.time_increment > 0.0F
                               ? static_cast<double>(scan.time_increment)
                               : 0.0;

  for (std::size_t index = 0U; index < scan.ranges.size(); ++index) {
    RaySample ray;
    ray.range_m = scan.ranges[index];
    ray.azimuth_rad = scan.angle_min + static_cast<float>(index) * scan.angle_increment;
    ray.elevation_rad = 0.0F;
    ray.masked = false;
    ray.return_kind = classify(scan.ranges[index], scan.range_min, scan.range_max);
    ray.timestamp_ns =
      has_time ? header_ns + static_cast<uint64_t>(std::llround(
                               static_cast<double>(index) * increment_s * kNanosecondsPerSecond))
               : 0U;
    frame.rays.push_back(ray);
  }
  return frame;
}

bool ScanContinuityTracker::update(uint64_t scan_time_ns)
{
  const bool continuous = _has_previous && scan_time_ns != 0U && scan_time_ns > _previous_ns;
  _has_previous = true;
  _previous_ns = scan_time_ns;
  return continuous;
}

void LatestFrameQueue::push(livox_ray_source::RayFrame frame)
{
  {
    const std::lock_guard<std::mutex> lock(_mutex);
    if (_frame.has_value()) {
      ++_dropped;
    }
    _frame = std::move(frame);
  }
  _condition.notify_one();
}

std::optional<livox_ray_source::RayFrame> LatestFrameQueue::waitPop(
  std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(_mutex);
  if (!_condition.wait_for(lock, timeout, [this]() { return _frame.has_value(); })) {
    return std::nullopt;
  }
  auto frame = std::move(_frame);
  _frame.reset();
  return frame;
}

uint64_t LatestFrameQueue::droppedCount() const
{
  const std::lock_guard<std::mutex> lock(_mutex);
  return _dropped;
}

QueuedRaySource::QueuedRaySource(
  std::shared_ptr<LatestFrameQueue> queue, std::chrono::milliseconds timeout)
: _queue(std::move(queue)), _timeout(timeout)
{
  if (!_queue) {
    throw std::invalid_argument("QueuedRaySource needs a queue");
  }
  if (_timeout <= std::chrono::milliseconds(0)) {
    throw std::invalid_argument("QueuedRaySource needs a positive timeout");
  }
}

std::optional<livox_ray_source::RayFrame> QueuedRaySource::nextFrame()
{
  return _queue->waitPop(_timeout);
}

}  // namespace obstacle_sector_node
