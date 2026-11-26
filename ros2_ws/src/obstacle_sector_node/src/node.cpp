#include "obstacle_sector_node/node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace obstacle_sector_node
{
namespace
{

constexpr double kDefaultRateHz{10.0};

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, uint64_t value)
{
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = key;
  entry.value = std::to_string(value);
  return entry;
}

}  // namespace

bool LevelingCoverageConfig::isValid() const
{
  if (!enabled) {
    return true;
  }
  return std::isfinite(mount_roll_rad) && std::isfinite(mount_pitch_down_rad) &&
         std::isfinite(mount_yaw_left_rad) && std::isfinite(sensor_elevation_min_rad) &&
         std::isfinite(sensor_elevation_max_rad) && sensor_elevation_min_rad < sensor_elevation_max_rad &&
         std::isfinite(height_up_m) && height_up_m > 0.0 && std::isfinite(height_down_m) &&
         height_down_m > 0.0 && std::isfinite(attitude_allowance_rad) && attitude_allowance_rad >= 0.0 &&
         required_rays > 0U;
}

rclcpp::QoS obstacleDistanceQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile();
}

int64_t mapAgeNs(int64_t now_ns, uint64_t sample_timestamp_us)
{
  constexpr int64_t kNanosecondsPerMicrosecond{1'000};
  constexpr uint64_t kLargestSampleUs{
    static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / kNanosecondsPerMicrosecond)};

  // A time beyond int64 nanoseconds would overflow below and could wrap into an old-looking
  // sample; report the most negative age instead, so it counts as ahead of the clock.
  if (sample_timestamp_us > kLargestSampleUs) {
    return std::numeric_limits<int64_t>::min();
  }

  return now_ns - static_cast<int64_t>(sample_timestamp_us) * kNanosecondsPerMicrosecond;
}

uint8_t diagnosticLevel(
  StepStatus last_status, const PipelineStats & stats, uint64_t negative_age_drops,
  bool latest_map_dropped)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  if (
    last_status == StepStatus::kSourceEnded || last_status == StepStatus::kEncodeFailed ||
    latest_map_dropped) {
    return DiagnosticStatus::ERROR;
  }

  return (stats.encode_failures > 0U || negative_age_drops > 0U) ? DiagnosticStatus::WARN
                                                                 : DiagnosticStatus::OK;
}

ObstacleSectorNode::ObstacleSectorNode(
  std::unique_ptr<livox_ray_source::RaySource> source,
  const obstacle_sector_map::SectorMapConfig & config, const rclcpp::NodeOptions & options,
  const LevelingCoverageConfig & coverage)
: rclcpp::Node("obstacle_sector_node", options),
  _pipeline(std::make_unique<SectorMapPipeline>(std::move(source), config)),
  _coverage_config(coverage)
{
  const double rate_hz = declare_parameter("sector_map_rate_hz", kDefaultRateHz);

  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    throw std::invalid_argument("sector_map_rate_hz must be positive and finite");
  }
  if (!_coverage_config.isValid()) {
    throw std::invalid_argument("leveling coverage geometry is not valid");
  }

  _distance_pub = create_publisher<px4_msgs::msg::ObstacleDistance>(
    "/fmu/in/obstacle_distance", obstacleDistanceQos());
  if (_coverage_config.enabled) {
    _coverage_pub = create_publisher<collision_prevention_msgs::msg::CpLevelingCoverage>(
      "/collision_prevention/leveling_coverage", obstacleDistanceQos());
  }
  _diagnostics_pub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("status", 10);

  _map_timer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz)),
    [this]() { publishNext(); });
  _diagnostics_timer =
    create_wall_timer(std::chrono::seconds(1), [this]() { publishDiagnostics(); });
}

void ObstacleSectorNode::publishNext()
{
  auto result = _pipeline->step();

  _last_status = result.status;
  _latest_map_dropped = false;

  if (result.status != StepStatus::kMessage) {
    return;
  }

  // PX4 clamps a later timestamp to the receive time and cannot see it (ICD-CP-04), so a map whose
  // sample time is ahead of this clock is held back. No limit on a large age is applied (OPEN-033).
  if (mapAgeNs(now().nanoseconds(), result.message->timestamp) < 0) {
    ++_negative_age_drops;
    _latest_map_dropped = true;
    return;
  }

  _latest_sample_us = result.message->timestamp;
  _distance_pub->publish(*result.message);
  if (_coverage_pub && _pipeline->latestMap().has_value()) {
    publishLevelingCoverage(*_pipeline->latestMap());
  }
}

void ObstacleSectorNode::publishLevelingCoverage(const obstacle_sector_map::SectorMap & map)
{
  collision_prevention_msgs::msg::CpLevelingCoverage message;
  message.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1'000);
  message.sample_timestamp = map.sample_timestamp_ns / 1'000U;
  message.mount_roll_rad = static_cast<float>(_coverage_config.mount_roll_rad);
  message.mount_pitch_down_rad = static_cast<float>(_coverage_config.mount_pitch_down_rad);
  message.mount_yaw_left_rad = static_cast<float>(_coverage_config.mount_yaw_left_rad);
  message.sensor_elevation_min_rad = static_cast<float>(_coverage_config.sensor_elevation_min_rad);
  message.sensor_elevation_max_rad = static_cast<float>(_coverage_config.sensor_elevation_max_rad);
  message.height_up_m = static_cast<float>(_coverage_config.height_up_m);
  message.height_down_m = static_cast<float>(_coverage_config.height_down_m);
  message.attitude_allowance_rad = static_cast<float>(_coverage_config.attitude_allowance_rad);
  message.required_rays = _coverage_config.required_rays;
  message.packet_continuity_valid = map.packet_continuity_valid;
  for (std::size_t index = 0U; index < obstacle_sector_map::kSectorCount; ++index) {
    const auto & sector = map.sectors[index];
    message.up_ray_count[index] = sector.up_ray_count;
    message.down_ray_count[index] = sector.down_ray_count;
    message.rear_only[index] = sector.rear_only;
    message.coverage_ok[index] = sector.coverage_ok;
  }
  _coverage_pub->publish(message);
}

void ObstacleSectorNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "obstacle_sector_node";
  status.hardware_id = "obstacle_sector_node";
  status.level =
    diagnosticLevel(_last_status, _pipeline->stats(), _negative_age_drops, _latest_map_dropped);
  status.message = _last_status == StepStatus::kSourceEnded    ? "source ended"
                   : _last_status == StepStatus::kEncodeFailed ? "latest frame did not encode"
                   : _latest_map_dropped ? "latest map dropped: sample time ahead of the node clock"
                                         : "running";
  diagnostic_msgs::msg::KeyValue age;
  age.key = "latest_map_age_ms";
  age.value = _latest_sample_us.has_value()
                ? std::to_string(mapAgeNs(now().nanoseconds(), *_latest_sample_us) / 1'000'000)
                : "none";
  status.values.push_back(age);
  status.values.push_back(keyValue("frames", _pipeline->stats().frames));
  status.values.push_back(keyValue("messages", _pipeline->stats().messages));
  status.values.push_back(keyValue("encode_failures", _pipeline->stats().encode_failures));
  status.values.push_back(keyValue("negative_age_drops", _negative_age_drops));
  status.values.push_back(keyValue("rejected_rays", _pipeline->stats().rejected_rays));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(status);
  _diagnostics_pub->publish(array);
}

}  // namespace obstacle_sector_node
