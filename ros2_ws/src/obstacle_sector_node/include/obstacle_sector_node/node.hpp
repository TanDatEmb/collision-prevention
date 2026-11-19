#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "obstacle_sector_node/pipeline.hpp"
#include "px4_msgs/msg/obstacle_distance.hpp"
#include "rclcpp/rclcpp.hpp"

namespace obstacle_sector_node
{

/// Diagnostic level of the node: ERROR when the source ended, the latest frame did not encode or
/// the latest map was held back for a negative age (nothing reaches PX4), WARN when earlier frames
/// failed to encode or earlier maps were held back, OK otherwise.
[[nodiscard]] uint8_t diagnosticLevel(
  StepStatus last_status, const PipelineStats & stats, uint64_t negative_age_drops = 0U,
  bool latest_map_dropped = false);

/// QoS of the ObstacleDistance publisher (ICD-CP-03): best effort, volatile, keep last 2.
[[nodiscard]] rclcpp::QoS obstacleDistanceQos();

/// Age of a map in nanoseconds: `now_ns` (a clock reading, not negative) minus the map's sample
/// timestamp in microseconds. Both must be on the same clock (the two time profiles of ADR 0003);
/// a negative result means the sample time is ahead of the node clock. A sample time that does not
/// fit in int64 nanoseconds (after the year 2262, or garbled) gives the most negative age.
[[nodiscard]] int64_t mapAgeNs(int64_t now_ns, uint64_t sample_timestamp_us);

/// ROS 2 node around SectorMapPipeline. It publishes ObstacleDistance on
/// `/fmu/in/obstacle_distance` (best effort, volatile, keep last 2, ICD-CP-03) at
/// `sector_map_rate_hz` (default 10, the approved baseline) and diagnostics on `status` at 1 Hz.
/// It never publishes anything inferred: when the pipeline gives no message, nothing is sent.
/// A map whose sample time is ahead of the node clock (negative age) is not published either: PX4
/// would clamp it to the receive time and cannot see it (ICD-CP-04). It is counted and reported in
/// the diagnostics. No limit on a large age is applied here (OPEN-033).
class ObstacleSectorNode : public rclcpp::Node
{
public:
  /// Throws std::invalid_argument for a null source or `sector_map_rate_hz` not positive and
  /// finite.
  ObstacleSectorNode(
    std::unique_ptr<livox_ray_source::RaySource> source,
    const obstacle_sector_map::SectorMapConfig & config,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  [[nodiscard]] const PipelineStats & stats() const { return _pipeline->stats(); }

  /// Maps the pipeline built that were not published because their age was negative.
  [[nodiscard]] uint64_t negativeAgeDrops() const { return _negative_age_drops; }

private:
  void publishNext();
  void publishDiagnostics();

  std::unique_ptr<SectorMapPipeline> _pipeline;
  StepStatus _last_status{StepStatus::kMessage};
  uint64_t _negative_age_drops{0U};
  bool _latest_map_dropped{false};
  std::optional<uint64_t> _latest_sample_us;
  rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr _distance_pub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr _diagnostics_pub;
  rclcpp::TimerBase::SharedPtr _map_timer;
  rclcpp::TimerBase::SharedPtr _diagnostics_timer;
};

}  // namespace obstacle_sector_node
