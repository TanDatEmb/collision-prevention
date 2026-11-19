#include "obstacle_sector_node/node.hpp"

#include <chrono>
#include <cmath>
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

ObstacleSectorNode::ObstacleSectorNode(
  std::unique_ptr<livox_ray_source::RaySource> source,
  const obstacle_sector_map::SectorMapConfig & config, const rclcpp::NodeOptions & options)
: rclcpp::Node("obstacle_sector_node", options),
  _pipeline(std::make_unique<SectorMapPipeline>(std::move(source), config))
{
  const double rate_hz = declare_parameter("sector_map_rate_hz", kDefaultRateHz);

  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    throw std::invalid_argument("sector_map_rate_hz must be positive and finite");
  }

  _distance_pub = create_publisher<px4_msgs::msg::ObstacleDistance>(
    "/fmu/in/obstacle_distance",
    rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile());
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

  if (result.status == StepStatus::kMessage) {
    _distance_pub->publish(*result.message);
  }
  else if (result.status == StepStatus::kSourceEnded) {
    _source_ended = true;
  }
}

void ObstacleSectorNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "obstacle_sector_node";
  status.hardware_id = "obstacle_sector_node";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = _source_ended ? "source ended" : "running";
  status.values.push_back(keyValue("frames", _pipeline->stats().frames));
  status.values.push_back(keyValue("messages", _pipeline->stats().messages));
  status.values.push_back(keyValue("encode_failures", _pipeline->stats().encode_failures));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(status);
  _diagnostics_pub->publish(array);
}

}  // namespace obstacle_sector_node
