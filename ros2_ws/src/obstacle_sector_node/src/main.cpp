// Executable of the Orin side pipeline: planar scan (Gazebo SITL, through ros_gz_bridge) to the
// sector map to `/fmu/in/obstacle_distance`.
//
// Parameters (all ROS parameters, so `--ros-args -p name:=value` works):
//   platform_config  path of the platform YAML; required. A null value in it is an error: the
//                    elevation band and the coverage rule are never defaulted here (OPEN-005,
//                    OPEN-023).
//   scan_topic       planar scan topic, default `scan`.
//   source_wait_s    how long the node waits for a frame before it reports "source ended",
//                    default 1.0.
//   sector_map_rate_hz  see ObstacleSectorNode (default 10).
#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "obstacle_sector_node/laser_scan_source.hpp"
#include "obstacle_sector_node/node.hpp"
#include "obstacle_sector_node/platform_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscribeToScans(
  rclcpp::Node & node, const std::string & topic,
  const std::shared_ptr<obstacle_sector_node::LatestFrameQueue> & queue)
{
  auto tracker = std::make_shared<obstacle_sector_node::ScanContinuityTracker>();
  return node.create_subscription<sensor_msgs::msg::LaserScan>(
    topic, rclcpp::SensorDataQoS(),
    [queue, tracker](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
      const uint64_t scan_time_ns = static_cast<uint64_t>(scan->header.stamp.sec) * 1'000'000'000U +
                                    static_cast<uint64_t>(scan->header.stamp.nanosec);
      queue->push(obstacle_sector_node::laserScanToFrame(*scan, tracker->update(scan_time_ns)));
    });
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    // Global `--ros-args -p` overrides reach this helper node as well as the others.
    auto options_node = std::make_shared<rclcpp::Node>("obstacle_sector_node_options");
    const auto platform_config =
      options_node->declare_parameter<std::string>("platform_config", "");
    const auto scan_topic = options_node->declare_parameter<std::string>("scan_topic", "scan");
    const double source_wait_s = options_node->declare_parameter<double>("source_wait_s", 1.0);
    if (platform_config.empty()) {
      throw std::invalid_argument(
        "parameter platform_config is required (path of a platform YAML)");
    }
    if (!(source_wait_s > 0.0)) {
      throw std::invalid_argument("parameter source_wait_s must be positive");
    }

    const auto config = obstacle_sector_node::loadSectorMapConfig(platform_config);
    auto queue = std::make_shared<obstacle_sector_node::LatestFrameQueue>();
    auto source = std::make_unique<obstacle_sector_node::QueuedRaySource>(
      queue, std::chrono::milliseconds(static_cast<int64_t>(source_wait_s * 1000.0)));
    auto node =
      std::make_shared<obstacle_sector_node::ObstacleSectorNode>(std::move(source), config);

    // The scan callback runs on its own thread: the node thread waits for frames inside nextFrame().
    auto feeder = std::make_shared<rclcpp::Node>("laser_scan_feeder");
    auto subscription = subscribeToScans(*feeder, scan_topic, queue);
    rclcpp::executors::SingleThreadedExecutor feeder_executor;
    feeder_executor.add_node(feeder);
    std::thread feeder_thread([&feeder_executor]() { feeder_executor.spin(); });

    RCLCPP_INFO(
      node->get_logger(),
      "platform %s, scan topic %s, elevation band [%.4f, %.4f] rad, coverage %u",
      platform_config.c_str(), scan_topic.c_str(), static_cast<double>(config.min_elevation_rad),
      static_cast<double>(config.max_elevation_rad), config.coverage_min_rays_per_sector);

    rclcpp::spin(node);

    feeder_executor.cancel();
    feeder_thread.join();
    RCLCPP_INFO(
      node->get_logger(),
      "frames %lu, messages %lu, encode failures %lu, scans replaced unread %lu",
      static_cast<unsigned long>(node->stats().frames),
      static_cast<unsigned long>(node->stats().messages),
      static_cast<unsigned long>(node->stats().encode_failures),
      static_cast<unsigned long>(queue->droppedCount()));
  }
  catch (const std::exception & error) {
    std::cerr << "obstacle_sector_node: " << error.what() << std::endl;
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
