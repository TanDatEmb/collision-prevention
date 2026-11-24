// Executable of the Orin side pipeline: a ray source, the sector map, `/fmu/in/obstacle_distance`.
//
// Parameters (all ROS parameters, so `--ros-args -p name:=value` works):
//   platform_config  path of the platform YAML; required. A null value in it is an error: the
//                    height band and the evidence rule are never defaulted here (OPEN-005,
//                    OPEN-023).
//   source           `laser_scan` (default): a planar scan, for the Gazebo 2D lidar through
//                    ros_gz_bridge; `ring_cloud`: an organised multi-ring PointCloud2, for the
//                    bench multi-ring lidar of Gazebo (ADR 0022 decision 4); or `livox`: a Livox
//                    Mid-360(S) through the Livox SDK2.
//   scan_topic       planar scan topic, default `scan` (source laser_scan).
//   cloud_topic      cloud topic, default `points` (source ring_cloud).
//   ring_layout.*    source ring_cloud: azimuth_count, azimuth_min_rad, azimuth_step_rad, ring_count,
//                    elevation_min_rad, elevation_step_rad, range_min_m, range_max_m and
//                    max_index_angle_error_rad. All required, no defaults: the layout of the sensor
//                    (the sensor frame, before the mount and the leveling). Azimuth and elevation of
//                    every ray come from the index and this layout, never from x, y, z.
//   livox_config     path of the Livox SDK2 JSON config; required for source livox.
//   coverage_window_ms  frame window of the Livox stream, default 100 (the ICD value).
//   configure_point_type  source livox: ask the lidar for spherical points when it sends another
//                    type (one read, one write only if it differs), default true.
//   source_wait_s    how long the node waits for a frame before it reports "source ended",
//                    default 1.0.
//   sector_map_rate_hz  see ObstacleSectorNode (default 10).
//
// Leveling (ADR 0020 D1, ADR 0022, issue #137): the sources ring_cloud and livox are leveled with the
// calibrated mount and the pose chain of the FMU, `/fmu/out/vehicle_odometry` (best-effort, as PX4
// publishes it). The platform YAML must then also hold `sensor.mount_pose_in_base_link` and
// `pose_history.*` (no defaults). A pose with no time or ahead of the node clock is not stored. The FMU pose
// is used at the reference time of a frame; the rotation inside the frame comes from the Livox gyro (ADR 0023),
// so the source livox also needs `gyro_history.*`. The source laser_scan, a planar bench scan, is not leveled:
// a plane makes no leveling claim (ADR 0022 decision 4).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "livox_ray_source/frame_segmenter.hpp"
#include "livox_ray_source/gyro_track.hpp"
#include "livox_ray_source/leveling_ray_source.hpp"
#include "livox_ray_source/livox_sdk_sample_source.hpp"
#include "livox_ray_source/masking_ray_source.hpp"
#include "livox_ray_source/pose_history.hpp"
#include "livox_ray_source/self_mask.hpp"
#include "obstacle_sector_node/laser_scan_source.hpp"
#include "obstacle_sector_node/node.hpp"
#include "obstacle_sector_node/odometry_pose.hpp"
#include "obstacle_sector_node/platform_config.hpp"
#include "obstacle_sector_node/ring_cloud_source.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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

unsigned long asUnsigned(uint64_t value) { return static_cast<unsigned long>(value); }

/// Counters of the ring cloud source, written by the subscriber thread and read by the log timer.
struct RingCloudCounters
{
  std::atomic<uint64_t> clouds{0U};
  std::atomic<uint64_t> shape_errors{0U};
  std::atomic<uint64_t> returns{0U};
  std::atomic<uint64_t> no_returns{0U};
  std::atomic<uint64_t> invalid{0U};
  std::atomic<uint64_t> index_mismatches{0U};
};

obstacle_sector_node::RingCloudLayout declareRingLayout(rclcpp::Node & node)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  obstacle_sector_node::RingCloudLayout layout;
  layout.azimuth_count =
    static_cast<uint32_t>(std::max<int64_t>(0, node.declare_parameter<int64_t>("ring_layout.azimuth_count", 0)));
  layout.azimuth_min_rad = node.declare_parameter<double>("ring_layout.azimuth_min_rad", nan);
  layout.azimuth_step_rad = node.declare_parameter<double>("ring_layout.azimuth_step_rad", nan);
  layout.ring_count =
    static_cast<uint32_t>(std::max<int64_t>(0, node.declare_parameter<int64_t>("ring_layout.ring_count", 0)));
  layout.elevation_min_rad = node.declare_parameter<double>("ring_layout.elevation_min_rad", nan);
  layout.elevation_step_rad = node.declare_parameter<double>("ring_layout.elevation_step_rad", nan);
  layout.range_min_m = static_cast<float>(node.declare_parameter<double>("ring_layout.range_min_m", nan));
  layout.range_max_m = static_cast<float>(node.declare_parameter<double>("ring_layout.range_max_m", nan));
  layout.max_index_angle_error_rad =
    node.declare_parameter<double>("ring_layout.max_index_angle_error_rad", nan);
  if (!layout.valid()) {
    throw std::invalid_argument(
      "source ring_cloud needs a valid ring_layout.* (all nine parameters, finite and positive)");
  }
  return layout;
}

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscribeToRingClouds(
  rclcpp::Node & node, const std::string & topic,
  const std::shared_ptr<obstacle_sector_node::LatestFrameQueue> & queue,
  const obstacle_sector_node::RingCloudLayout & layout,
  const std::shared_ptr<RingCloudCounters> & counters)
{
  auto tracker = std::make_shared<obstacle_sector_node::ScanContinuityTracker>();
  // Reliable, not SensorDataQoS: a cloud of 43200 points is 1.4 MB, and best-effort delivery of such a
  // message over DDS lost 60 to 75 % of the clouds on the bench (2.4 Hz received of 10 Hz sent; reliable
  // gave 10 Hz). Depth 2: the queue to the node keeps only the newest frame anyway.
  return node.create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::QoS(rclcpp::KeepLast(2)).reliable(),
    [queue, tracker, layout, counters](const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
      const uint64_t cloud_time_ns = static_cast<uint64_t>(cloud->header.stamp.sec) * 1'000'000'000U +
                                     static_cast<uint64_t>(cloud->header.stamp.nanosec);
      auto result = obstacle_sector_node::ringCloudToFrame(
        *cloud, layout, tracker->update(cloud_time_ns));
      counters->clouds.fetch_add(1U);
      if (!result.stats.shape_ok) {
        counters->shape_errors.fetch_add(1U);
      }
      counters->returns.fetch_add(result.stats.returns);
      counters->no_returns.fetch_add(result.stats.no_returns);
      counters->invalid.fetch_add(result.stats.invalid);
      counters->index_mismatches.fetch_add(result.stats.index_mismatch);
      queue->push(std::move(result.frame));
    });
}

/// Counters of the pose input, written by the odometry thread and read by the log timer.
struct PoseCounters
{
  std::atomic<uint64_t> received{0U};
  std::atomic<uint64_t> rejected{0U};    // no time, or ahead of the node clock: not stored
  std::atomic<uint64_t> not_stored{0U};  // not later than the newest sample (duplicate or out of order)
};

rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscribeToOdometry(
  rclcpp::Node & feeder, const rclcpp::Clock::SharedPtr & clock,
  const std::shared_ptr<livox_ray_source::PoseHistory> & poses,
  const std::shared_ptr<PoseCounters> & counters)
{
  return feeder.create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
    [clock, poses, counters](const px4_msgs::msg::VehicleOdometry::SharedPtr odometry) {
      counters->received.fetch_add(1U);
      const auto pose = obstacle_sector_node::poseFromOdometry(*odometry, clock->now().nanoseconds());
      if (!pose.has_value()) {
        counters->rejected.fetch_add(1U);
      }
      else if (!poses->push(*pose)) {
        counters->not_stored.fetch_add(1U);
      }
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
    const auto source_kind = options_node->declare_parameter<std::string>("source", "laser_scan");
    const auto scan_topic = options_node->declare_parameter<std::string>("scan_topic", "scan");
    const auto cloud_topic = options_node->declare_parameter<std::string>("cloud_topic", "points");
    const auto livox_config = options_node->declare_parameter<std::string>("livox_config", "");
    const auto window_ms = options_node->declare_parameter<int64_t>("coverage_window_ms", 100);
    const bool configure_point_type =
      options_node->declare_parameter<bool>("configure_point_type", true);
    const double source_wait_s = options_node->declare_parameter<double>("source_wait_s", 1.0);
    if (platform_config.empty()) {
      throw std::invalid_argument(
        "parameter platform_config is required (path of a platform YAML)");
    }
    if (source_kind != "laser_scan" && source_kind != "ring_cloud" && source_kind != "livox") {
      throw std::invalid_argument("parameter source must be laser_scan, ring_cloud or livox");
    }
    if (source_kind == "livox" && livox_config.empty()) {
      throw std::invalid_argument("source livox needs the parameter livox_config");
    }
    if (window_ms <= 0) {
      throw std::invalid_argument("parameter coverage_window_ms must be positive");
    }
    if (!(source_wait_s > 0.0)) {
      throw std::invalid_argument("parameter source_wait_s must be positive");
    }

    const auto config = obstacle_sector_node::loadSectorMapConfig(platform_config);
    auto queue = std::make_shared<obstacle_sector_node::LatestFrameQueue>();
    std::unique_ptr<livox_ray_source::RaySource> source =
      std::make_unique<obstacle_sector_node::QueuedRaySource>(
      queue, std::chrono::milliseconds(static_cast<int64_t>(source_wait_s * 1000.0)));

    // Leveling for the 3-D sources: the decorator reads the pose history that the odometry thread fills. The
    // history must outlive the node, so it is declared here.
    const bool leveled = source_kind != "laser_scan";
    std::shared_ptr<livox_ray_source::PoseHistory> poses;
    // The Livox gyro gives the rotation inside a frame (ADR 0023); the Gazebo cloud stamps all rays of a frame
    // with one time and needs none.
    std::shared_ptr<livox_ray_source::GyroTrack> gyro;
    livox_ray_source::LevelingRaySource * leveling_source = nullptr;
    if (leveled) {
      const auto settings = obstacle_sector_node::loadLevelingSettings(platform_config);
      // The self-mask first, on the sensor-frame rays, with the same mount as the leveler (FEAT-23, #206): a ray
      // that meets the airframe is marked masked and never counts as obstacle or free evidence.
      source = std::make_unique<livox_ray_source::MaskingRaySource>(
        std::move(source), livox_ray_source::loadSelfMask(platform_config, settings.leveling),
        static_cast<double>(config.certified_range_m));
      poses = std::make_shared<livox_ray_source::PoseHistory>(settings.history);
      if (source_kind == "livox") {
        gyro = std::make_shared<livox_ray_source::GyroTrack>(
          obstacle_sector_node::loadGyroHistoryConfig(platform_config));
      }
      auto leveling = std::make_unique<livox_ray_source::LevelingRaySource>(
        std::move(source), *poses, settings.leveling, gyro.get());
      leveling_source = leveling.get();
      source = std::move(leveling);
    }
    auto node =
      std::make_shared<obstacle_sector_node::ObstacleSectorNode>(std::move(source), config);

    // Planar scan: the scan callback runs on its own thread; the node thread waits for frames
    // inside nextFrame().
    std::shared_ptr<rclcpp::Node> feeder;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
    rclcpp::executors::SingleThreadedExecutor feeder_executor;
    std::thread feeder_thread;

    // Livox: the SDK thread gives rays to a segmenter, which hands finished frames to the queue.
    std::unique_ptr<livox_ray_source::LivoxSdkSampleSource> livox;
    rclcpp::TimerBase::SharedPtr livox_stats_timer;

    // Ring cloud: the same feeder thread, with the layout and counters of the cloud source.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription;
    auto cloud_counters = std::make_shared<RingCloudCounters>();
    rclcpp::TimerBase::SharedPtr cloud_stats_timer;

    if (source_kind == "laser_scan") {
      feeder = std::make_shared<rclcpp::Node>("laser_scan_feeder");
      subscription = subscribeToScans(*feeder, scan_topic, queue);
      feeder_executor.add_node(feeder);
      feeder_thread = std::thread([&feeder_executor]() { feeder_executor.spin(); });
    }
    else if (source_kind == "ring_cloud") {
      const auto layout = declareRingLayout(*options_node);
      feeder = std::make_shared<rclcpp::Node>("ring_cloud_feeder");
      cloud_subscription = subscribeToRingClouds(*feeder, cloud_topic, queue, layout, cloud_counters);
      feeder_executor.add_node(feeder);
      feeder_thread = std::thread([&feeder_executor]() { feeder_executor.spin(); });
      cloud_stats_timer = node->create_wall_timer(std::chrono::seconds(5), [node, cloud_counters]() {
        RCLCPP_INFO(
          node->get_logger(),
          "ring cloud: clouds %lu, shape errors %lu, rays: returns %lu, no-returns %lu, invalid %lu "
          "(index mismatches %lu); frames %lu, messages %lu, encode failures %lu",
          asUnsigned(cloud_counters->clouds.load()), asUnsigned(cloud_counters->shape_errors.load()),
          asUnsigned(cloud_counters->returns.load()), asUnsigned(cloud_counters->no_returns.load()),
          asUnsigned(cloud_counters->invalid.load()),
          asUnsigned(cloud_counters->index_mismatches.load()), asUnsigned(node->stats().frames),
          asUnsigned(node->stats().messages), asUnsigned(node->stats().encode_failures));
      });
    }
    else {
      auto segmenter = std::make_shared<livox_ray_source::FrameSegmenter>(
        static_cast<uint64_t>(window_ms) * 1'000'000U);
      auto segmenter_mutex = std::make_shared<std::mutex>();
      const auto logger = node->get_logger();
      livox = std::make_unique<livox_ray_source::LivoxSdkSampleSource>(
        livox_config,
        [queue, segmenter, segmenter_mutex](const livox_ray_source::StreamSample & sample) {
          const std::lock_guard<std::mutex> lock(*segmenter_mutex);
          auto finished = segmenter->push(sample);
          if (finished.has_value()) {
            queue->push(std::move(*finished));
          }
        },
        configure_point_type,
        [logger](const std::string & text) { RCLCPP_INFO(logger, "%s", text.c_str()); });
      livox->setGyroSink([gyro](const livox_ray_source::GyroSample & sample) { gyro->push(sample); });
      auto * livox_source = livox.get();
      livox_stats_timer = node->create_wall_timer(std::chrono::seconds(5), [node, livox_source]() {
        const auto stats = livox_source->stats();
        RCLCPP_INFO(
          node->get_logger(),
          "livox packets %lu, decoded %lu, rejected %lu, not spherical %lu, unsynchronized %lu, "
          "continuity breaks %lu, rays dropped for no time %lu, imu packets %lu, imu unusable %lu; "
          "frames %lu, messages %lu, encode failures %lu",
          asUnsigned(stats.packets), asUnsigned(stats.decoded_packets),
          asUnsigned(stats.rejected_packets), asUnsigned(stats.not_spherical_packets),
          asUnsigned(stats.unsynchronized_packets), asUnsigned(stats.continuity_breaks),
          asUnsigned(stats.dropped_rays), asUnsigned(stats.imu_packets), asUnsigned(stats.imu_unusable),
          asUnsigned(node->stats().frames),
          asUnsigned(node->stats().messages), asUnsigned(node->stats().encode_failures));
      });
      if (!livox->start()) {
        throw std::runtime_error("the Livox SDK did not start");
      }
    }

    // Odometry and the leveling log. The subscription goes on the feeder of the source (ring_cloud) or on a new
    // feeder for livox: the node thread blocks in nextFrame() and would starve the callback. A feeder thread that
    // is not running yet starts here, last, so that a failure above (the Livox SDK) leaves no thread behind.
    auto pose_counters = std::make_shared<PoseCounters>();
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription;
    rclcpp::TimerBase::SharedPtr leveling_stats_timer;
    if (leveled) {
      if (!feeder) {
        feeder = std::make_shared<rclcpp::Node>("odometry_feeder");
        feeder_executor.add_node(feeder);
      }
      odometry_subscription = subscribeToOdometry(*feeder, node->get_clock(), poses, pose_counters);
      if (!feeder_thread.joinable()) {
        feeder_thread = std::thread([&feeder_executor]() { feeder_executor.spin(); });
      }
      // Runs on the node thread, the one that calls nextFrame(), so it may read the decorator's count.
      leveling_stats_timer = node->create_wall_timer(
        std::chrono::seconds(5), [node, leveling_source, pose_counters]() {
          RCLCPP_INFO(
            node->get_logger(), "poses: received %lu, rejected %lu, not stored %lu; leveling: rays dropped %lu",
            asUnsigned(pose_counters->received.load()), asUnsigned(pose_counters->rejected.load()),
            asUnsigned(pose_counters->not_stored.load()), asUnsigned(leveling_source->raysDropped()));
        });
    }

    RCLCPP_INFO(
      node->get_logger(),
      "platform %s, source %s, height band [-%.2f, +%.2f] m, window %.2f m, %u returns, %u free rays",
      platform_config.c_str(), source_kind.c_str(), static_cast<double>(config.height_down_m),
      static_cast<double>(config.height_up_m), static_cast<double>(config.obstacle_window_m),
      config.obstacle_min_returns, config.free_min_rays);

    rclcpp::spin(node);

    if (livox) {
      livox->stop();
    }
    if (feeder_thread.joinable()) {
      feeder_executor.cancel();
      feeder_thread.join();
    }
    RCLCPP_INFO(
      node->get_logger(),
      "frames %lu, messages %lu, encode failures %lu, frames replaced unread %lu",
      asUnsigned(node->stats().frames), asUnsigned(node->stats().messages),
      asUnsigned(node->stats().encode_failures), asUnsigned(queue->droppedCount()));
  }
  catch (const std::exception & error) {
    std::cerr << "obstacle_sector_node: " << error.what() << std::endl;
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
