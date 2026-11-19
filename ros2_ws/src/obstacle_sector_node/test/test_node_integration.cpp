#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "livox_ray_source/replay_ray_source.hpp"
#include "obstacle_sector_node/node.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using livox_ray_source::ReplayRaySource;
using livox_ray_source::StreamSample;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;
using obstacle_sector_map::SectorMapConfig;
using obstacle_sector_node::ObstacleSectorNode;
using px4_msgs::msg::ObstacleDistance;

namespace
{

// Test fixture values, not approved platform values.
constexpr SectorMapConfig kConfig{-0.2F, 0.4F, 2U};
constexpr uint64_t kWindowNs = 1'000'000U;
constexpr uint64_t kFrameSpacingNs = 10'000'000U;
constexpr double kRateHz = 20.0;
constexpr const char * kTopic = "/fmu/in/obstacle_distance";
// About the year 2096: ahead of any real clock, and still inside the int64 nanosecond range.
constexpr uint64_t kFarFutureNs = 4'000'000'000'000'000'000U;

// Frames of two no-return rays each; with `timestamps` false the rays carry no time and the
// encoder refuses them. `first_frame_ns` is the time of the first ray (a small value is long
// before the node clock, kFarFutureNs is after it).
std::unique_ptr<livox_ray_source::RaySource> makeSource(
  int frames, bool timestamps = true, uint64_t first_frame_ns = 5'000'000U)
{
  std::vector<StreamSample> samples;
  for (int i = 0; i < frames; ++i) {
    const uint64_t base =
      timestamps ? first_frame_ns + static_cast<uint64_t>(i) * kFrameSpacingNs : 0U;
    for (uint64_t k = 0U; k < 2U; ++k) {
      samples.push_back(StreamSample{
        RaySample{
          timestamps ? base + k * 100'000U : 0U, std::numeric_limits<float>::quiet_NaN(), 0.0F,
          0.0F, false, RayReturnKind::kQualifiedNoReturn},
        false});
    }
  }
  return std::make_unique<ReplayRaySource>(std::move(samples), kWindowNs);
}

class Harness
{
public:
  // With `simulated_time` the node clock follows /clock (it reads 0 until the first message), so a
  // test can move it; otherwise the node runs on the system clock.
  explicit Harness(std::unique_ptr<livox_ray_source::RaySource> source, bool simulated_time = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("sector_map_rate_hz", kRateHz),
       rclcpp::Parameter("use_sim_time", simulated_time)});
    node = std::make_shared<ObstacleSectorNode>(std::move(source), kConfig, options);
    listener = std::make_shared<rclcpp::Node>("obstacle_listener");
    clock = listener->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    maps = listener->create_subscription<ObstacleDistance>(
      kTopic, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile(),
      [this](const ObstacleDistance &) { arrivals.push_back(std::chrono::steady_clock::now()); });
    diagnostics = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      std::string(node->get_namespace()) == "/" ? "/status"
                                                : std::string(node->get_namespace()) + "/status",
      10, [this](const diagnostic_msgs::msg::DiagnosticArray & array) {
        last_status = array.status.front();
        ++status_count;
      });
    executor.add_node(node);
    executor.add_node(listener);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some(std::chrono::milliseconds(5));
    }
  }

  // Sends the simulated time. The topic is best effort, so a test that needs the node to have it
  // sends it again while it spins.
  void setClock(std::chrono::seconds time)
  {
    rosgraph_msgs::msg::Clock message;
    message.clock =
      rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(time).count());
    clock->publish(message);
  }

  std::string value(const std::string & key) const
  {
    for (const auto & entry : last_status.values) {
      if (entry.key == key) {
        return entry.value;
      }
    }
    return "";
  }

  std::shared_ptr<ObstacleSectorNode> node;
  std::shared_ptr<rclcpp::Node> listener;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::Subscription<ObstacleDistance>::SharedPtr maps;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
  std::vector<std::chrono::steady_clock::time_point> arrivals;
  diagnostic_msgs::msg::DiagnosticStatus last_status;
  int status_count{0};
};

class NodeIntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

}  // namespace

TEST_F(NodeIntegrationTest, PublisherQosMatchesTheInterface)
{
  // The graph reports reliability and durability of the live publisher, not its history depth,
  // so the depth is checked on the QoS the node is built with.
  const auto profile = obstacle_sector_node::obstacleDistanceQos().get_rmw_qos_profile();
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 2U);

  Harness harness{makeSource(1)};
  const auto infos = harness.node->get_publishers_info_by_topic(kTopic);
  ASSERT_EQ(infos.size(), 1U);
  EXPECT_EQ(infos.front().qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(infos.front().qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(NodeIntegrationTest, PublishesAtTheConfiguredRateWithoutBursts)
{
  Harness harness{makeSource(40)};

  harness.spinFor(std::chrono::milliseconds(1'000));

  ASSERT_GE(harness.arrivals.size(), 10U);
  EXPECT_LE(harness.arrivals.size(), 22U);

  const double period_ms = 1'000.0 / kRateHz;
  auto shortest = std::chrono::duration<double, std::milli>::max();
  for (std::size_t i = 1U; i < harness.arrivals.size(); ++i) {
    shortest = std::min(
      shortest,
      std::chrono::duration<double, std::milli>(harness.arrivals[i] - harness.arrivals[i - 1U]));
  }
  EXPECT_GE(shortest.count(), 0.4 * period_ms);
}

TEST_F(NodeIntegrationTest, EncodeFailuresPublishNothingAndAreCounted)
{
  Harness harness{makeSource(10, false)};

  harness.spinFor(std::chrono::milliseconds(600));

  EXPECT_TRUE(harness.arrivals.empty());
  EXPECT_GT(harness.node->stats().encode_failures, 0U);
  EXPECT_EQ(harness.node->stats().messages, 0U);
}

TEST_F(NodeIntegrationTest, EndedSourceStopsPublishingAndDiagnosticsReportAnError)
{
  Harness harness{makeSource(2)};

  harness.spinFor(std::chrono::milliseconds(2'500));

  EXPECT_EQ(harness.arrivals.size(), 2U);
  ASSERT_GT(harness.status_count, 0);
  EXPECT_EQ(harness.last_status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(harness.last_status.message, "source ended");
  EXPECT_EQ(harness.value("messages"), "2");
}

TEST_F(NodeIntegrationTest, DiagnosticsCarryTheLatestMapAgeWhileMapsFlow)
{
  Harness harness{makeSource(40)};

  harness.spinFor(std::chrono::milliseconds(1'600));

  ASSERT_GT(harness.status_count, 0);
  EXPECT_EQ(harness.last_status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  const auto age = harness.value("latest_map_age_ms");
  EXPECT_FALSE(age.empty());
  EXPECT_NE(age, "none");
  EXPECT_EQ(harness.value("negative_age_drops"), "0");  // maps before the clock are never held back
}

TEST_F(NodeIntegrationTest, MapsAheadOfTheNodeClockAreNotPublished)
{
  Harness harness{makeSource(40, true, kFarFutureNs)};

  harness.spinFor(std::chrono::milliseconds(600));

  EXPECT_TRUE(harness.arrivals.empty());
  // the pipeline did build maps: they were held back, not missing
  EXPECT_GT(harness.node->stats().messages, 0U);
}

TEST_F(NodeIntegrationTest, HeldBackMapsAreCountedAndDiagnosticsReportThem)
{
  Harness harness{makeSource(40, true, kFarFutureNs)};

  harness.spinFor(std::chrono::milliseconds(1'600));

  EXPECT_GT(harness.node->negativeAgeDrops(), 0U);
  EXPECT_EQ(harness.node->negativeAgeDrops(), harness.node->stats().messages);
  ASSERT_GT(harness.status_count, 0);
  EXPECT_EQ(harness.last_status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(harness.last_status.message, "latest map dropped: sample time ahead of the node clock");
  EXPECT_FALSE(harness.value("negative_age_drops").empty());
  EXPECT_NE(harness.value("negative_age_drops"), "0");
  EXPECT_EQ(harness.value("latest_map_age_ms"), "none");  // nothing was published
}

TEST_F(NodeIntegrationTest, MapsFlowAgainOnceTheNodeClockHasCaughtUp)
{
  // The node runs on simulated time that reads 0, so maps built from times after 3 s are ahead of
  // its clock and are held back. When the clock passes them they must flow again, and the
  // diagnostics must stop reporting the held-back state while still counting the earlier drops.
  Harness harness{makeSource(40, true, 3'000'000'000U), true};

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (harness.node->negativeAgeDrops() == 0U && std::chrono::steady_clock::now() < deadline) {
    harness.spinFor(std::chrono::milliseconds(10));
  }
  ASSERT_GT(harness.node->negativeAgeDrops(), 0U);
  EXPECT_TRUE(harness.arrivals.empty());

  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1'200);
  while (std::chrono::steady_clock::now() < end) {
    harness.setClock(std::chrono::seconds(10));
    harness.spinFor(std::chrono::milliseconds(20));
  }

  EXPECT_GE(harness.arrivals.size(), 10U);
  ASSERT_GT(harness.status_count, 0);
  EXPECT_EQ(harness.last_status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(harness.last_status.message, "running");
  EXPECT_NE(harness.value("negative_age_drops"), "0");
}
