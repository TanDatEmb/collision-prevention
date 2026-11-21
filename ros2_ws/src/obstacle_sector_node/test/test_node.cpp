#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "livox_ray_source/replay_ray_source.hpp"
#include "obstacle_sector_node/node.hpp"

using livox_ray_source::ReplayRaySource;
using livox_ray_source::StreamSample;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;
using obstacle_sector_map::SectorMapConfig;
using obstacle_sector_node::ObstacleSectorNode;

namespace
{

// Test fixture values, not approved platform values.
constexpr SectorMapConfig kConfig{1.0F, 1.0F, 0.5F, 2U, 2U};
constexpr uint64_t kWindowNs = 1'000'000U;

std::unique_ptr<livox_ray_source::RaySource> makeSource()
{
  auto ray = [](uint64_t ts) {
    return StreamSample{
      RaySample{
        ts, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, false,
        RayReturnKind::kQualifiedNoReturn},
      false};
  };
  return std::make_unique<ReplayRaySource>(
    std::vector<StreamSample>{ray(5'000'000U), ray(5'100'000U)}, kWindowNs);
}

class NodeTest : public ::testing::Test
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

TEST(MapAgeTest, IsTheNodeClockMinusTheSampleTime)
{
  using obstacle_sector_node::mapAgeNs;

  EXPECT_EQ(mapAgeNs(2'000'000'000, 1'500'000U), 500'000'000);
  EXPECT_EQ(mapAgeNs(1'000'000, 1'000U), 0);
  EXPECT_LT(mapAgeNs(1'000'000, 2'000U), 0);
}

TEST(MapAgeTest, ASampleTimeBeyondTheNanosecondRangeIsAheadOfAnyClock)
{
  using obstacle_sector_node::mapAgeNs;

  // Nanoseconds in int64 end at 9'223'372'036'854'775 us (year 2262). A later or garbled time must
  // not wrap around and look like an old sample: it is ahead of every clock.
  EXPECT_LT(mapAgeNs(2'000'000'000, std::numeric_limits<uint64_t>::max()), 0);
  EXPECT_LT(mapAgeNs(2'000'000'000, 9'223'372'036'854'776U), 0);
}

TEST(MapAgeTest, TheLastRepresentableSampleTimeKeepsItsExactAge)
{
  using obstacle_sector_node::mapAgeNs;

  // 9'223'372'036'854'775 us is 9'223'372'036'854'775'000 ns, the largest that fits in int64.
  EXPECT_EQ(mapAgeNs(2'000'000'000, 9'223'372'036'854'775U), -9'223'372'034'854'775'000);
}

TEST(DiagnosticLevelTest, ReflectsEndedSourceAndEncodeFailures)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using obstacle_sector_node::diagnosticLevel;
  using obstacle_sector_node::PipelineStats;
  using obstacle_sector_node::StepStatus;

  const PipelineStats none{};
  const PipelineStats some_failures{10U, 8U, 2U};

  EXPECT_EQ(diagnosticLevel(StepStatus::kMessage, none), DiagnosticStatus::OK);
  EXPECT_EQ(diagnosticLevel(StepStatus::kMessage, some_failures), DiagnosticStatus::WARN);
  EXPECT_EQ(diagnosticLevel(StepStatus::kEncodeFailed, some_failures), DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnosticLevel(StepStatus::kSourceEnded, none), DiagnosticStatus::ERROR);
}

TEST(DiagnosticLevelTest, ReflectsMapsHeldBackForANegativeAge)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using obstacle_sector_node::diagnosticLevel;
  using obstacle_sector_node::PipelineStats;
  using obstacle_sector_node::StepStatus;

  const PipelineStats none{};

  // earlier drops warn, a drop of the latest map is an error (nothing reaches PX4), no drops change
  // nothing
  EXPECT_EQ(diagnosticLevel(StepStatus::kMessage, none, 0U, false), DiagnosticStatus::OK);
  EXPECT_EQ(diagnosticLevel(StepStatus::kMessage, none, 3U, false), DiagnosticStatus::WARN);
  EXPECT_EQ(diagnosticLevel(StepStatus::kMessage, none, 3U, true), DiagnosticStatus::ERROR);
}

TEST_F(NodeTest, RejectsANullSourceAndANonPositiveRate)
{
  EXPECT_THROW(ObstacleSectorNode(nullptr, kConfig), std::invalid_argument);

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("sector_map_rate_hz", 0.0)});
  EXPECT_THROW(ObstacleSectorNode(makeSource(), kConfig, options), std::invalid_argument);
}

TEST_F(NodeTest, PublishesTheMapOnTheInterfaceTopic)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("sector_map_rate_hz", 50.0)});
  auto node = std::make_shared<ObstacleSectorNode>(makeSource(), kConfig, options);

  auto listener = std::make_shared<rclcpp::Node>("obstacle_listener");
  std::vector<px4_msgs::msg::ObstacleDistance> received;
  auto subscription = listener->create_subscription<px4_msgs::msg::ObstacleDistance>(
    "/fmu/in/obstacle_distance",
    rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile(),
    [&received](const px4_msgs::msg::ObstacleDistance & message) { received.push_back(message); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(listener);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
  }

  ASSERT_FALSE(received.empty());
  EXPECT_EQ(received.front().timestamp, 5'000U);
  EXPECT_EQ(received.front().distances[0], 4'001U);
  EXPECT_EQ(node->stats().messages, 1U);
}
