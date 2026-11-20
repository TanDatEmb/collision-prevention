#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "obstacle_sector_node/laser_scan_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using livox_ray_source::RayFrame;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_node::laserScanToFrame;
using obstacle_sector_node::LatestFrameQueue;
using obstacle_sector_node::QueuedRaySource;
using obstacle_sector_node::ScanContinuityTracker;
using sensor_msgs::msg::LaserScan;

namespace
{

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

LaserScan makeScan(
  const std::vector<float> & ranges, float angle_min = 0.0F, float range_max = 40.0F)
{
  LaserScan scan;
  scan.header.stamp.sec = 5;
  scan.header.stamp.nanosec = 250'000'000U;
  scan.angle_min = angle_min;
  scan.angle_increment = 0.1F;
  scan.angle_max = angle_min + 0.1F * static_cast<float>(ranges.size());
  scan.time_increment = 0.0F;
  scan.range_min = 0.2F;
  scan.range_max = range_max;
  scan.ranges = ranges;
  return scan;
}

RayFrame frameWithTimestamp(uint64_t timestamp_ns)
{
  RayFrame frame;
  frame.rays.push_back(
    obstacle_sector_map::RaySample{timestamp_ns, 1.0F, 0.0F, 0.0F, false, RayReturnKind::kReturn});
  frame.packet_continuity_valid = true;
  return frame;
}

TEST(LaserScanToFrameTest, ARangeInsideTheLimitsIsAReturnAtTheScanAngleAndTime)
{
  const auto frame = laserScanToFrame(makeScan({2.0F}), true);
  ASSERT_EQ(frame.rays.size(), 1U);
  EXPECT_EQ(frame.rays[0].return_kind, RayReturnKind::kReturn);
  EXPECT_FLOAT_EQ(frame.rays[0].range_m, 2.0F);
  EXPECT_FLOAT_EQ(frame.rays[0].azimuth_rad, 0.0F);
  EXPECT_FLOAT_EQ(frame.rays[0].elevation_rad, 0.0F);
  EXPECT_EQ(frame.rays[0].timestamp_ns, 5'250'000'000U);
  EXPECT_FALSE(frame.rays[0].masked);
  EXPECT_TRUE(frame.packet_continuity_valid);
}

TEST(LaserScanToFrameTest, EachRayHasItsOwnAngleCounterClockwiseFromTheFront)
{
  const auto frame = laserScanToFrame(makeScan({1.0F, 1.0F, 1.0F}, -0.1F), true);
  ASSERT_EQ(frame.rays.size(), 3U);
  EXPECT_NEAR(frame.rays[0].azimuth_rad, -0.1F, 1e-6F);
  EXPECT_NEAR(frame.rays[1].azimuth_rad, 0.0F, 1e-6F);
  EXPECT_NEAR(frame.rays[2].azimuth_rad, 0.1F, 1e-6F);
}

TEST(
  LaserScanToFrameTest,
  ARangeAtOrBeyondTheMaximumOrInfiniteIsAQualifiedNoReturnWhenTheScanCoversTheMap)
{
  // range_max 40 m is the map's own limit (obstacle_sector_map::kMaxDistanceM).
  const auto frame = laserScanToFrame(makeScan({kInf, 40.0F, 41.0F}), true);
  ASSERT_EQ(frame.rays.size(), 3U);
  for (const auto & ray : frame.rays) {
    EXPECT_EQ(ray.return_kind, RayReturnKind::kQualifiedNoReturn);
  }
}

TEST(LaserScanToFrameTest, ANoReturnIsNotClearWhenTheScanReachesLessFarThanTheMap)
{
  // A lidar with range_max 30 m saw nothing up to 30 m and says nothing about 30 to 40 m.
  for (float range_max : {10.0F, 30.0F, 39.99F}) {
    const auto frame =
      laserScanToFrame(makeScan({kInf, range_max, range_max + 1.0F}, 0.0F, range_max), true);
    ASSERT_EQ(frame.rays.size(), 3U);
    for (const auto & ray : frame.rays) {
      EXPECT_EQ(ray.return_kind, RayReturnKind::kInvalid) << "range_max " << range_max;
    }
  }
}

TEST(LaserScanToFrameTest, AReturnStaysAReturnWhateverTheScanRange)
{
  const auto frame = laserScanToFrame(makeScan({2.0F, 29.9F}, 0.0F, 30.0F), true);
  ASSERT_EQ(frame.rays.size(), 2U);
  EXPECT_EQ(frame.rays[0].return_kind, RayReturnKind::kReturn);
  EXPECT_EQ(frame.rays[1].return_kind, RayReturnKind::kReturn);
}

TEST(LaserScanToFrameTest, AScanWithoutAUsableMaximumNeverQualifiesANoReturn)
{
  for (float range_max : {kNaN, kInf, 0.0F, -1.0F}) {
    const auto frame = laserScanToFrame(makeScan({kInf, 5.0F}, 0.0F, range_max), true);
    ASSERT_EQ(frame.rays.size(), 2U);
    EXPECT_EQ(frame.rays[0].return_kind, RayReturnKind::kInvalid);
  }
}

TEST(LaserScanToFrameTest, AnythingElseIsInvalidAndNeverFree)
{
  const auto frame = laserScanToFrame(makeScan({kNaN, -kInf, 0.05F, -1.0F, 0.0F}), true);
  ASSERT_EQ(frame.rays.size(), 5U);
  for (const auto & ray : frame.rays) {
    EXPECT_EQ(ray.return_kind, RayReturnKind::kInvalid);
  }
}

TEST(LaserScanToFrameTest, TheRangeLimitsThemselvesFollowTheHalfOpenRule)
{
  // range_min is a valid return, range_max is already a no-return.
  const auto frame = laserScanToFrame(makeScan({0.2F, 9.99F}), true);
  ASSERT_EQ(frame.rays.size(), 2U);
  EXPECT_EQ(frame.rays[0].return_kind, RayReturnKind::kReturn);
  EXPECT_EQ(frame.rays[1].return_kind, RayReturnKind::kReturn);
}

TEST(LaserScanToFrameTest, TimeIncrementShiftsTheRayTimeAndTheFirstRayKeepsTheHeaderTime)
{
  auto scan = makeScan({1.0F, 1.0F, 1.0F});
  scan.time_increment = 0.001F;
  const auto frame = laserScanToFrame(scan, true);
  ASSERT_EQ(frame.rays.size(), 3U);
  EXPECT_EQ(frame.rays[0].timestamp_ns, 5'250'000'000U);
  EXPECT_NEAR(static_cast<double>(frame.rays[1].timestamp_ns), 5'251'000'000.0, 1.0);
  EXPECT_NEAR(static_cast<double>(frame.rays[2].timestamp_ns), 5'252'000'000.0, 1.0);
}

TEST(LaserScanToFrameTest, AZeroHeaderTimeStaysZeroSoItIsRejectedDownstream)
{
  auto scan = makeScan({1.0F});
  scan.header.stamp.sec = 0;
  scan.header.stamp.nanosec = 0U;
  const auto frame = laserScanToFrame(scan, true);
  ASSERT_EQ(frame.rays.size(), 1U);
  EXPECT_EQ(frame.rays[0].timestamp_ns, 0U);
}

TEST(LaserScanToFrameTest, TheContinuityFlagIsCopiedToTheFrame)
{
  EXPECT_FALSE(laserScanToFrame(makeScan({1.0F}), false).packet_continuity_valid);
  EXPECT_TRUE(laserScanToFrame(makeScan({1.0F}), true).packet_continuity_valid);
}

TEST(LaserScanToFrameTest, AnEmptyScanGivesAnEmptyFrame)
{
  EXPECT_TRUE(laserScanToFrame(makeScan({}), true).rays.empty());
}

TEST(ScanContinuityTrackerTest, ContinuityNeedsAPreviousScanAndTimeMovingForward)
{
  ScanContinuityTracker tracker;
  EXPECT_FALSE(tracker.update(1'000U));  // first scan: nothing before it
  EXPECT_TRUE(tracker.update(2'000U));   // forward
  EXPECT_FALSE(tracker.update(2'000U));  // same time
  EXPECT_FALSE(tracker.update(1'500U));  // backwards
  EXPECT_TRUE(tracker.update(3'000U));   // forward again
}

TEST(ScanContinuityTrackerTest, AZeroTimeNeverCountsAsContinuous)
{
  ScanContinuityTracker tracker;
  EXPECT_FALSE(tracker.update(0U));
  EXPECT_FALSE(tracker.update(0U));
}

TEST(LatestFrameQueueTest, TheNewestFrameWinsAndTheReplacedOneIsCounted)
{
  LatestFrameQueue queue;
  queue.push(frameWithTimestamp(1U));
  queue.push(frameWithTimestamp(2U));
  const auto frame = queue.waitPop(std::chrono::milliseconds(10));
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->rays.at(0).timestamp_ns, 2U);
  EXPECT_EQ(queue.droppedCount(), 1U);
  EXPECT_FALSE(queue.waitPop(std::chrono::milliseconds(5)).has_value());
}

TEST(LatestFrameQueueTest, WaitingTimesOutWhenNothingArrives)
{
  LatestFrameQueue queue;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(queue.waitPop(std::chrono::milliseconds(30)).has_value());
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(25));
}

TEST(LatestFrameQueueTest, WaitingEndsAsSoonAsAFrameIsPushedFromAnotherThread)
{
  LatestFrameQueue queue;
  std::thread producer([&queue]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    queue.push(frameWithTimestamp(7U));
  });
  const auto frame = queue.waitPop(std::chrono::seconds(5));
  producer.join();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->rays.at(0).timestamp_ns, 7U);
}

TEST(QueuedRaySourceTest, RejectsANullQueueAndANonPositiveTimeout)
{
  EXPECT_THROW(QueuedRaySource(nullptr, std::chrono::milliseconds(10)), std::invalid_argument);
  EXPECT_THROW(
    QueuedRaySource(std::make_shared<LatestFrameQueue>(), std::chrono::milliseconds(0)),
    std::invalid_argument);
}

TEST(QueuedRaySourceTest, GivesTheQueuedFrameThenNothingAfterTheTimeout)
{
  auto queue = std::make_shared<LatestFrameQueue>();
  QueuedRaySource source(queue, std::chrono::milliseconds(20));
  queue.get()->push(frameWithTimestamp(9U));
  const auto frame = source.nextFrame();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->rays.at(0).timestamp_ns, 9U);
  EXPECT_FALSE(source.nextFrame().has_value());
}

}  // namespace
