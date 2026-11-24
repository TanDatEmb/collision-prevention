#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "obstacle_sector_node/odometry_pose.hpp"

using obstacle_sector_node::poseFromOdometry;
using px4_msgs::msg::VehicleOdometry;

namespace
{

// Test fixture values, not platform values.
constexpr int64_t kNowNs = 5'000'000'000;  // 5 s

VehicleOdometry odometryAt(uint64_t sample_us)
{
  VehicleOdometry odometry;
  odometry.timestamp = sample_us + 1'000U;
  odometry.timestamp_sample = sample_us;
  odometry.position = {1.0F, 2.0F, -3.0F};
  odometry.q = {0.5F, 0.5F, 0.5F, 0.5F};
  odometry.reset_counter = 7U;
  return odometry;
}

}  // namespace

TEST(OdometryPoseTest, ScalesMicrosecondsToNanosecondsAndCopiesThePose)
{
  const auto pose = poseFromOdometry(odometryAt(4'000'000U), kNowNs);
  ASSERT_TRUE(pose.has_value());
  EXPECT_EQ(pose->timestamp_sample_ns, 4'000'000'000U);
  EXPECT_EQ(pose->reset_counter, 7U);
  EXPECT_DOUBLE_EQ(pose->position_ned_m[0], 1.0);
  EXPECT_DOUBLE_EQ(pose->position_ned_m[1], 2.0);
  EXPECT_DOUBLE_EQ(pose->position_ned_m[2], -3.0);
  EXPECT_DOUBLE_EQ(pose->q_wxyz[0], 0.5);
  EXPECT_TRUE(pose->isValid());
}

TEST(OdometryPoseTest, UnknownTimeIsNotASample)
{
  EXPECT_FALSE(poseFromOdometry(odometryAt(0U), kNowNs).has_value());
  EXPECT_FALSE(
    poseFromOdometry(odometryAt(std::numeric_limits<uint64_t>::max()), kNowNs).has_value());
}

TEST(OdometryPoseTest, PoseAheadOfTheNodeClockIsRejected)
{
  EXPECT_TRUE(poseFromOdometry(odometryAt(5'000'000U), kNowNs).has_value());   // equal is not ahead
  EXPECT_FALSE(poseFromOdometry(odometryAt(5'000'001U), kNowNs).has_value());  // 1 us ahead
  EXPECT_FALSE(poseFromOdometry(odometryAt(4'000'000U), -1).has_value());      // clock not running
}

TEST(OdometryPoseTest, NanQuaternionOrPositionGivesAnInvalidSample)
{
  // Stored as an invalid sample, so that a query across it gets no pose.
  auto invalid_q = odometryAt(4'000'000U);
  invalid_q.q = {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F};
  auto sample = poseFromOdometry(invalid_q, kNowNs);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->isValid());
  EXPECT_EQ(sample->timestamp_sample_ns, 4'000'000'000U);

  auto invalid_position = odometryAt(4'000'000U);
  invalid_position.position = {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
  sample = poseFromOdometry(invalid_position, kNowNs);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->isValid());
}

TEST(OdometryPoseTest, VelocityIsCopiedOnlyInTheNedFrame)
{
  VehicleOdometry odometry = odometryAt(4'000'000U);
  odometry.velocity = {12.0F, -1.0F, 0.5F};
  odometry.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
  auto pose = poseFromOdometry(odometry, kNowNs);
  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->velocity_ned_m_s[0], 12.0);
  EXPECT_DOUBLE_EQ(pose->velocity_ned_m_s[1], -1.0);
  EXPECT_DOUBLE_EQ(pose->velocity_ned_m_s[2], 0.5);
  for (const uint8_t frame : {VehicleOdometry::VELOCITY_FRAME_UNKNOWN, VehicleOdometry::VELOCITY_FRAME_FRD,
      VehicleOdometry::VELOCITY_FRAME_BODY_FRD})
  {
    odometry.velocity_frame = frame;
    pose = poseFromOdometry(odometry, kNowNs);
    ASSERT_TRUE(pose.has_value());
    EXPECT_TRUE(std::isnan(pose->velocity_ned_m_s[0])) << static_cast<int>(frame);
    EXPECT_TRUE(pose->isValid());  // the pose itself stays usable at the reference time
  }
}
