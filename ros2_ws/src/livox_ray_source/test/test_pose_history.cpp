#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "livox_ray_source/fixed_pose_provider.hpp"
#include "livox_ray_source/pose_history.hpp"

using livox_ray_source::FixedPoseProvider;
using livox_ray_source::PoseHistory;
using livox_ray_source::PoseHistoryConfig;
using livox_ray_source::PoseHistoryPushStatus;
using livox_ray_source::PoseLookupResult;
using livox_ray_source::PoseLookupStatus;
using livox_ray_source::PoseSample;

namespace
{

// Test fixture values, not approved parameters.
constexpr std::size_t kFixtureDepth = 8U;
constexpr uint64_t kFixtureMaxGapNs = 100'000'000U;  // 100 ms
constexpr uint64_t kMs = 1'000'000U;
constexpr double kPi = 3.14159265358979323846;

PoseHistoryConfig fixtureConfig()
{
  return PoseHistoryConfig{kFixtureDepth, kFixtureMaxGapNs};
}

// Rotation of `angle_rad` about the down axis (a pure heading change), as a quaternion.
PoseSample yawPose(uint64_t time_ns, double yaw_rad, double north_m, uint8_t reset_counter = 0U)
{
  PoseSample s;
  s.timestamp_sample_ns = time_ns;
  s.q_wxyz = {std::cos(yaw_rad / 2.0), 0.0, 0.0, std::sin(yaw_rad / 2.0)};
  s.position_ned_m = {north_m, 0.0, 0.0};
  s.reset_counter = reset_counter;
  return s;
}

}  // namespace

TEST(PoseHistoryConfigTest, DefaultHoldsTheSentinelsAndIsNotValid)
{
  const PoseHistoryConfig config;
  EXPECT_EQ(config.depth, 0U);
  EXPECT_EQ(config.max_bracket_gap_ns, 0U);
  EXPECT_FALSE(config.isValid());
  EXPECT_THROW(PoseHistory{config}, std::invalid_argument);
}

TEST(PoseHistoryConfigTest, EachRequiredValueIsChecked)
{
  EXPECT_TRUE((PoseHistoryConfig{2U, 1U}).isValid());
  EXPECT_FALSE((PoseHistoryConfig{1U, 1U}).isValid());
  EXPECT_FALSE((PoseHistoryConfig{2U, 0U}).isValid());
  EXPECT_FALSE((PoseHistoryConfig{0U, 1U}).isValid());
}

TEST(PoseSampleTest, DefaultIsNotValidAndNaNQuaternionIsNotValid)
{
  EXPECT_FALSE(PoseSample{}.isValid());
  EXPECT_TRUE(yawPose(1U, 0.0, 0.0).isValid());
  PoseSample nan_q = yawPose(1U, 0.0, 0.0);
  nan_q.q_wxyz[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(nan_q.isValid());
  PoseSample zero_q = yawPose(1U, 0.0, 0.0);
  zero_q.q_wxyz = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(zero_q.isValid());
  PoseSample nan_position = yawPose(1U, 0.0, 0.0);
  nan_position.position_ned_m[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(nan_position.isValid());
  EXPECT_FALSE(yawPose(0U, 0.0, 0.0).isValid());
}

TEST(PoseHistoryTest, EmptyHistoryHasNoPose)
{
  const PoseHistory history{fixtureConfig()};
  EXPECT_EQ(history.poseAt(10U * kMs).status, PoseLookupStatus::kNoSampleBefore);
}

TEST(PoseHistoryTest, InterpolatesMidwayLinearPositionAndSphericalQuaternion)
{
  PoseHistory history{fixtureConfig()};
  ASSERT_EQ(history.push(yawPose(100U * kMs, 0.0, 0.0)), PoseHistoryPushStatus::kStored);
  ASSERT_EQ(history.push(yawPose(200U * kMs, kPi / 2.0, 1.0)), PoseHistoryPushStatus::kStored);

  const PoseLookupResult r = history.poseAt(150U * kMs);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.pose.timestamp_sample_ns, 150U * kMs);
  EXPECT_NEAR(r.pose.position_ned_m[0], 0.5, 1e-12);
  // Halfway between yaw 0 and yaw 90 degrees is yaw 45 degrees.
  EXPECT_NEAR(r.pose.q_wxyz[0], std::cos(kPi / 8.0), 1e-12);
  EXPECT_NEAR(r.pose.q_wxyz[3], std::sin(kPi / 8.0), 1e-12);
  EXPECT_NEAR(r.pose.q_wxyz[1], 0.0, 1e-12);
  EXPECT_NEAR(r.pose.q_wxyz[2], 0.0, 1e-12);
}

TEST(PoseHistoryTest, InterpolationUsesTheTimeFractionNotTheMidpoint)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0));
  history.push(yawPose(200U * kMs, kPi / 2.0, 1.0));
  const PoseLookupResult r = history.poseAt(125U * kMs);
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(r.pose.position_ned_m[0], 0.25, 1e-12);
  EXPECT_NEAR(r.pose.q_wxyz[3], std::sin(kPi / 16.0), 1e-12);
}

TEST(PoseHistoryTest, TakesTheShortestRotationWhenTheQuaternionSignFlips)
{
  PoseHistory history{fixtureConfig()};
  PoseSample a = yawPose(100U * kMs, 0.0, 0.0);
  PoseSample b = yawPose(200U * kMs, kPi / 2.0, 0.0);
  for (double & v : b.q_wxyz) {v = -v;}  // the same attitude with the opposite sign
  history.push(a);
  history.push(b);
  const PoseLookupResult r = history.poseAt(150U * kMs);
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(std::fabs(r.pose.q_wxyz[0]), std::cos(kPi / 8.0), 1e-12);
  EXPECT_NEAR(std::fabs(r.pose.q_wxyz[3]), std::sin(kPi / 8.0), 1e-12);
}

TEST(PoseHistoryTest, ResultIsAUnitQuaternion)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0));
  history.push(yawPose(200U * kMs, 0.3, 0.0));
  const PoseLookupResult r = history.poseAt(137U * kMs);
  ASSERT_TRUE(r.ok());
  double norm2 = 0.0;
  for (double v : r.pose.q_wxyz) {norm2 += v * v;}
  EXPECT_NEAR(norm2, 1.0, 1e-12);
}

TEST(PoseHistoryTest, NoSampleBeforeTheQueryTime)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0));
  history.push(yawPose(150U * kMs, 0.0, 0.0));
  EXPECT_EQ(history.poseAt(99U * kMs).status, PoseLookupStatus::kNoSampleBefore);
}

TEST(PoseHistoryTest, NoSampleAfterTheQueryTime)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0));
  history.push(yawPose(150U * kMs, 0.0, 0.0));
  EXPECT_EQ(history.poseAt(151U * kMs).status, PoseLookupStatus::kNoSampleAfter);
}

TEST(PoseHistoryTest, QueryOnTheExactEdgesReturnsTheSample)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0));
  history.push(yawPose(150U * kMs, 0.0, 2.0));
  const PoseLookupResult first = history.poseAt(100U * kMs);
  ASSERT_TRUE(first.ok());
  EXPECT_DOUBLE_EQ(first.pose.position_ned_m[0], 0.0);
  const PoseLookupResult last = history.poseAt(150U * kMs);
  ASSERT_TRUE(last.ok());
  EXPECT_DOUBLE_EQ(last.pose.position_ned_m[0], 2.0);
}

TEST(PoseHistoryTest, GapEqualToTheMaximumInterpolatesAndAboveItGivesNoPose)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0));
  history.push(yawPose(100U * kMs + kFixtureMaxGapNs, 0.0, 1.0));
  EXPECT_TRUE(history.poseAt(150U * kMs).ok());

  PoseHistory wide{fixtureConfig()};
  wide.push(yawPose(100U * kMs, 0.0, 0.0));
  wide.push(yawPose(100U * kMs + kFixtureMaxGapNs + 1U, 0.0, 1.0));
  EXPECT_EQ(wide.poseAt(150U * kMs).status, PoseLookupStatus::kBracketGapTooLarge);
  // A query that equals a sample needs no bracket.
  EXPECT_TRUE(wide.poseAt(100U * kMs).ok());
}

TEST(PoseHistoryTest, ResetCounterChangeInsideTheBracketGivesNoPose)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0, 3U));
  history.push(yawPose(150U * kMs, 0.0, 5.0, 4U));
  EXPECT_EQ(history.poseAt(125U * kMs).status, PoseLookupStatus::kResetInBracket);
  // Outside the changed bracket the same counter interpolates again.
  history.push(yawPose(200U * kMs, 0.0, 6.0, 4U));
  const PoseLookupResult r = history.poseAt(175U * kMs);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.pose.reset_counter, 4U);
  EXPECT_NEAR(r.pose.position_ned_m[0], 5.5, 1e-12);
}

TEST(PoseHistoryTest, ResetCounterWrapIsAChange)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0, 255U));
  history.push(yawPose(150U * kMs, 0.0, 0.0, 0U));
  EXPECT_EQ(history.poseAt(125U * kMs).status, PoseLookupStatus::kResetInBracket);
}

TEST(PoseHistoryTest, NaNQuaternionInTheBracketGivesNoPose)
{
  PoseHistory history{fixtureConfig()};
  history.push(yawPose(100U * kMs, 0.0, 0.0));
  PoseSample bad = yawPose(150U * kMs, 0.0, 0.0);
  bad.q_wxyz[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(history.push(bad), PoseHistoryPushStatus::kStored);
  history.push(yawPose(200U * kMs, 0.0, 0.0));
  EXPECT_EQ(history.poseAt(125U * kMs).status, PoseLookupStatus::kInvalidSampleInBracket);
  EXPECT_EQ(history.poseAt(175U * kMs).status, PoseLookupStatus::kInvalidSampleInBracket);
  // On a valid sample the query still answers.
  EXPECT_TRUE(history.poseAt(100U * kMs).ok());
  EXPECT_TRUE(history.poseAt(200U * kMs).ok());
  // The invalid sample itself gives no pose.
  EXPECT_EQ(history.poseAt(150U * kMs).status, PoseLookupStatus::kInvalidSampleInBracket);
}

TEST(PoseHistoryTest, RejectsZeroTimeDuplicatesAndOutOfOrderSamples)
{
  PoseHistory history{fixtureConfig()};
  EXPECT_EQ(history.push(yawPose(0U, 0.0, 0.0)), PoseHistoryPushStatus::kRejectedNoTime);
  EXPECT_EQ(history.push(yawPose(100U * kMs, 0.0, 0.0)), PoseHistoryPushStatus::kStored);
  EXPECT_EQ(history.push(yawPose(100U * kMs, 0.0, 1.0)), PoseHistoryPushStatus::kRejectedNotNewer);
  EXPECT_EQ(history.push(yawPose(90U * kMs, 0.0, 1.0)), PoseHistoryPushStatus::kRejectedNotNewer);
  EXPECT_EQ(history.size(), 1U);
}

TEST(PoseHistoryTest, KeepsOnlyTheConfiguredDepth)
{
  PoseHistory history{PoseHistoryConfig{3U, kFixtureMaxGapNs}};
  for (uint64_t i = 1U; i <= 5U; ++i) {
    history.push(yawPose(i * 10U * kMs, 0.0, static_cast<double>(i)));
  }
  EXPECT_EQ(history.size(), 3U);
  EXPECT_EQ(history.poseAt(25U * kMs).status, PoseLookupStatus::kNoSampleBefore);
  EXPECT_TRUE(history.poseAt(35U * kMs).ok());
  EXPECT_TRUE(history.poseAt(50U * kMs).ok());
}

TEST(FixedPoseProviderTest, ReturnsTheSamePoseAtTheQueryTime)
{
  const FixedPoseProvider provider{yawPose(1U, 0.5, 3.0)};
  const PoseLookupResult r = provider.poseAt(777U);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.pose.timestamp_sample_ns, 777U);
  EXPECT_DOUBLE_EQ(r.pose.position_ned_m[0], 3.0);
}

TEST(FixedPoseProviderTest, CanStandForAMissingPose)
{
  const FixedPoseProvider provider{PoseLookupResult{PoseLookupStatus::kNoSampleAfter, {}}};
  EXPECT_EQ(provider.poseAt(1U).status, PoseLookupStatus::kNoSampleAfter);
}
