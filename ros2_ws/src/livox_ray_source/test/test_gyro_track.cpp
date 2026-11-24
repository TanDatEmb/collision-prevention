#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "livox_ray_source/gyro_track.hpp"

using livox_ray_source::GyroSample;
using livox_ray_source::GyroTrack;
using livox_ray_source::PoseHistoryConfig;
using livox_ray_source::PoseLookupStatus;

namespace
{

// Test fixtures only; none is an approved platform value.
constexpr uint64_t kMs = 1'000'000U;
constexpr uint64_t kStart = 1'000U * kMs;
const PoseHistoryConfig kConfig{256U, 10U * kMs};

GyroSample sampleAt(uint64_t time_ns, double x, double y, double z)
{
  GyroSample s;
  s.timestamp_ns = time_ns;
  s.rate_flu_rad_s = {x, y, z};
  return s;
}

// Rotation angle about a principal axis of a quaternion that rotates about that axis only.
double angleAbout(const std::array<double, 4> & q, int axis)
{
  return 2.0 * std::atan2(q[static_cast<std::size_t>(axis) + 1U], q[0]);
}

}  // namespace

TEST(GyroTrackTest, InvalidConfigAndSamplesAreRefused)
{
  EXPECT_THROW((GyroTrack{PoseHistoryConfig{1U, kMs}}), std::invalid_argument);
  EXPECT_THROW((GyroTrack{PoseHistoryConfig{8U, 0U}}), std::invalid_argument);
  GyroTrack track{kConfig};
  EXPECT_FALSE(track.push(sampleAt(0U, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(track.push(sampleAt(kStart, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)));
  EXPECT_TRUE(track.push(sampleAt(kStart, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(track.push(sampleAt(kStart, 1.0, 0.0, 0.0)));  // the time of the newest sample
  EXPECT_EQ(track.orientationAt(kStart - 1U).status, PoseLookupStatus::kNoSampleBefore);
}

TEST(GyroTrackTest, ConstantRateIntegratesToRateTimesTime)
{
  GyroTrack track{kConfig};
  const double rate = 3.0;  // rad/s about up
  for (uint64_t i = 0U; i <= 20U; ++i) {
    ASSERT_TRUE(track.push(sampleAt(kStart + i * 5U * kMs, 0.0, 0.0, rate)));
  }
  const auto at = track.orientationAt(kStart + 100U * kMs);
  ASSERT_TRUE(at.ok());
  EXPECT_NEAR(angleAbout(at.pose.q_wxyz, 2), rate * 0.1, 1.0e-12);
  // Between two samples the orientation is interpolated.
  const auto between = track.orientationAt(kStart + 52U * kMs);
  ASSERT_TRUE(between.ok());
  EXPECT_NEAR(angleAbout(between.pose.q_wxyz, 2), rate * 0.052, 1.0e-12);
}

TEST(GyroTrackTest, TrapezoidIsExactForALinearRampWithUnevenSteps)
{
  // Rate alpha * t about left (pitch) with the uneven IMU steps measured (4.1 to 6.0 ms): the angle is
  // alpha t^2 / 2 exactly. Holding each rate over its step would be off by about alpha T dt / 2.
  const double alpha = 1760.0 * 3.14159265358979323846 / 180.0;  // rad/s^2
  GyroTrack track{kConfig};
  uint64_t t = 0U;
  uint64_t last = 0U;
  int i = 0;
  while (t <= 100U * kMs) {
    const double ts = static_cast<double>(t) * 1.0e-9;
    ASSERT_TRUE(track.push(sampleAt(kStart + t, 0.0, alpha * ts, 0.0)));
    last = t;
    t += (i++ % 2 == 0) ? 4'100'000U : 6'000'000U;
  }
  const auto at = track.orientationAt(kStart + last);
  ASSERT_TRUE(at.ok());
  const double ls = static_cast<double>(last) * 1.0e-9;
  EXPECT_NEAR(angleAbout(at.pose.q_wxyz, 1), alpha * ls * ls / 2.0, 1.0e-12);
}

TEST(GyroTrackTest, AfterTheNewestSampleTheNewestRateIsHeldUpToTheGapLimit)
{
  GyroTrack track{kConfig};
  ASSERT_TRUE(track.push(sampleAt(kStart, 0.0, 0.0, 2.0)));
  ASSERT_TRUE(track.push(sampleAt(kStart + 5U * kMs, 0.0, 0.0, 2.0)));
  const auto tail = track.orientationAt(kStart + 9U * kMs);
  ASSERT_TRUE(tail.ok());
  EXPECT_NEAR(angleAbout(tail.pose.q_wxyz, 2), 2.0 * 0.009, 1.0e-12);
  EXPECT_TRUE(track.orientationAt(kStart + 15U * kMs).ok());  // exactly the gap limit after the newest
  EXPECT_EQ(track.orientationAt(kStart + 15U * kMs + 1U).status, PoseLookupStatus::kNoSampleAfter);
}

TEST(GyroTrackTest, AGapStartsANewSegmentThatCannotBeCombinedWithTheOld)
{
  GyroTrack track{kConfig};
  ASSERT_TRUE(track.push(sampleAt(kStart, 0.0, 0.0, 1.0)));
  ASSERT_TRUE(track.push(sampleAt(kStart + 5U * kMs, 0.0, 0.0, 1.0)));
  ASSERT_TRUE(track.push(sampleAt(kStart + 30U * kMs, 0.0, 0.0, 1.0)));  // 25 ms gap: above the limit
  ASSERT_TRUE(track.push(sampleAt(kStart + 35U * kMs, 0.0, 0.0, 1.0)));
  const auto before = track.orientationAt(kStart + 5U * kMs);
  const auto after = track.orientationAt(kStart + 35U * kMs);
  ASSERT_TRUE(before.ok());
  ASSERT_TRUE(after.ok());
  EXPECT_NE(before.pose.reset_counter, after.pose.reset_counter);
  EXPECT_NEAR(angleAbout(after.pose.q_wxyz, 2), 0.005, 1.0e-12);  // restarted from the identity
  EXPECT_EQ(track.orientationAt(kStart + 20U * kMs).status, PoseLookupStatus::kResetInBracket);
}

TEST(GyroTrackTest, ATimeStepBackStartsANewSegmentAndForgetsTheFuture)
{
  // A step of the IMU time stamps, as seen while PTP locked (ADR 0023): earlier stored times are dropped.
  GyroTrack track{kConfig};
  ASSERT_TRUE(track.push(sampleAt(kStart + 500U * kMs, 0.0, 0.0, 1.0)));
  ASSERT_TRUE(track.push(sampleAt(kStart + 505U * kMs, 0.0, 0.0, 1.0)));
  const auto old_segment = track.orientationAt(kStart + 505U * kMs);
  ASSERT_TRUE(old_segment.ok());
  ASSERT_TRUE(track.push(sampleAt(kStart, 0.0, 0.0, 1.0)));
  ASSERT_TRUE(track.push(sampleAt(kStart + 5U * kMs, 0.0, 0.0, 1.0)));
  const auto now = track.orientationAt(kStart + 5U * kMs);
  ASSERT_TRUE(now.ok());
  EXPECT_NE(now.pose.reset_counter, old_segment.pose.reset_counter);
  EXPECT_FALSE(track.orientationAt(kStart + 505U * kMs).ok());
}
