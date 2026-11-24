#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "livox_ray_source/masking_ray_source.hpp"
#include "livox_ray_source/replay_ray_source.hpp"

using livox_ray_source::MaskCylinder;
using livox_ray_source::MaskingRaySource;
using livox_ray_source::RaySource;
using livox_ray_source::ReplayRaySource;
using livox_ray_source::SelfMask;
using livox_ray_source::StreamSample;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;

namespace
{

// Test fixture geometry, not an approved platform value.
const MaskCylinder kCylinder{1.0, 0.0, 0.2, -0.1, 0.1};
// A sensor at the body origin with no rotation: the body frame is the sensor frame.
const livox_ray_source::LevelingConfig kAtSensor{0.0, 0.0, 0.0, {0.0, 0.0, 0.0}};
constexpr double kMaxDistanceM = 40.0;
constexpr uint64_t kWindowNs = 1'000'000U;

StreamSample makeReturn(uint64_t timestamp_ns, float range_m, float azimuth_rad)
{
  return StreamSample{
    RaySample{timestamp_ns, range_m, azimuth_rad, 0.0F, false, RayReturnKind::kReturn}, false};
}

StreamSample makeNoReturn(uint64_t timestamp_ns, float azimuth_rad)
{
  return StreamSample{
    RaySample{
      timestamp_ns, std::numeric_limits<float>::quiet_NaN(), azimuth_rad, 0.0F, false,
      RayReturnKind::kQualifiedNoReturn},
    false};
}

}  // namespace

TEST(MaskingRaySourceTest, MasksSelfReturnsAndCrossingNoReturnsOnly)
{
  auto inner = std::make_unique<ReplayRaySource>(
    std::vector<StreamSample>{
      makeReturn(5'000U, 1.0F, 0.0F),  // end point inside the cylinder
      makeReturn(5'100U, 1.0F, 0.5F),  // end point outside
      makeNoReturn(5'200U, 0.0F),      // segment crosses the cylinder
      makeNoReturn(5'300U, 0.5F)},     // segment passes beside it
    kWindowNs);
  MaskingRaySource source{std::move(inner), SelfMask{{kCylinder}, kAtSensor}, kMaxDistanceM};

  const auto frame = source.nextFrame();
  ASSERT_TRUE(frame.has_value());
  ASSERT_EQ(frame->rays.size(), 4U);
  EXPECT_TRUE(frame->rays[0].masked);
  EXPECT_FALSE(frame->rays[1].masked);
  EXPECT_TRUE(frame->rays[2].masked);
  EXPECT_FALSE(frame->rays[3].masked);
  EXPECT_FALSE(source.nextFrame().has_value());
}

TEST(MaskingRaySourceTest, KeepsContinuityAndRayContent)
{
  StreamSample broken = makeReturn(5'000U, 12.0F, 0.5F);
  broken.continuity_break = true;
  auto inner = std::make_unique<ReplayRaySource>(std::vector<StreamSample>{broken}, kWindowNs);
  MaskingRaySource source{std::move(inner), SelfMask{{kCylinder}, kAtSensor}, kMaxDistanceM};

  const auto frame = source.nextFrame();
  ASSERT_TRUE(frame.has_value());
  EXPECT_FALSE(frame->packet_continuity_valid);
  EXPECT_FLOAT_EQ(frame->rays[0].range_m, 12.0F);
  EXPECT_FLOAT_EQ(frame->rays[0].azimuth_rad, 0.5F);
}

TEST(MaskingRaySourceTest, RejectsNullSourceAndBadMaxDistance)
{
  EXPECT_THROW(
    MaskingRaySource(nullptr, SelfMask{{kCylinder}, kAtSensor}, kMaxDistanceM), std::invalid_argument);

  for (const double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    auto inner = std::make_unique<ReplayRaySource>(std::vector<StreamSample>{}, kWindowNs);
    EXPECT_THROW(
      MaskingRaySource(std::move(inner), SelfMask{{kCylinder}, kAtSensor}, bad), std::invalid_argument);
  }
}
