#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <initializer_list>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "livox_ray_source/gyro_track.hpp"
#include "livox_ray_source/leveling_ray_source.hpp"
#include "obstacle_sector_map/builder.hpp"

using livox_ray_source::GyroSample;
using livox_ray_source::GyroTrack;
using livox_ray_source::LevelingConfig;
using livox_ray_source::LevelingRaySource;
using livox_ray_source::PoseHistory;
using livox_ray_source::PoseHistoryConfig;
using livox_ray_source::PoseSample;
using livox_ray_source::RayFrame;
using livox_ray_source::RaySource;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;

namespace
{

// Test fixture values, not approved platform values.
constexpr float kTol = 1.0e-5F;

// Delivers the frames it was given, in order.
class FrameListSource : public RaySource
{
public:
  explicit FrameListSource(std::deque<RayFrame> frames)
  : _frames(std::move(frames)) {}

  std::optional<RayFrame> nextFrame() override
  {
    if (_frames.empty()) {
      return std::nullopt;
    }
    RayFrame f = std::move(_frames.front());
    _frames.pop_front();
    return f;
  }

private:
  std::deque<RayFrame> _frames;
};

// 1 m per 1000 ns: the speed of the fixtures whose x grows by 1 m per microsecond.
constexpr double kMetrePerMicrosecond = 1.0e6;  // m/s

PoseSample poseAtX(
  uint64_t time_ns, double x_m, double z_ned_m = 0.0, double vx_m_s = kMetrePerMicrosecond,
  double vz_m_s = 0.0)
{
  PoseSample s;
  s.timestamp_sample_ns = time_ns;
  s.q_wxyz = {1.0, 0.0, 0.0, 0.0};  // level, heading north
  s.position_ned_m = {x_m, 0.0, z_ned_m};
  s.velocity_ned_m_s = {vx_m_s, 0.0, vz_m_s};
  return s;
}

// A gyro that does not turn, with samples every 100 ns over [from_ns, to_ns] and a gap limit of 500 ns.
class StillGyro : public GyroTrack
{
public:
  StillGyro(uint64_t from_ns = 100U, uint64_t to_ns = 4'000U)
  : GyroTrack(PoseHistoryConfig{256U, 500U})
  {
    add(from_ns, to_ns);
  }
  void add(uint64_t from_ns, uint64_t to_ns)
  {
    for (uint64_t t = from_ns; t <= to_ns; t += 100U) {
      GyroSample g;
      g.timestamp_ns = t;
      g.rate_flu_rad_s = {0.0, 0.0, 0.0};
      push(g);
    }
  }
};

RaySample returnRay(uint64_t time_ns, float range_m, float azimuth_rad = 0.0F)
{
  return RaySample{time_ns, range_m, azimuth_rad, 0.0F, false, RayReturnKind::kReturn};
}

RaySample noReturnRay(uint64_t time_ns)
{
  return RaySample{
    time_ns, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.1F, false,
    RayReturnKind::kQualifiedNoReturn};
}

LevelingConfig zeroMountConfig()
{
  return LevelingConfig{0.0, 0.0, 0.0, {0.0, 0.0, 0.0}};
}

// A pose history holding the given samples (PoseHistory itself cannot be moved).
class History : public PoseHistory
{
public:
  History(std::initializer_list<PoseSample> samples)
  : PoseHistory(PoseHistoryConfig{8U, 1'000'000'000U})
  {
    for (const auto & sample : samples) {
      push(sample);
    }
  }
};

std::unique_ptr<RaySource> oneFrame(RayFrame frame, bool continuity = true)
{
  frame.packet_continuity_valid = continuity;
  std::deque<RayFrame> frames;
  frames.push_back(std::move(frame));
  return std::make_unique<FrameListSource>(std::move(frames));
}

}  // namespace

TEST(LevelingRaySourceTest, NullInnerSourceThrows)
{
  const History poses{poseAtX(500U, 0.0), poseAtX(1500U, 1.0), poseAtX(2500U, 2.0)};
  const StillGyro gyro;
  EXPECT_THROW(LevelingRaySource(nullptr, poses, zeroMountConfig()), std::invalid_argument);
}

TEST(LevelingRaySourceTest, InvalidConfigThrows)
{
  const History poses{poseAtX(500U, 0.0), poseAtX(1500U, 1.0), poseAtX(2500U, 2.0)};
  const StillGyro gyro;
  EXPECT_THROW(
    LevelingRaySource(oneFrame(RayFrame{}), poses, LevelingConfig{}), std::invalid_argument);
}

TEST(LevelingRaySourceTest, ReferenceIsTheEarliestRayAndLaterReturnsAreShifted)
{
  const History poses{poseAtX(500U, 0.0), poseAtX(1500U, 1.0), poseAtX(2500U, 2.0)};
  const StillGyro gyro;
  // The earliest ray is the second one (1000 ns, x = 0.5 m). The ray at 2000 ns is taken at
  // x = 1.5 m: 1 m ahead of the reference position, so its 4 m return is 5 m away from it.
  RayFrame frame;
  frame.rays = {returnRay(2000U, 4.0F), returnRay(1000U, 4.0F)};
  LevelingRaySource source{oneFrame(frame), poses, zeroMountConfig(), &gyro};

  const auto out = source.nextFrame();
  ASSERT_TRUE(out.has_value());
  ASSERT_EQ(out->rays.size(), 2U);
  EXPECT_NEAR(out->rays[0].range_m, 5.0F, kTol);
  EXPECT_NEAR(out->rays[1].range_m, 4.0F, kTol);
  EXPECT_TRUE(out->packet_continuity_valid);
  EXPECT_FALSE(source.nextFrame().has_value());
}

TEST(LevelingRaySourceTest, RayWithoutPoseIsRemovedAndTheFrameIsDiscontinuous)
{
  const History poses{poseAtX(500U, 0.0), poseAtX(1500U, 1.0), poseAtX(2500U, 2.0)};
  const StillGyro gyro;
  RayFrame frame;  // 9000 ns is past the gyro samples (to 4000 ns) and their 500 ns tail
  frame.rays = {returnRay(1000U, 4.0F), returnRay(9000U, 4.0F), returnRay(2000U, 3.0F)};
  LevelingRaySource source{oneFrame(frame), poses, zeroMountConfig(), &gyro};

  const auto out = source.nextFrame();
  ASSERT_TRUE(out.has_value());
  ASSERT_EQ(out->rays.size(), 2U);
  EXPECT_NEAR(out->rays[0].range_m, 4.0F, kTol);
  EXPECT_NEAR(out->rays[1].range_m, 4.0F, kTol);  // 3 m ahead of x = 1.5 is 4 m from x = 0.5
  EXPECT_FALSE(out->packet_continuity_valid);
  EXPECT_EQ(source.raysDropped(), 1U);
}

TEST(LevelingRaySourceTest, NoReferencePoseRemovesEveryRay)
{
  const History poses{poseAtX(500U, 0.0), poseAtX(1500U, 1.0), poseAtX(2500U, 2.0)};
  const StillGyro gyro;
  RayFrame frame;  // the earliest ray (100 ns) is before the oldest pose
  frame.rays = {returnRay(100U, 4.0F), returnRay(1000U, 4.0F)};
  LevelingRaySource source{oneFrame(frame), poses, zeroMountConfig(), &gyro};

  const auto out = source.nextFrame();
  ASSERT_TRUE(out.has_value());
  EXPECT_TRUE(out->rays.empty());
  EXPECT_FALSE(out->packet_continuity_valid);
  EXPECT_EQ(source.raysDropped(), 2U);
}

TEST(LevelingRaySourceTest, ARayAfterAGyroRestartIsRemovedAndTheFrameIsDiscontinuous)
{
  // The gyro has no sample between 1500 ns and 2600 ns (a gap above its 500 ns limit): the ray at 3000 ns is
  // in another integration than the reference ray (1000 ns), so it is dropped and the reference ray is kept.
  // An FMU reset after the reference does not matter (ADR 0023): the FMU is used at the reference only.
  PoseSample p1 = poseAtX(2500U, 0.0, 0.0, 0.0);
  p1.reset_counter = 1U;
  const History poses{poseAtX(500U, 0.0, 0.0, 0.0), poseAtX(1500U, 0.0, 0.0, 0.0), p1};
  StillGyro gyro{100U, 1'500U};
  gyro.add(2'600U, 4'000U);
  RayFrame frame;
  frame.rays = {returnRay(1000U, 4.0F), returnRay(3000U, 4.0F)};
  LevelingRaySource source{oneFrame(frame), poses, zeroMountConfig(), &gyro};
  const auto out = source.nextFrame();
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->rays.size(), 1U);
  EXPECT_FALSE(out->packet_continuity_valid);
  EXPECT_EQ(source.raysDropped(), 1U);
}

TEST(LevelingRaySourceTest, FrameBoundsAreTheLargestRiseAndFallOverTheKeptRays)
{
  // The FMU velocity at the reference (1000 ns) carries the sensor: climbing 0.2 m per microsecond, the rays at
  // 1500 and 2500 ns are 0.1 and 0.3 m higher; sinking at the same rate they are lower by as much.
  const StillGyro gyro;
  for (const double vz : {-2.0e5, 2.0e5}) {  // NED: negative is up
    const History poses{poseAtX(500U, 0.0, 0.0, 0.0, vz), poseAtX(1500U, 0.0, 0.0, 0.0, vz)};
    RayFrame frame;
    frame.rays = {noReturnRay(1000U), noReturnRay(1500U), noReturnRay(2500U)};
    LevelingRaySource source{oneFrame(frame), poses, zeroMountConfig(), &gyro};
    const auto out = source.nextFrame();
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->rays.size(), 3U);  // no ray is dropped for motion
    EXPECT_TRUE(out->packet_continuity_valid);
    EXPECT_NEAR(out->origin_rise_bound_m, vz < 0.0 ? 0.3 : 0.0, 1.0e-9) << vz;
    EXPECT_NEAR(out->origin_fall_bound_m, vz < 0.0 ? 0.0 : 0.3, 1.0e-9) << vz;
  }
}

TEST(LevelingRaySourceTest, BoundsAreZeroForAFrameThatDoesNotMove)
{
  const History poses{poseAtX(500U, 0.0, 0.0, 0.0), poseAtX(2500U, 0.0, 0.0, 0.0)};
  const StillGyro gyro;
  RayFrame frame;
  frame.rays = {returnRay(1000U, 4.0F), noReturnRay(2000U)};
  LevelingRaySource source{oneFrame(frame), poses, zeroMountConfig(), &gyro};
  const auto out = source.nextFrame();
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->origin_rise_bound_m, 0.0);
  EXPECT_EQ(out->origin_fall_bound_m, 0.0);
}

TEST(LevelingRaySourceTest, FrameWithoutAnyRayTimeIsPassedThroughAndInnerDiscontinuityIsKept)
{
  const History poses{poseAtX(500U, 0.0), poseAtX(1500U, 1.0), poseAtX(2500U, 2.0)};
  const StillGyro gyro;
  RayFrame frame;
  frame.rays = {returnRay(0U, 4.0F)};
  LevelingRaySource source{oneFrame(frame, false), poses, zeroMountConfig(), &gyro};
  const auto out = source.nextFrame();
  ASSERT_TRUE(out.has_value());
  ASSERT_EQ(out->rays.size(), 1U);
  EXPECT_EQ(out->rays[0].range_m, 4.0F);
  EXPECT_FALSE(out->packet_continuity_valid);
  EXPECT_TRUE(std::isnan(out->origin_rise_bound_m));  // not leveled, so not provided
  EXPECT_TRUE(std::isnan(out->origin_fall_bound_m));

  RayFrame ok_frame;
  ok_frame.rays = {returnRay(1000U, 4.0F)};
  LevelingRaySource kept{oneFrame(ok_frame, false), poses, zeroMountConfig(), &gyro};
  const auto out2 = kept.nextFrame();
  ASSERT_TRUE(out2.has_value());
  EXPECT_FALSE(out2->packet_continuity_valid);  // a discontinuity of the inner source is never cleared
}

TEST(LevelingRaySourceTest, FrameBoundsShortenTheFreeDistanceInTheSectorMap)
{
  // The sensor climbs 0.3 m between the reference ray (1000 ns) and the later rays (2000 ns). With a
  // band 1 m above the origin, a no-return 3 degrees up now certifies 0.7 m of room, not 1 m.
  const History poses{
    poseAtX(500U, 0.0, 0.0, 0.0, -3.0e5), poseAtX(1500U, 0.0, -0.3, 0.0, -3.0e5),
    poseAtX(2500U, 0.0, -0.6, 0.0, -3.0e5)};
  const StillGyro gyro;
  const float three = 3.0F * 3.14159265358979323846F / 180.0F;
  const auto no_return_at = [three](uint64_t time_ns, float elevation_sign) {
      RaySample r = noReturnRay(time_ns);
      r.elevation_rad = elevation_sign * three;
      return r;
    };
  RayFrame frame;
  frame.rays = {no_return_at(1000U, 1.0F), no_return_at(1000U, -1.0F), no_return_at(2000U, 1.0F),
    no_return_at(2000U, -1.0F)};
  LevelingRaySource source{oneFrame(frame), poses, zeroMountConfig(), &gyro};
  const auto out = source.nextFrame();
  ASSERT_TRUE(out.has_value());

  obstacle_sector_map::SectorMapBuilder builder{obstacle_sector_map::SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U}};
  builder.reset(
    out->packet_continuity_valid, static_cast<float>(out->origin_rise_bound_m),
    static_cast<float>(out->origin_fall_bound_m));
  for (const auto & ray : out->rays) {
    builder.addRay(ray);
  }
  const auto sector = builder.finalize().sectors[0];
  EXPECT_NEAR(sector.distance_m, 0.7F / std::tan(three), 1.0e-3F);
}

TEST(LevelingRaySourceTest, ReferenceIsTheEarliestRayThatTheSectorMapTakes)
{
  // The earliest ray (1000 ns) is masked and the sector map rejects it, so the map timestamp is 2000 ns
  // and the reference must be 2000 ns: the ray at 2000 ns is not shifted, the one at 2500 ns (0.5 m ahead
  // of it) is. The same holds for an invalid return and a ray that is not finite.
  const History poses{poseAtX(500U, 0.0), poseAtX(1500U, 1.0), poseAtX(2500U, 2.0), poseAtX(3500U, 3.0)};
  const StillGyro gyro;
  RaySample masked = returnRay(1000U, 4.0F);
  masked.masked = true;
  RaySample not_finite = returnRay(1200U, 4.0F);
  not_finite.azimuth_rad = std::numeric_limits<float>::quiet_NaN();
  RaySample invalid_return = returnRay(1300U, -1.0F);
  RayFrame frame;
  frame.rays = {masked, not_finite, invalid_return, returnRay(2000U, 4.0F), returnRay(2500U, 4.0F)};
  LevelingRaySource source{oneFrame(frame), poses, zeroMountConfig(), &gyro};

  const auto out = source.nextFrame();
  ASSERT_TRUE(out.has_value());
  ASSERT_EQ(out->rays.size(), 5U);
  EXPECT_NEAR(out->rays[3].range_m, 4.0F, kTol);  // the reference ray itself
  EXPECT_NEAR(out->rays[4].range_m, 4.5F, kTol);  // moved 0.5 m since the reference
  EXPECT_TRUE(out->packet_continuity_valid);
}
