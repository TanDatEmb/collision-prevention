#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "livox_ray_source/frame_segmenter.hpp"
#include "livox_ray_source/replay_ray_source.hpp"
#include "obstacle_sector_map/builder.hpp"

using livox_ray_source::FrameSegmenter;
using livox_ray_source::RayFrame;
using livox_ray_source::ReplayRaySource;
using livox_ray_source::StreamSample;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;
using obstacle_sector_map::SectorMapBuilder;
using obstacle_sector_map::SectorMapConfig;
using obstacle_sector_map::SectorState;

namespace
{

constexpr uint64_t kWindowNs = 1'000U;
constexpr SectorMapConfig kConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 40.0F, 0.0F};

StreamSample makeNoReturn(uint64_t timestamp_ns, float azimuth_rad = 0.0F, bool brk = false)
{
  RaySample ray{timestamp_ns, std::numeric_limits<float>::quiet_NaN(), azimuth_rad, 0.0F,
                false,        RayReturnKind::kQualifiedNoReturn};
  return StreamSample{ray, brk};
}

StreamSample makeReturn(uint64_t timestamp_ns, float range_m, float azimuth_rad = 0.0F)
{
  RaySample ray{timestamp_ns, range_m, azimuth_rad, 0.1F, false, RayReturnKind::kReturn};
  return StreamSample{ray, false};
}

}  // namespace

TEST(FrameSegmenterTest, RejectsZeroWindow)
{
  EXPECT_THROW(FrameSegmenter(0U), std::invalid_argument);
  EXPECT_THROW(ReplayRaySource({}, 0U), std::invalid_argument);
}

TEST(FrameSegmenterTest, SplitsFramesByWindowFromFirstRay)
{
  FrameSegmenter segmenter{kWindowNs};

  EXPECT_FALSE(segmenter.push(makeNoReturn(5'000U)).has_value());
  EXPECT_FALSE(segmenter.push(makeNoReturn(5'999U)).has_value());

  const auto first = segmenter.push(makeNoReturn(6'000U));
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->rays.size(), 2U);
  EXPECT_TRUE(first->packet_continuity_valid);

  const auto second = segmenter.flush();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->rays.size(), 1U);
  EXPECT_FALSE(segmenter.flush().has_value());
}

TEST(FrameSegmenterTest, ContinuityBreakMarksOnlyItsFrame)
{
  FrameSegmenter segmenter{kWindowNs};
  EXPECT_FALSE(segmenter.push(makeNoReturn(5'000U)).has_value());
  EXPECT_FALSE(segmenter.push(makeNoReturn(5'100U, 0.0F, true)).has_value());

  const auto broken = segmenter.push(makeNoReturn(6'000U));
  ASSERT_TRUE(broken.has_value());
  EXPECT_FALSE(broken->packet_continuity_valid);

  const auto clean = segmenter.flush();
  ASSERT_TRUE(clean.has_value());
  EXPECT_TRUE(clean->packet_continuity_valid);
}

TEST(FrameSegmenterTest, BackwardTimestampBreaksContinuity)
{
  FrameSegmenter segmenter{kWindowNs};
  EXPECT_FALSE(segmenter.push(makeNoReturn(5'500U)).has_value());
  EXPECT_FALSE(segmenter.push(makeNoReturn(5'400U)).has_value());

  const auto frame = segmenter.flush();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->rays.size(), 2U);
  EXPECT_FALSE(frame->packet_continuity_valid);
}

TEST(FrameSegmenterTest, ZeroTimestampDoesNotStartAWindow)
{
  FrameSegmenter segmenter{kWindowNs};
  EXPECT_FALSE(segmenter.push(makeNoReturn(0U)).has_value());
  EXPECT_FALSE(segmenter.push(makeNoReturn(9'000U)).has_value());
  EXPECT_FALSE(segmenter.push(makeNoReturn(9'500U)).has_value());

  const auto frame = segmenter.flush();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->rays.size(), 3U);
  EXPECT_EQ(frame->rays.front().timestamp_ns, 0U);
}

TEST(ReplayRaySourceTest, PreservesDirectionAndKindOfReturnsAndNoReturns)
{
  constexpr float kAzimuthRad = 1.25F;
  ReplayRaySource source{
    {makeReturn(5'000U, 12.0F, kAzimuthRad), makeNoReturn(5'100U, -kAzimuthRad)}, kWindowNs};

  const auto frame = source.nextFrame();
  ASSERT_TRUE(frame.has_value());
  ASSERT_EQ(frame->rays.size(), 2U);

  EXPECT_EQ(frame->rays[0].return_kind, RayReturnKind::kReturn);
  EXPECT_FLOAT_EQ(frame->rays[0].azimuth_rad, kAzimuthRad);
  EXPECT_FLOAT_EQ(frame->rays[0].range_m, 12.0F);

  EXPECT_EQ(frame->rays[1].return_kind, RayReturnKind::kQualifiedNoReturn);
  EXPECT_FLOAT_EQ(frame->rays[1].azimuth_rad, -kAzimuthRad);
  EXPECT_TRUE(std::isnan(frame->rays[1].range_m));
}

TEST(ReplayRaySourceTest, YieldsAllFramesThenNothing)
{
  ReplayRaySource source{
    {makeNoReturn(5'000U), makeNoReturn(6'000U), makeNoReturn(6'100U), makeNoReturn(7'200U)},
    kWindowNs};

  std::vector<std::size_t> sizes;
  while (const auto frame = source.nextFrame()) {
    sizes.push_back(frame->rays.size());
  }

  EXPECT_EQ(sizes, (std::vector<std::size_t>{1U, 2U, 1U}));
  EXPECT_FALSE(source.nextFrame().has_value());
}

TEST(ReplayRaySourceTest, EmptyStreamHasNoFrames)
{
  ReplayRaySource source{{}, kWindowNs};
  EXPECT_FALSE(source.nextFrame().has_value());
}

TEST(ReplayRaySourceTest, FrameDrivesSectorMapBuilderPerContract)
{
  ReplayRaySource clean{{makeNoReturn(5'000U), makeNoReturn(5'100U)}, kWindowNs};
  ReplayRaySource lossy{{makeNoReturn(5'000U), makeNoReturn(5'100U, 0.0F, true)}, kWindowNs};

  for (const bool expect_clear : {true, false}) {
    const auto frame = (expect_clear ? clean : lossy).nextFrame();
    ASSERT_TRUE(frame.has_value());

    SectorMapBuilder builder{kConfig};
    builder.reset(frame->packet_continuity_valid);
    for (const auto & ray : frame->rays) {
      builder.addRay(ray);
    }

    const auto state = builder.finalize().sectors[0].state;
    EXPECT_EQ(state, expect_clear ? SectorState::kObservedClear : SectorState::kUnknown);
  }
}
