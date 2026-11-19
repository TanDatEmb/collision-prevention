#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "livox_ray_source/replay_ray_source.hpp"
#include "obstacle_sector_node/pipeline.hpp"

using livox_ray_source::ReplayRaySource;
using livox_ray_source::StreamSample;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;
using obstacle_sector_map::SectorMapConfig;
using obstacle_sector_node::SectorMapPipeline;
using obstacle_sector_node::StepStatus;

namespace
{

// Test fixture values, not approved platform values.
constexpr SectorMapConfig kConfig{-0.2F, 0.4F, 2U};
constexpr uint64_t kWindowNs = 1'000'000U;
constexpr uint16_t kUnknown = std::numeric_limits<uint16_t>::max();

StreamSample makeNoReturn(uint64_t timestamp_ns, bool brk = false)
{
  return StreamSample{
    RaySample{
      timestamp_ns, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, false,
      RayReturnKind::kQualifiedNoReturn},
    brk};
}

StreamSample makeReturn(uint64_t timestamp_ns, float range_m)
{
  return StreamSample{
    RaySample{timestamp_ns, range_m, 0.0F, 0.0F, false, RayReturnKind::kReturn}, false};
}

std::unique_ptr<livox_ray_source::RaySource> replay(std::vector<StreamSample> samples)
{
  return std::make_unique<ReplayRaySource>(std::move(samples), kWindowNs);
}

}  // namespace

TEST(SectorMapPipelineTest, RejectsNullSourceAndInvalidConfig)
{
  EXPECT_THROW(SectorMapPipeline(nullptr, kConfig), std::invalid_argument);
  EXPECT_THROW(
    SectorMapPipeline(replay({}), SectorMapConfig{0.4F, -0.2F, 2U}), std::invalid_argument);
}

TEST(SectorMapPipelineTest, ContinuousNoReturnsEncodeObservedClearWithTheEarliestTimestamp)
{
  SectorMapPipeline pipeline{replay({makeNoReturn(6'500'000U), makeNoReturn(7'000'000U)}), kConfig};

  const auto result = pipeline.step();

  ASSERT_EQ(result.status, StepStatus::kMessage);
  ASSERT_TRUE(result.message.has_value());
  EXPECT_EQ(result.message->timestamp, 6'500U);
  EXPECT_EQ(result.message->distances[0], 4'001U);
  EXPECT_EQ(result.message->distances[1], kUnknown);
}

TEST(SectorMapPipelineTest, LostPacketsGiveUnknownNotClear)
{
  SectorMapPipeline pipeline{
    replay({makeNoReturn(5'000'000U), makeNoReturn(5'100'000U, true)}), kConfig};

  const auto result = pipeline.step();

  ASSERT_EQ(result.status, StepStatus::kMessage);
  EXPECT_EQ(result.message->distances[0], kUnknown);
}

TEST(SectorMapPipelineTest, ReturnInsideTheRangeEncodesItsDistanceInCentimetres)
{
  SectorMapPipeline pipeline{replay({makeReturn(5'000'000U, 12.34F)}), kConfig};

  const auto result = pipeline.step();

  ASSERT_EQ(result.status, StepStatus::kMessage);
  EXPECT_EQ(result.message->distances[0], 1'234U);
}

TEST(SectorMapPipelineTest, EncodeFailureIsSilentAndCounted)
{
  // A frame whose rays have no timestamp is rejected by the map, so the map has no sample time.
  SectorMapPipeline pipeline{replay({makeNoReturn(0U), makeNoReturn(0U)}), kConfig};

  const auto result = pipeline.step();

  EXPECT_EQ(result.status, StepStatus::kEncodeFailed);
  EXPECT_FALSE(result.message.has_value());
  EXPECT_EQ(pipeline.stats().frames, 1U);
  EXPECT_EQ(pipeline.stats().encode_failures, 1U);
  EXPECT_EQ(pipeline.stats().messages, 0U);
}

TEST(SectorMapPipelineTest, ReportsTheEndOfTheSourceAndKeepsTheCounters)
{
  SectorMapPipeline pipeline{
    replay({makeNoReturn(5'000'000U), makeNoReturn(5'100'000U), makeNoReturn(9'000'000U)}),
    kConfig};

  EXPECT_EQ(pipeline.step().status, StepStatus::kMessage);
  EXPECT_EQ(pipeline.step().status, StepStatus::kMessage);
  EXPECT_EQ(pipeline.step().status, StepStatus::kSourceEnded);
  EXPECT_EQ(pipeline.stats().frames, 2U);
  EXPECT_EQ(pipeline.stats().messages, 2U);
}
