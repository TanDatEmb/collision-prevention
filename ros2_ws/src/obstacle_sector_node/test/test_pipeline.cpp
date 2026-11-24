#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <cmath>

#include "livox_ray_source/leveling_ray_source.hpp"
#include "livox_ray_source/pose_history.hpp"
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
constexpr SectorMapConfig kConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 40.0F, 0.0F};
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
    SectorMapPipeline(replay({}), SectorMapConfig{1.0F, 1.0F, 0.5F, 0U, 2U, 40.0F, 0.0F}), std::invalid_argument);
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

TEST(SectorMapPipelineTest, ConfirmedReturnsInsideTheRangeEncodeTheirDistanceInCentimetres)
{
  // Two returns inside one window confirm the obstacle (n_obstacle 2); a lone return would be noise.
  SectorMapPipeline pipeline{
    replay({makeReturn(5'000'000U, 12.34F), makeReturn(5'000'100U, 12.35F)}), kConfig};

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

// Leveling through the whole pipeline (issue #137): the sensor is mounted 10 degrees down. A wall return straight
// ahead of the sensor is 5 m away horizontally when the vehicle is pitched 10 degrees nose up (the boresight is
// then level, the rays lie in both halves of the band). With the vehicle level the same rays all point 10 degrees
// down: no ray lies in the upper half, so the sector is unknown (coverage rule, SYS-CP-029) instead of showing a
// wall at 4.92 m. A pitch applied with the wrong sign would also leave the sector unknown, not at 5 m. Both rays
// carry the reference time, as in the Gazebo cloud, so no gyro is needed (ADR 0023).
namespace
{

class PitchedPoses : public livox_ray_source::PoseHistory
{
public:
  explicit PitchedPoses(double pitch_up_rad)
  : PoseHistory(livox_ray_source::PoseHistoryConfig{8U, 1'000'000'000U})
  {
    for (const uint64_t time_ns : {6'000'000U, 7'000'000U}) {
      livox_ray_source::PoseSample sample;
      sample.timestamp_sample_ns = time_ns;
      sample.q_wxyz = {std::cos(0.5 * pitch_up_rad), 0.0, std::sin(0.5 * pitch_up_rad), 0.0};
      sample.position_ned_m = {0.0, 0.0, 0.0};
      push(sample);
    }
  }
};

uint16_t wallDistanceCm(double vehicle_pitch_up_rad)
{
  const double mount_pitch_down_rad = 10.0 * 3.14159265358979323846 / 180.0;
  const PitchedPoses poses{vehicle_pitch_up_rad};
  livox_ray_source::LevelingConfig leveling{0.0, mount_pitch_down_rad, 0.0, {0.0, 0.0, 0.0}};
  SectorMapPipeline pipeline{
    std::make_unique<livox_ray_source::LevelingRaySource>(
      replay({makeReturn(6'500'000U, 5.0F), makeReturn(6'500'000U, 5.0F)}), poses, leveling),
    kConfig};
  const auto result = pipeline.step();
  EXPECT_EQ(result.status, StepStatus::kMessage);
  return result.message.has_value() ? result.message->distances[0] : 0U;
}

}  // namespace

TEST(SectorMapPipelineTest, LevelingPutsTheBoresightLevelWhenTheVehiclePitchesUpByTheMountAngle)
{
  const double ten_deg = 10.0 * 3.14159265358979323846 / 180.0;
  EXPECT_EQ(wallDistanceCm(ten_deg), 500U);
  EXPECT_EQ(wallDistanceCm(0.0), kUnknown);
}
