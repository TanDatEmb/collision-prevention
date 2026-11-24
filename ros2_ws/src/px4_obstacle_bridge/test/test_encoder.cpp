#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "obstacle_sector_map/types.hpp"
#include "px4_msgs/msg/obstacle_distance.hpp"
#include "px4_obstacle_bridge/encoder.hpp"

using obstacle_sector_map::SectorMap;
using obstacle_sector_map::SectorMapConfig;
using obstacle_sector_map::SectorState;
using px4_obstacle_bridge::encodeObstacleDistance;

namespace
{

constexpr SectorMapConfig kConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 40.0F, 0.0F};

SectorMap makeMap()
{
  SectorMap map{};
  map.sample_timestamp_ns = 1'234'567U;
  map.packet_continuity_valid = true;
  return map;
}

void setSector(SectorMap & map, std::size_t index, SectorState state, float distance_m)
{
  map.sectors.at(index).state = state;
  map.sectors.at(index).distance_m = distance_m;

  if (state == SectorState::kObservedClear) {
    map.sectors.at(index).free_distance_m = obstacle_sector_map::kMaxDistanceM;
  }
}

}  // namespace

TEST(ObstacleDistanceEncoderTest, EncodesPinnedMetadataAndValues)
{
  auto map = makeMap();
  setSector(map, 0U, SectorState::kTooClose, 0.0F);
  setSector(map, 1U, SectorState::kObservedClear, std::numeric_limits<float>::quiet_NaN());
  setSector(map, 2U, SectorState::kOccupied, 12.349F);

  const auto encoded = encodeObstacleDistance(map, kConfig);

  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(encoded->timestamp, 1'234U);
  EXPECT_EQ(encoded->frame, px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD);
  EXPECT_EQ(encoded->sensor_type, px4_msgs::msg::ObstacleDistance::MAV_DISTANCE_SENSOR_LASER);
  EXPECT_FLOAT_EQ(encoded->increment, 5.0F);
  EXPECT_FLOAT_EQ(encoded->angle_offset, 0.0F);
  EXPECT_EQ(encoded->min_distance, 10U);
  EXPECT_EQ(encoded->max_distance, 4'000U);
  EXPECT_EQ(encoded->distances[0], 0U);
  EXPECT_EQ(encoded->distances[71], 4'001U);
  EXPECT_EQ(encoded->distances[70], 1'234U);
  EXPECT_EQ(encoded->distances[1], std::numeric_limits<uint16_t>::max());
}

TEST(ObstacleDistanceEncoderTest, KeepsForwardAtIndexZero)
{
  auto map = makeMap();
  setSector(map, 0U, SectorState::kOccupied, 5.0F);

  const auto encoded = encodeObstacleDistance(map, kConfig);

  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(encoded->distances[0], 500U);
}

TEST(ObstacleDistanceEncoderTest, MapsFluLeftToFrdCounterIndex)
{
  auto map = makeMap();
  setSector(map, 1U, SectorState::kOccupied, 6.0F);

  const auto encoded = encodeObstacleDistance(map, kConfig);

  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(encoded->distances[71], 600U);
  EXPECT_EQ(encoded->distances[1], std::numeric_limits<uint16_t>::max());
}

TEST(ObstacleDistanceEncoderTest, MapsFluRightToFrdPositiveIndex)
{
  auto map = makeMap();
  setSector(map, 71U, SectorState::kOccupied, 7.0F);

  const auto encoded = encodeObstacleDistance(map, kConfig);

  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(encoded->distances[1], 700U);
  EXPECT_EQ(encoded->distances[71], std::numeric_limits<uint16_t>::max());
}

TEST(ObstacleDistanceEncoderTest, FloorsOccupiedCentimetresConservatively)
{
  auto map = makeMap();
  setSector(map, 0U, SectorState::kOccupied, 12.349F);

  const auto encoded = encodeObstacleDistance(map, kConfig);

  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(encoded->distances[0], 1'234U);
}

TEST(ObstacleDistanceEncoderTest, RejectsOccupiedDistanceOutsideQualifiedRange)
{
  auto map = makeMap();
  setSector(map, 0U, SectorState::kOccupied, 1.0e20F);

  EXPECT_FALSE(encodeObstacleDistance(map, kConfig).has_value());
}

TEST(ObstacleDistanceEncoderTest, RejectsClearWithoutContinuityOrFullFreeDistance)
{
  auto map = makeMap();
  setSector(map, 0U, SectorState::kObservedClear, std::numeric_limits<float>::quiet_NaN());

  map.packet_continuity_valid = false;
  EXPECT_FALSE(encodeObstacleDistance(map, kConfig).has_value());

  map.packet_continuity_valid = true;
  map.sectors[0].free_distance_m = std::numeric_limits<float>::quiet_NaN();  // nothing certified
  EXPECT_FALSE(encodeObstacleDistance(map, kConfig).has_value());

  map.sectors[0].free_distance_m = 11.4F;  // certified, but short of the qualified range
  EXPECT_FALSE(encodeObstacleDistance(map, kConfig).has_value());

  map.sectors[0].free_distance_m = obstacle_sector_map::kMaxDistanceM;
  EXPECT_TRUE(encodeObstacleDistance(map, kConfig).has_value());
}

TEST(ObstacleDistanceEncoderTest, RejectsZeroOrSubMicrosecondTimestamp)
{
  auto map = makeMap();
  map.sample_timestamp_ns = 0U;
  EXPECT_FALSE(encodeObstacleDistance(map, kConfig).has_value());

  map.sample_timestamp_ns = 999U;
  EXPECT_FALSE(encodeObstacleDistance(map, kConfig).has_value());
}

TEST(ObstacleDistanceEncoderTest, RejectsInvalidConfigAndInvalidOccupiedDistance)
{
  auto map = makeMap();
  setSector(map, 0U, SectorState::kOccupied, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(encodeObstacleDistance(map, kConfig).has_value());

  setSector(map, 0U, SectorState::kOccupied, 0.0F);
  EXPECT_FALSE(encodeObstacleDistance(map, kConfig).has_value());

  setSector(map, 0U, SectorState::kOccupied, 1.0F);
  const SectorMapConfig invalid_config{1.0F, 1.0F, 0.5F, 0U, 2U, 40.0F, 0.0F};
  EXPECT_FALSE(encodeObstacleDistance(map, invalid_config).has_value());
}

TEST(ObstacleDistanceEncoderTest, RangeFieldsAreFixedByTheInterface)
{
  const auto encoded = encodeObstacleDistance(makeMap(), kConfig);

  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(encoded->min_distance, 10U);
  EXPECT_EQ(encoded->max_distance, 4'000U);
}
