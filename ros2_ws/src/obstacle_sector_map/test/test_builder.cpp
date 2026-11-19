#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "obstacle_sector_map/builder.hpp"
#include "obstacle_sector_map/types.hpp"

using obstacle_sector_map::kSectorCount;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;
using obstacle_sector_map::SectorMapBuilder;
using obstacle_sector_map::SectorMapConfig;
using obstacle_sector_map::SectorState;

static_assert(kSectorCount == 72);

namespace
{

constexpr float kDegreesToRadians = 3.14159265358979323846F / 180.0F;
constexpr SectorMapConfig kConfig{0.1F, 40.0F, -0.2F, 0.4F, 2U, 1.0e-6F};
constexpr SectorMapConfig kZeroBoundaryEpsilonConfig{0.1F, 40.0F, -0.2F, 0.4F, 2U, 0.0F};

RaySample makeRay(
  RayReturnKind return_kind, float range_m, float azimuth_rad = 0.0F,
  uint64_t timestamp_ns = 1'000U)
{
  return RaySample{timestamp_ns, range_m, azimuth_rad, 0.0F, false, return_kind};
}

RaySample makeNoReturn(float azimuth_rad = 0.0F, uint64_t timestamp_ns = 1'000U)
{
  return makeRay(
    RayReturnKind::kQualifiedNoReturn, std::numeric_limits<float>::quiet_NaN(), azimuth_rad,
    timestamp_ns);
}

RaySample makeReturn(float range_m, float azimuth_rad = 0.0F, uint64_t timestamp_ns = 1'000U)
{
  return makeRay(RayReturnKind::kReturn, range_m, azimuth_rad, timestamp_ns);
}

}  // namespace

TEST(SectorMapConfigTest, AcceptsExplicitQualifiedGeometry)
{
  const SectorMapConfig config{0.1F, 40.0F, -0.2F, 0.4F, 8U, 1.0e-6F};
  EXPECT_TRUE(config.isValid());
}

TEST(SectorMapConfigTest, RejectsUnsafeValues)
{
  EXPECT_FALSE((SectorMapConfig{0.0F, 40.0F, -0.2F, 0.4F, 8U, 1.0e-6F}).isValid());
  EXPECT_FALSE((SectorMapConfig{0.1F, 0.1F, -0.2F, 0.4F, 8U, 1.0e-6F}).isValid());
  EXPECT_FALSE((SectorMapConfig{0.1F, 40.0F, 0.4F, -0.2F, 8U, 1.0e-6F}).isValid());
  EXPECT_FALSE((SectorMapConfig{0.1F, 40.0F, -1.6F, 0.4F, 8U, 1.0e-6F}).isValid());
  EXPECT_FALSE((SectorMapConfig{0.1F, 40.0F, -0.2F, 1.6F, 8U, 1.0e-6F}).isValid());
  EXPECT_FALSE((SectorMapConfig{0.1F, 40.0F, -0.2F, 0.4F, 0U, 1.0e-6F}).isValid());
  EXPECT_FALSE((SectorMapConfig{0.1F, 40.0F, -0.2F, 0.4F, 8U, -1.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{0.1F, 40.0F, -0.2F, 0.4F, 8U, 0.01F}).isValid());
}

TEST(SectorMapBuilderTest, RejectsInvalidConfiguration)
{
  EXPECT_THROW(
    SectorMapBuilder((SectorMapConfig{0.0F, 40.0F, -0.2F, 0.4F, 2U, 1.0e-6F})),
    std::invalid_argument);
}

TEST(SectorMapBuilderTest, EmptyWindowIsUnknown)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);

  const auto map = builder.finalize();

  EXPECT_EQ(map.sample_timestamp_ns, 0U);
  EXPECT_EQ(map.accepted_ray_count, 0U);
  for (const auto & sector : map.sectors) {
    EXPECT_EQ(sector.state, SectorState::kUnknown);
    EXPECT_TRUE(std::isnan(sector.distance_m));
  }
}

TEST(SectorMapBuilderTest, QualifiedNoReturnCoverageProducesObservedClear)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeNoReturn(0.0F, 2'000U));
  builder.addRay(makeNoReturn(0.0F, 1'000U));

  const auto map = builder.finalize();

  EXPECT_EQ(map.sectors[0].state, SectorState::kObservedClear);
  EXPECT_EQ(map.sectors[0].contributing_ray_count, 2U);
  EXPECT_EQ(map.sectors[0].clear_ray_count, 2U);
  EXPECT_EQ(map.sample_timestamp_ns, 1'000U);
}

TEST(SectorMapBuilderTest, MissingPacketContinuityPreventsClear)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(false);
  builder.addRay(makeNoReturn());
  builder.addRay(makeNoReturn());

  const auto map = builder.finalize();

  EXPECT_EQ(map.sectors[0].state, SectorState::kUnknown);
  EXPECT_EQ(map.sectors[0].contributing_ray_count, 2U);
}

TEST(SectorMapBuilderTest, PositiveObstacleSurvivesMissingPacketContinuity)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(false);
  builder.addRay(makeReturn(12.0F));

  const auto map = builder.finalize();

  EXPECT_EQ(map.sectors[0].state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(map.sectors[0].distance_m, 12.0F);
  EXPECT_EQ(map.sectors[1].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, ChoosesMinimumCredibleRange)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(20.0F));
  builder.addRay(makeReturn(7.5F));

  EXPECT_FLOAT_EQ(builder.finalize().sectors[0].distance_m, 7.5F);
}

TEST(SectorMapBuilderTest, RangeBelowMinimumIsTooClose)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(0.05F));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kTooClose);
  EXPECT_FLOAT_EQ(sector.distance_m, 0.0F);
}

TEST(SectorMapBuilderTest, UsesHorizontalProjectedRangeForClassification)
{
  constexpr float kSixtyDegreesRad{1.0471975511965976F};
  const SectorMapConfig wide_elevation_config{0.1F, 40.0F, -1.2F, 1.2F, 1U, 1.0e-6F};

  SectorMapBuilder occupied_builder{wide_elevation_config};
  occupied_builder.reset(true);
  auto elevated_return = makeReturn(10.0F);
  elevated_return.elevation_rad = kSixtyDegreesRad;
  occupied_builder.addRay(elevated_return);

  const auto occupied = occupied_builder.finalize().sectors[0];
  ASSERT_EQ(occupied.state, SectorState::kOccupied);
  EXPECT_NEAR(occupied.distance_m, 5.0F, 1.0e-5F);

  SectorMapBuilder too_close_builder{wide_elevation_config};
  too_close_builder.reset(true);
  auto elevated_near_return = makeReturn(0.15F);
  elevated_near_return.elevation_rad = kSixtyDegreesRad;
  too_close_builder.addRay(elevated_near_return);

  EXPECT_EQ(too_close_builder.finalize().sectors[0].state, SectorState::kTooClose);
}

TEST(SectorMapBuilderTest, ReturnBeyondQualifiedRangeIsNotClearEvidence)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(50.0F));
  builder.addRay(makeReturn(60.0F));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kUnknown);
  EXPECT_EQ(sector.contributing_ray_count, 2U);
  EXPECT_EQ(sector.clear_ray_count, 0U);
}

TEST(SectorMapBuilderTest, OnlyQualifiedNoReturnCountsTowardClearCoverage)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeNoReturn());
  builder.addRay(makeReturn(50.0F));
  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kUnknown);

  builder.addRay(makeNoReturn());
  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kObservedClear);
  EXPECT_EQ(sector.clear_ray_count, 2U);
  EXPECT_EQ(sector.contributing_ray_count, 3U);
}

TEST(SectorMapBuilderTest, RejectsNonFiniteInvalidMaskedAndVerticalOutliers)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);

  auto non_finite = makeNoReturn();
  non_finite.azimuth_rad = std::numeric_limits<float>::infinity();
  builder.addRay(non_finite);

  auto invalid_kind = makeNoReturn();
  invalid_kind.return_kind = RayReturnKind::kInvalid;
  builder.addRay(invalid_kind);

  auto masked = makeNoReturn();
  masked.masked = true;
  builder.addRay(masked);

  auto vertical_outlier = makeNoReturn();
  vertical_outlier.elevation_rad = 0.5F;
  builder.addRay(vertical_outlier);

  const auto map = builder.finalize();
  EXPECT_EQ(map.accepted_ray_count, 0U);
  EXPECT_EQ(map.rejected_ray_count, 4U);
  EXPECT_EQ(map.sectors[0].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, UsesEarliestAcceptedRayTimestamp)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeNoReturn(0.0F, 9'000U));
  builder.addRay(makeNoReturn(5.0F * kDegreesToRadians, 3'000U));
  builder.addRay(makeNoReturn(-5.0F * kDegreesToRadians, 6'000U));

  EXPECT_EQ(builder.finalize().sample_timestamp_ns, 3'000U);
}

TEST(SectorMapBuilderTest, ExactPositiveBoundaryUpdatesBothAdjacentSectors)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(10.0F, 2.5F * kDegreesToRadians));

  const auto map = builder.finalize();
  EXPECT_EQ(map.sectors[0].state, SectorState::kOccupied);
  EXPECT_EQ(map.sectors[1].state, SectorState::kOccupied);
}

TEST(SectorMapBuilderTest, ExactNegativeBoundaryUpdatesBothAdjacentSectors)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(10.0F, -2.5F * kDegreesToRadians));

  const auto map = builder.finalize();
  EXPECT_EQ(map.sectors[0].state, SectorState::kOccupied);
  EXPECT_EQ(map.sectors[71].state, SectorState::kOccupied);
}

TEST(SectorMapBuilderTest, ZeroEpsilonAllPositiveFloatBoundariesUpdateBothAdjacentSectors)
{
  for (std::size_t lower_index = 0U; lower_index < kSectorCount; ++lower_index) {
    SCOPED_TRACE(lower_index);
    SectorMapBuilder builder{kZeroBoundaryEpsilonConfig};
    builder.reset(true);
    const float boundary_deg = 2.5F + 5.0F * static_cast<float>(lower_index);
    builder.addRay(makeReturn(10.0F, boundary_deg * kDegreesToRadians));

    const auto map = builder.finalize();
    EXPECT_EQ(map.sectors[lower_index].state, SectorState::kOccupied);
    EXPECT_EQ(map.sectors[(lower_index + 1U) % kSectorCount].state, SectorState::kOccupied);
  }
}

TEST(SectorMapBuilderTest, ZeroEpsilonAllNegativeFloatBoundariesUpdateBothAdjacentSectors)
{
  for (std::size_t boundary_number = 0U; boundary_number < kSectorCount; ++boundary_number) {
    SCOPED_TRACE(boundary_number);
    SectorMapBuilder builder{kZeroBoundaryEpsilonConfig};
    builder.reset(true);
    const float boundary_deg = -(2.5F + 5.0F * static_cast<float>(boundary_number));
    builder.addRay(makeReturn(10.0F, boundary_deg * kDegreesToRadians));

    const std::size_t lower_index = kSectorCount - 1U - boundary_number;
    const auto map = builder.finalize();
    EXPECT_EQ(map.sectors[lower_index].state, SectorState::kOccupied);
    EXPECT_EQ(map.sectors[(lower_index + 1U) % kSectorCount].state, SectorState::kOccupied);
  }
}

TEST(SectorMapBuilderTest, ZeroEpsilonJustOffBoundaryDoesNotDuplicateObstacleEvidence)
{
  constexpr float kBoundaryRad = 27.5F * kDegreesToRadians;
  constexpr float kOutsideNumericalToleranceRad = 1.0e-4F;

  SectorMapBuilder lower_builder{kZeroBoundaryEpsilonConfig};
  lower_builder.reset(true);
  lower_builder.addRay(makeReturn(10.0F, kBoundaryRad - kOutsideNumericalToleranceRad));
  const auto lower_map = lower_builder.finalize();
  EXPECT_EQ(lower_map.sectors[5].state, SectorState::kOccupied);
  EXPECT_EQ(lower_map.sectors[6].state, SectorState::kUnknown);

  SectorMapBuilder upper_builder{kZeroBoundaryEpsilonConfig};
  upper_builder.reset(true);
  upper_builder.addRay(makeReturn(10.0F, kBoundaryRad + kOutsideNumericalToleranceRad));
  const auto upper_map = upper_builder.finalize();
  EXPECT_EQ(upper_map.sectors[5].state, SectorState::kUnknown);
  EXPECT_EQ(upper_map.sectors[6].state, SectorState::kOccupied);
}

TEST(SectorMapBuilderTest, ZeroEpsilonBoundaryNoReturnDoesNotDuplicateClearCoverage)
{
  SectorMapBuilder builder{kZeroBoundaryEpsilonConfig};
  builder.reset(true);
  builder.addRay(makeNoReturn(2.5F * kDegreesToRadians));
  builder.addRay(makeNoReturn(2.5F * kDegreesToRadians));

  const auto map = builder.finalize();
  const bool sector_zero_clear = map.sectors[0].state == SectorState::kObservedClear;
  const bool sector_one_clear = map.sectors[1].state == SectorState::kObservedClear;
  EXPECT_NE(sector_zero_clear, sector_one_clear);
  EXPECT_EQ(map.sectors[0].contributing_ray_count + map.sectors[1].contributing_ray_count, 2U);
}

TEST(SectorMapBuilderTest, WrapBoundaryNeverIndexesOutsideSeventyTwoSectors)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(11.0F, 3.14159265358979323846F));
  builder.addRay(makeReturn(9.0F, -3.14159265358979323846F));

  const auto map = builder.finalize();
  EXPECT_EQ(map.sectors[36].state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(map.sectors[36].distance_m, 9.0F);
  EXPECT_EQ(map.accepted_ray_count, 2U);
}

TEST(SectorMapBuilderTest, RejectsRayWithZeroTimestamp)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(5.0F, 0.0F, 0U));
  builder.addRay(makeNoReturn(0.0F, 0U));

  const auto map = builder.finalize();
  EXPECT_EQ(map.accepted_ray_count, 0U);
  EXPECT_EQ(map.rejected_ray_count, 2U);
  EXPECT_EQ(map.sample_timestamp_ns, 0U);
  EXPECT_EQ(map.sectors[0].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, RejectsReturnWithInvalidRange)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(std::numeric_limits<float>::quiet_NaN()));
  builder.addRay(makeReturn(std::numeric_limits<float>::infinity()));
  builder.addRay(makeReturn(0.0F));
  builder.addRay(makeReturn(-3.0F));

  const auto map = builder.finalize();
  EXPECT_EQ(map.accepted_ray_count, 0U);
  EXPECT_EQ(map.rejected_ray_count, 4U);
  EXPECT_EQ(map.sectors[0].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, ElevationBoundsAreInclusive)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);

  auto at_minimum = makeNoReturn();
  at_minimum.elevation_rad = kConfig.min_elevation_rad;
  builder.addRay(at_minimum);

  auto at_maximum = makeNoReturn();
  at_maximum.elevation_rad = kConfig.max_elevation_rad;
  builder.addRay(at_maximum);

  auto below_minimum = makeNoReturn();
  below_minimum.elevation_rad = std::nextafter(kConfig.min_elevation_rad, -1.0F);
  builder.addRay(below_minimum);

  auto above_maximum = makeNoReturn();
  above_maximum.elevation_rad = std::nextafter(kConfig.max_elevation_rad, 1.0F);
  builder.addRay(above_maximum);

  const auto map = builder.finalize();
  EXPECT_EQ(map.accepted_ray_count, 2U);
  EXPECT_EQ(map.rejected_ray_count, 2U);
  EXPECT_EQ(map.sectors[0].state, SectorState::kObservedClear);
}

TEST(SectorMapBuilderTest, ResetClearsPreviousWindow)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(5.0F, 0.0F, 7'000U));
  builder.addRay(makeNoReturn(0.0F, 7'000U));
  builder.addRay(makeNoReturn(0.0F, 0U));

  builder.reset(false);
  const auto map = builder.finalize();

  EXPECT_FALSE(map.packet_continuity_valid);
  EXPECT_EQ(map.sample_timestamp_ns, 0U);
  EXPECT_EQ(map.accepted_ray_count, 0U);
  EXPECT_EQ(map.rejected_ray_count, 0U);
  for (const auto & sector : map.sectors) {
    EXPECT_EQ(sector.state, SectorState::kUnknown);
    EXPECT_EQ(sector.contributing_ray_count, 0U);
  }
}

TEST(SectorMapBuilderTest, TooCloseOverridesOccupiedInSameSector)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(8.0F));
  builder.addRay(makeReturn(0.05F));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kTooClose);
  EXPECT_FLOAT_EQ(sector.distance_m, 0.0F);
}
