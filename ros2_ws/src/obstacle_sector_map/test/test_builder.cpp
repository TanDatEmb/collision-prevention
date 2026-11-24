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
// Test values only: h_up 1 m, h_down 1 m, dr 0.5 m, n_obstacle 2, n_free 2, certified range 40 m (the cap), no
// attitude allowance. Not platform values.
constexpr SectorMapConfig kConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 40.0F, 0.0F};
// One return confirms an obstacle and one ray covers a half.
constexpr SectorMapConfig kSingle{1.0F, 1.0F, 0.5F, 1U, 1U, 40.0F, 0.0F};
// The same as kConfig with a certified range of 12 m (ADR 0024).
constexpr SectorMapConfig kResolved{1.0F, 1.0F, 0.5F, 2U, 2U, 12.0F, 0.0F};

RaySample makeRay(
  RayReturnKind return_kind, float range_m, float azimuth_rad = 0.0F, float elevation_rad = 0.0F,
  uint64_t timestamp_ns = 1'000U)
{
  return RaySample{timestamp_ns, range_m, azimuth_rad, elevation_rad, false, return_kind};
}

RaySample makeNoReturn(
  float azimuth_rad = 0.0F, uint64_t timestamp_ns = 1'000U, float elevation_rad = 0.0F)
{
  return makeRay(
    RayReturnKind::kQualifiedNoReturn, std::numeric_limits<float>::quiet_NaN(), azimuth_rad,
    elevation_rad, timestamp_ns);
}

RaySample makeReturn(
  float range_m, float azimuth_rad = 0.0F, uint64_t timestamp_ns = 1'000U, float elevation_rad = 0.0F)
{
  return makeRay(RayReturnKind::kReturn, range_m, azimuth_rad, elevation_rad, timestamp_ns);
}

// Two horizon no-returns: enough to certify the full qualified range with n_free 2.
void addOpenSky(SectorMapBuilder & builder, float azimuth_rad = 0.0F)
{
  builder.addRay(makeNoReturn(azimuth_rad));
  builder.addRay(makeNoReturn(azimuth_rad));
}

}  // namespace

TEST(SectorMapConfigTest, AcceptsExplicitValues)
{
  EXPECT_TRUE(kConfig.isValid());
}

TEST(SectorMapConfigTest, RejectsUnsafeValues)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE((SectorMapConfig{0.0F, 1.0F, 0.5F, 2U, 2U, 40.0F, 0.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, -1.0F, 0.5F, 2U, 2U, 40.0F, 0.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{nan, 1.0F, 0.5F, 2U, 2U, 40.0F, 0.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, nan, 0.5F, 2U, 2U, 40.0F, 0.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.0F, 2U, 2U, 40.0F, 0.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, nan, 2U, 2U, 40.0F, 0.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 0U, 2U, 40.0F, 0.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 0U, 40.0F, 0.0F}).isValid());
}

TEST(SectorMapBuilderTest, RejectsInvalidConfiguration)
{
  EXPECT_THROW(
    SectorMapBuilder((SectorMapConfig{1.0F, 1.0F, 0.5F, 0U, 2U, 40.0F, 0.0F})), std::invalid_argument);
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

TEST(SectorMapBuilderTest, OpenSkyHorizonRaysCertifyTheQualifiedRange)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeNoReturn(0.0F, 2'000U));
  builder.addRay(makeNoReturn(0.0F, 1'000U));

  const auto map = builder.finalize();

  EXPECT_EQ(map.sectors[0].state, SectorState::kObservedClear);
  EXPECT_EQ(map.sectors[0].contributing_ray_count, 2U);
  EXPECT_EQ(map.sectors[0].clear_ray_count, 2U);
  EXPECT_FLOAT_EQ(map.sectors[0].free_distance_m, obstacle_sector_map::kMaxDistanceM);
  EXPECT_EQ(map.sample_timestamp_ns, 1'000U);
}

TEST(SectorMapBuilderTest, MissingPacketContinuityPreventsClear)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(false);
  addOpenSky(builder);

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kUnknown);
  EXPECT_TRUE(std::isnan(sector.free_distance_m));
}

TEST(SectorMapBuilderTest, ConfirmedObstacleSurvivesMissingPacketContinuity)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(false);
  builder.addRay(makeReturn(5.0F));
  builder.addRay(makeReturn(5.2F));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(sector.distance_m, 5.0F);
}

TEST(SectorMapBuilderTest, LoneNoiseReturnDoesNotBrakeInOpenSky)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(3.0F));
  addOpenSky(builder);

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kObservedClear);
  EXPECT_TRUE(std::isnan(sector.obstacle_distance_m));
}

TEST(SectorMapBuilderTest, ObstacleDistanceIsTheNearEdgeOfTheWindow)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(5.3F));
  builder.addRay(makeReturn(5.0F));
  addOpenSky(builder);

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(sector.distance_m, 5.0F);
  EXPECT_FLOAT_EQ(sector.obstacle_distance_m, 5.0F);
}

TEST(SectorMapBuilderTest, ReturnsFartherApartThanTheWindowDoNotConfirm)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(5.0F));
  builder.addRay(makeReturn(6.0F));
  addOpenSky(builder);

  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kObservedClear);
}

TEST(SectorMapBuilderTest, NearestConfirmedWindowWinsOverAnIsolatedNearerReturn)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(4.0F));
  builder.addRay(makeReturn(9.2F));
  builder.addRay(makeReturn(9.0F));
  addOpenSky(builder);

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(sector.distance_m, 9.0F);
}

TEST(SectorMapBuilderTest, ThinPoleIsConfirmedOnlyByEnoughReturns)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(12.0F));
  builder.addRay(makeReturn(12.1F));
  addOpenSky(builder);

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(sector.distance_m, 12.0F);

  SectorMapBuilder lone{kConfig};
  lone.reset(true);
  lone.addRay(makeReturn(12.0F));
  addOpenSky(lone);
  EXPECT_EQ(lone.finalize().sectors[0].state, SectorState::kObservedClear);
}

TEST(SectorMapBuilderTest, ReturnBelowMinimumIsTooClose)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(0.05F));
  addOpenSky(builder);

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kTooClose);
  EXPECT_FLOAT_EQ(sector.distance_m, 0.0F);
}

TEST(SectorMapBuilderTest, TooCloseOverridesAConfirmedObstacleInTheSameSector)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(8.0F));
  builder.addRay(makeReturn(8.1F));
  builder.addRay(makeReturn(0.05F));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kTooClose);
  EXPECT_FLOAT_EQ(sector.distance_m, 0.0F);
}

TEST(SectorMapBuilderTest, UsesHorizontalProjectedRangeForTheObstacleDistance)
{
  // 10 m at +5 degrees is 0.87 m high (inside the 1 m band) and 9.96 m away horizontally.
  const float elevation = 5.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, elevation));
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, elevation));
  // The same wall seen in the lower half too (both halves are seen, issue #148).
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, -elevation));
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, -elevation));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_NEAR(sector.distance_m, 10.0F * std::cos(elevation), 1.0e-4F);
}

TEST(SectorMapBuilderTest, ReturnAboveTheBandIsNotAnObstacle)
{
  // 10 m at +10 degrees is 1.74 m high: above h_up, so it is no obstacle evidence.
  const float elevation = 10.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, elevation));
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, elevation));
  addOpenSky(builder);

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kObservedClear);
  EXPECT_TRUE(std::isnan(sector.obstacle_distance_m));
}

TEST(SectorMapBuilderTest, ACoveredSectorIsCertifiedToTheCertifiedRange)
{
  // ADR 0024 decision 1: rays no longer certify a distance of their own. No-returns at +-5 degrees cover both
  // halves; the sector is certified to certified_range_m (12 m), and to the cap (clear) when that is 40 m.
  const float five = 5.0F * kDegreesToRadians;
  for (const bool resolved : {true, false}) {
    SectorMapBuilder builder{resolved ? kResolved : kConfig};
    builder.reset(true);
    builder.addRay(makeNoReturn(0.0F, 1'000U, five));
    builder.addRay(makeNoReturn(0.0F, 1'000U, five));
    builder.addRay(makeNoReturn(0.0F, 1'000U, -five));
    builder.addRay(makeNoReturn(0.0F, 1'000U, -five));
    const auto sector = builder.finalize().sectors[0];
    if (resolved) {
      EXPECT_EQ(sector.state, SectorState::kOccupied);
      EXPECT_FLOAT_EQ(sector.distance_m, 12.0F);
      EXPECT_FLOAT_EQ(sector.free_distance_m, 12.0F);
    } else {
      EXPECT_EQ(sector.state, SectorState::kObservedClear);
    }
  }
}

TEST(SectorMapBuilderTest, OutOfBandReturnsDoNotShortenTheCertifiedRange)
{
  // Ground returns 5 m below the band (-30 degrees, 10 m) and a slope above it only cover their halves: the
  // per-ray band exit (1.73 m before ADR 0024) no longer limits the sector.
  const float down = -30.0F * kDegreesToRadians;
  const float up = 30.0F * kDegreesToRadians;
  SectorMapBuilder builder{kResolved};
  builder.reset(true);
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, down));
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, down));
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, up));
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, up));
  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(sector.distance_m, 12.0F);
  EXPECT_TRUE(std::isnan(sector.obstacle_distance_m));
}

TEST(SectorMapBuilderTest, TheNearerOfTheObstacleAndTheCertifiedRangeDecides)
{
  for (const float obstacle_m : {8.0F, 20.0F}) {
    SectorMapBuilder builder{kResolved};
    builder.reset(true);
    builder.addRay(makeReturn(obstacle_m));
    builder.addRay(makeReturn(obstacle_m + 0.1F));
    addOpenSky(builder);
    const auto sector = builder.finalize().sectors[0];
    EXPECT_EQ(sector.state, SectorState::kOccupied) << obstacle_m;
    EXPECT_FLOAT_EQ(sector.distance_m, std::min(obstacle_m, 12.0F)) << obstacle_m;
    EXPECT_FLOAT_EQ(sector.obstacle_distance_m, obstacle_m) << obstacle_m;
  }
}

TEST(SectorMapBuilderTest, ThePartialSectorIsUnknownUntilBothHalvesHaveEnoughRays)
{
  const float up = 3.0F * kDegreesToRadians;
  SectorMapBuilder only_up{kConfig};
  only_up.reset(true);
  only_up.addRay(makeNoReturn(0.0F, 1'000U, up));
  only_up.addRay(makeNoReturn(0.0F, 1'000U, up));
  EXPECT_EQ(only_up.finalize().sectors[0].state, SectorState::kUnknown);

  SectorMapBuilder one_horizon_ray{kConfig};
  one_horizon_ray.reset(true);
  one_horizon_ray.addRay(makeNoReturn());
  EXPECT_EQ(one_horizon_ray.finalize().sectors[0].state, SectorState::kUnknown);
}

// Issue #148, SYS-CP-029: a half outside the field of view is not a half filled by a wall.
TEST(SectorMapBuilderTest, ConfirmedObstacleWithAHalfOutsideTheFieldOfViewIsUnknown)
{
  // A confirmed obstacle at 8 m, free rays in the upper half only (the lower half lies outside the field
  // of view): the sector is unknown, cap 0, not occupied at 8 m, so the cap cannot rise on it.
  const float three = 3.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(8.0F, 0.0F, 1'000U, three));  // in the band (0.42 m high), upper half
  builder.addRay(makeReturn(8.1F, 0.0F, 1'000U, three));
  builder.addRay(makeNoReturn(0.0F, 1'000U, three));
  builder.addRay(makeNoReturn(0.0F, 1'000U, three));
  builder.addRay(makeNoReturn(0.0F, 1'000U, three));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kUnknown);
  EXPECT_TRUE(std::isnan(sector.distance_m));
  EXPECT_NEAR(sector.obstacle_distance_m, 8.0F * std::cos(three), 1.0e-3F);  // kept as a diagnostic
}

TEST(SectorMapBuilderTest, AHalfWithFewerThanNFreeRaysIsOutsideTheFieldOfView)
{
  const float up = 3.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(8.0F, 0.0F, 1'000U, up));
  builder.addRay(makeReturn(8.1F, 0.0F, 1'000U, up));
  builder.addRay(makeNoReturn(0.0F, 1'000U, up));
  builder.addRay(makeNoReturn(0.0F, 1'000U, up));
  builder.addRay(makeNoReturn(0.0F, 1'000U, -up));  // one ray below, n_free is 2

  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, AWallThatFillsBothHalvesStaysOccupiedAtItsDistance)
{
  // Returns above and below the horizon, all inside the band: both halves are seen, filled by obstacle.
  const float two = 2.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(8.0F, 0.0F, 1'000U, two));
  builder.addRay(makeReturn(8.1F, 0.0F, 1'000U, two));
  builder.addRay(makeReturn(8.0F, 0.0F, 1'000U, -two));
  builder.addRay(makeReturn(8.1F, 0.0F, 1'000U, -two));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_NEAR(sector.distance_m, 8.0F * std::cos(two), 1.0e-3F);
}

TEST(SectorMapBuilderTest, ObstacleReturnsAtTheHorizonCountForBothHalves)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(8.0F));
  builder.addRay(makeReturn(8.1F));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
}

TEST(SectorMapBuilderTest, WithoutPacketContinuityAConfirmedObstacleStaysOccupiedWhateverTheCoverage)
{
  const float three = 3.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(false);
  builder.addRay(makeReturn(8.0F));
  builder.addRay(makeReturn(8.1F));
  builder.addRay(makeNoReturn(0.0F, 1'000U, three));
  builder.addRay(makeNoReturn(0.0F, 1'000U, three));

  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kOccupied);
}

TEST(SectorMapBuilderTest, ReturnBeyondTheQualifiedRangeIsFreeEvidence)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(47.0F));
  builder.addRay(makeReturn(55.0F));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kObservedClear);
  EXPECT_EQ(sector.clear_ray_count, 2U);
}

TEST(SectorMapBuilderTest, AWallThatFillsTheSectorGivesItsDistanceNotZero)
{
  // Every ray hits: no free ray exists, and that must not shorten the confirmed distance.
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(30.0F));
  builder.addRay(makeReturn(30.0F));
  builder.addRay(makeReturn(30.1F));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(sector.distance_m, 30.0F);
}

TEST(SectorMapBuilderTest, SteepOutOfBandReturnsStillCoverTheirHalf)
{
  // Steep returns outside the band count for the coverage of their half and certify the certified range.
  const float sixty = 60.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(41.0F, 0.0F, 1'000U, sixty));
  builder.addRay(makeReturn(41.0F, 0.0F, 1'000U, sixty));
  builder.addRay(makeReturn(41.0F, 0.0F, 1'000U, -sixty));
  builder.addRay(makeReturn(41.0F, 0.0F, 1'000U, -sixty));

  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kObservedClear);
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

  auto straight_up = makeNoReturn();
  straight_up.elevation_rad = obstacle_sector_map::kHalfPiRad;
  builder.addRay(straight_up);

  const auto map = builder.finalize();
  EXPECT_EQ(map.accepted_ray_count, 0U);
  EXPECT_EQ(map.rejected_ray_count, 4U);
  EXPECT_EQ(map.sectors[0].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, RejectsRayWithZeroTimestamp)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeNoReturn(0.0F, 0U));

  const auto map = builder.finalize();
  EXPECT_EQ(map.accepted_ray_count, 0U);
  EXPECT_EQ(map.rejected_ray_count, 1U);
}

TEST(SectorMapBuilderTest, RejectsReturnWithInvalidRange)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(std::numeric_limits<float>::quiet_NaN()));
  builder.addRay(makeReturn(0.0F));
  builder.addRay(makeReturn(-1.0F));

  const auto map = builder.finalize();
  EXPECT_EQ(map.accepted_ray_count, 0U);
  EXPECT_EQ(map.rejected_ray_count, 3U);
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
  SectorMapBuilder builder{kSingle};
  builder.reset(true);
  builder.addRay(makeReturn(10.0F, 2.5F * kDegreesToRadians));

  const auto map = builder.finalize();
  EXPECT_EQ(map.sectors[0].state, SectorState::kOccupied);
  EXPECT_EQ(map.sectors[1].state, SectorState::kOccupied);
}

TEST(SectorMapBuilderTest, ExactNegativeBoundaryUpdatesBothAdjacentSectors)
{
  SectorMapBuilder builder{kSingle};
  builder.reset(true);
  builder.addRay(makeReturn(10.0F, -2.5F * kDegreesToRadians));

  const auto map = builder.finalize();
  EXPECT_EQ(map.sectors[0].state, SectorState::kOccupied);
  EXPECT_EQ(map.sectors[71].state, SectorState::kOccupied);
}

TEST(SectorMapBuilderTest, AllFloatBoundariesUpdateBothAdjacentSectors)
{
  for (std::size_t lower_index = 0U; lower_index < kSectorCount; ++lower_index) {
    SCOPED_TRACE(lower_index);
    SectorMapBuilder builder{kSingle};
    builder.reset(true);
    const float boundary_rad =
      (static_cast<float>(lower_index) + 0.5F) * (360.0F / static_cast<float>(kSectorCount)) *
      kDegreesToRadians;
    builder.addRay(makeReturn(10.0F, boundary_rad));

    const auto map = builder.finalize();
    EXPECT_EQ(map.sectors[lower_index].state, SectorState::kOccupied);
    EXPECT_EQ(map.sectors[(lower_index + 1U) % kSectorCount].state, SectorState::kOccupied);
  }
}

TEST(SectorMapBuilderTest, JustOffBoundaryDoesNotDuplicateObstacleEvidence)
{
  constexpr float kBoundaryRad = 27.5F * kDegreesToRadians;
  constexpr float kOutsideNumericalToleranceRad = 1.0e-4F;

  SectorMapBuilder lower_builder{kSingle};
  lower_builder.reset(true);
  lower_builder.addRay(makeReturn(10.0F, kBoundaryRad - kOutsideNumericalToleranceRad));
  const auto lower_map = lower_builder.finalize();
  EXPECT_EQ(lower_map.sectors[5].state, SectorState::kOccupied);
  EXPECT_EQ(lower_map.sectors[6].state, SectorState::kUnknown);

  SectorMapBuilder upper_builder{kSingle};
  upper_builder.reset(true);
  upper_builder.addRay(makeReturn(10.0F, kBoundaryRad + kOutsideNumericalToleranceRad));
  const auto upper_map = upper_builder.finalize();
  EXPECT_EQ(upper_map.sectors[5].state, SectorState::kUnknown);
  EXPECT_EQ(upper_map.sectors[6].state, SectorState::kOccupied);
}

TEST(SectorMapBuilderTest, BoundaryNoReturnDoesNotDuplicateFreeEvidence)
{
  SectorMapBuilder builder{kConfig};
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
  SectorMapBuilder builder{kSingle};
  builder.reset(true);
  builder.addRay(makeReturn(11.0F, 3.14159265358979323846F));
  builder.addRay(makeReturn(9.0F, -3.14159265358979323846F));

  const auto map = builder.finalize();
  EXPECT_EQ(map.sectors[36].state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(map.sectors[36].distance_m, 9.0F);
  EXPECT_EQ(map.accepted_ray_count, 2U);
}

// Issue #141, ADR 0020 D3 option B: behind the vehicle the lidar never sees below its own height.
constexpr float kRearStart = 134.6F * kDegreesToRadians;

SectorMapConfig rearConfig()
{
  SectorMapConfig config = kResolved;
  config.rear_start_rad = kRearStart;
  return config;
}

TEST(SectorMapConfigTest, TheRearStartMustBeAnAngleFromZeroToPi)
{
  SectorMapConfig config = kConfig;
  EXPECT_TRUE(config.isValid());  // the default is pi: no rear class
  for (const float bad : {0.0F, -1.0F, 3.2F, std::numeric_limits<float>::quiet_NaN()}) {
    config.rear_start_rad = bad;
    EXPECT_FALSE(config.isValid()) << bad;
  }
  config.rear_start_rad = kRearStart;
  EXPECT_TRUE(config.isValid());
}

TEST(SectorMapBuilderTest, ARearSectorCertifiesFromTheUpperHalfAlone)
{
  const float pi = 3.14159265358979F;
  const float three = 3.0F * kDegreesToRadians;
  SectorMapBuilder builder{rearConfig()};
  builder.reset(true);
  builder.addRay(makeNoReturn(pi, 1'000U, three));
  builder.addRay(makeNoReturn(pi, 1'000U, three));

  const auto sector = builder.finalize().sectors[36];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_FLOAT_EQ(sector.distance_m, 12.0F);  // the certified range (ADR 0024)
}

TEST(SectorMapBuilderTest, TheSameRaysInTheFrontClassAreUnknownBecauseTheLowerHalfIsMissing)
{
  const float three = 3.0F * kDegreesToRadians;
  SectorMapBuilder builder{rearConfig()};
  builder.reset(true);
  builder.addRay(makeNoReturn(0.0F, 1'000U, three));
  builder.addRay(makeNoReturn(0.0F, 1'000U, three));
  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, AConfirmedObstacleInTheUpperHalfBehindTheVehicleStillLimits)
{
  const float pi = 3.14159265358979F;
  const float three = 3.0F * kDegreesToRadians;
  SectorMapBuilder builder{rearConfig()};
  builder.reset(true);
  builder.addRay(makeReturn(8.0F, pi, 1'000U, three));  // 0.42 m above the lidar, inside the band
  builder.addRay(makeReturn(8.1F, pi, 1'000U, three));

  const auto sector = builder.finalize().sectors[36];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_NEAR(sector.distance_m, 8.0F * std::cos(three), 1.0e-3F);
}

TEST(SectorMapBuilderTest, ARearSectorStillNeedsNFreeRaysInTheUpperHalf)
{
  const float pi = 3.14159265358979F;
  SectorMapBuilder builder{rearConfig()};
  builder.reset(true);
  builder.addRay(makeNoReturn(pi, 1'000U, 3.0F * kDegreesToRadians));  // one ray, n_free is 2
  EXPECT_EQ(builder.finalize().sectors[36].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, TheRearClassStartsAtTheCentreBearingOfTheSector)
{
  const float three = 3.0F * kDegreesToRadians;
  SectorMapBuilder builder{rearConfig()};
  builder.reset(true);
  // sector 27 is centred at 135 degrees (rear), sector 26 at 130 degrees (not rear); the mirror sectors 45 and 46
  for (const std::size_t index : {27U, 26U, 45U, 46U}) {
    const float azimuth = static_cast<float>(index) * 5.0F * kDegreesToRadians;
    builder.addRay(makeNoReturn(azimuth, 1'000U, three));
    builder.addRay(makeNoReturn(azimuth, 1'000U, three));
  }
  const auto map = builder.finalize();
  EXPECT_EQ(map.sectors[27].state, SectorState::kOccupied);
  EXPECT_EQ(map.sectors[45].state, SectorState::kOccupied);
  EXPECT_EQ(map.sectors[26].state, SectorState::kUnknown);
  EXPECT_EQ(map.sectors[46].state, SectorState::kUnknown);
}

TEST(SectorMapBuilderTest, ResetClearsTheCoverageOfThePreviousFrame)
{
  // A reused builder (SectorMapPipeline::step) must describe the current frame only: coverage of both
  // halves in frame 1 must not hide the lost lower half of frame 2 (review of #167).
  const float three = 3.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(8.0F));  // horizon returns: both halves seen
  builder.addRay(makeReturn(8.1F));
  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kOccupied);

  builder.reset(true);
  builder.addRay(makeReturn(8.0F, 0.0F, 1'000U, three));  // upper half only
  builder.addRay(makeReturn(8.1F, 0.0F, 1'000U, three));
  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kUnknown);

  SectorMapBuilder fresh{kConfig};
  fresh.reset(true);
  fresh.addRay(makeReturn(8.0F, 0.0F, 1'000U, three));
  fresh.addRay(makeReturn(8.1F, 0.0F, 1'000U, three));
  EXPECT_EQ(fresh.finalize().sectors[0].state, builder.finalize().sectors[0].state);
}

TEST(SectorMapBuilderTest, ResetClearsPreviousWindow)
{
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(5.0F, 0.0F, 7'000U));
  builder.addRay(makeReturn(5.1F, 0.0F, 7'000U));
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
    EXPECT_TRUE(std::isnan(sector.obstacle_distance_m));
  }
}

// ADR 0024 decision 3: the attitude allowance widens the band test.
TEST(SectorMapConfigTest, TheCertifiedRangeAndTheAllowanceAreChecked)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 12.0F, 0.01F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 0.05F, 0.0F}).isValid());   // below 10 cm
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 40.5F, 0.0F}).isValid());   // above the 40 m cap
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, nan, 0.0F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 12.0F, -0.01F}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 12.0F, nan}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 12.0F, 1.6F}).isValid());   // not below pi/2
}

TEST(SectorMapBuilderTest, TheAllowanceMakesABandEdgeReturnObstacleEvidence)
{
  // Returns 20 m away, 1.2 m up: 0.2 m above the band (h_up 1 m). Without an allowance they cover the upper half
  // only; with 1 deg (20 m * tan(1 deg) = 0.35 m of slack) they may lie in the band and confirm an obstacle.
  const float elevation = std::atan2(1.2F, 20.0F);
  const float range = std::hypot(20.0F, 1.2F);
  for (const float allowance_deg : {0.0F, 1.0F}) {
    SectorMapBuilder builder{SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 40.0F, allowance_deg * kDegreesToRadians}};
    builder.reset(true);
    builder.addRay(makeReturn(range, 0.0F, 1'000U, elevation));
    builder.addRay(makeReturn(range, 0.0F, 1'000U, elevation));
    addOpenSky(builder);
    const auto sector = builder.finalize().sectors[0];
    if (allowance_deg > 0.0F) {
      EXPECT_EQ(sector.state, SectorState::kOccupied);
      EXPECT_NEAR(sector.distance_m, 20.0F, 1.0e-3F);
    } else {
      EXPECT_EQ(sector.state, SectorState::kObservedClear);
    }
  }
}

TEST(SectorMapBuilderTest, TheAllowanceWidensTheLowerEdgeToo)
{
  const float elevation = std::atan2(-1.2F, 20.0F);
  const float range = std::hypot(20.0F, 1.2F);
  SectorMapBuilder builder{SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 2U, 40.0F, 1.0F * kDegreesToRadians}};
  builder.reset(true);
  builder.addRay(makeReturn(range, 0.0F, 1'000U, elevation));
  builder.addRay(makeReturn(range, 0.0F, 1'000U, elevation));
  addOpenSky(builder);
  EXPECT_EQ(builder.finalize().sectors[0].state, SectorState::kOccupied);
}
