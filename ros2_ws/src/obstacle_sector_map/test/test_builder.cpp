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
// Test values only: h_up 1 m, h_down 1 m, dr 0.5 m, n_obstacle 2, n_free 2. Not platform values.
constexpr SectorMapConfig kConfig{1.0F, 1.0F, 0.5F, 2U, 2U};
// One return confirms an obstacle and one ray certifies free distance.
constexpr SectorMapConfig kSingle{1.0F, 1.0F, 0.5F, 1U, 1U};

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
  EXPECT_FALSE((SectorMapConfig{0.0F, 1.0F, 0.5F, 2U, 2U}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, -1.0F, 0.5F, 2U, 2U}).isValid());
  EXPECT_FALSE((SectorMapConfig{nan, 1.0F, 0.5F, 2U, 2U}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, nan, 0.5F, 2U, 2U}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.0F, 2U, 2U}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, nan, 2U, 2U}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 0U, 2U}).isValid());
  EXPECT_FALSE((SectorMapConfig{1.0F, 1.0F, 0.5F, 2U, 0U}).isValid());
}

TEST(SectorMapBuilderTest, RejectsInvalidConfiguration)
{
  EXPECT_THROW(
    SectorMapBuilder((SectorMapConfig{1.0F, 1.0F, 0.5F, 0U, 2U})), std::invalid_argument);
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

TEST(SectorMapBuilderTest, GroundBelowTheBandCertifiesOnlyWhereItLeavesTheBand)
{
  // Returns at -30 degrees and 10 m are 5 m below: outside the band. Each certifies
  // h_down / tan(30 degrees) = 1.73 m of the lower half; the upper half has two rays at +5 degrees.
  const float down = -30.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, down));
  builder.addRay(makeReturn(10.0F, 0.0F, 1'000U, down));
  builder.addRay(makeNoReturn(0.0F, 1'000U, 5.0F * kDegreesToRadians));
  builder.addRay(makeNoReturn(0.0F, 1'000U, 5.0F * kDegreesToRadians));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_NEAR(sector.distance_m, 1.0F / std::tan(30.0F * kDegreesToRadians), 1.0e-3F);
}

TEST(SectorMapBuilderTest, FreeDistanceIsLimitedByWhereTheRayLeavesTheBand)
{
  // No-returns at +-5 degrees: h / tan(5 degrees) = 11.43 m, shorter than 40 m * cos(5 degrees).
  const float five = 5.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeNoReturn(0.0F, 1'000U, five));
  builder.addRay(makeNoReturn(0.0F, 1'000U, five));
  builder.addRay(makeNoReturn(0.0F, 1'000U, -five));
  builder.addRay(makeNoReturn(0.0F, 1'000U, -five));

  const auto sector = builder.finalize().sectors[0];
  EXPECT_EQ(sector.state, SectorState::kOccupied);
  EXPECT_NEAR(sector.distance_m, 1.0F / std::tan(five), 1.0e-3F);
}

TEST(SectorMapBuilderTest, TheSmallerHalfDecides)
{
  const float five = 5.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeNoReturn(0.0F, 1'000U, five));
  builder.addRay(makeNoReturn(0.0F, 1'000U, five));
  builder.addRay(makeNoReturn(0.0F, 1'000U, -0.5F * kDegreesToRadians));
  builder.addRay(makeNoReturn(0.0F, 1'000U, -0.5F * kDegreesToRadians));

  EXPECT_NEAR(builder.finalize().sectors[0].distance_m, 1.0F / std::tan(five), 1.0e-3F);
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

TEST(SectorMapBuilderTest, FreeDistanceShorterThanTheObstacleWins)
{
  // Confirmed obstacle at 20 m, but the free rays leave the band at 11.43 m.
  const float five = 5.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(20.0F));
  builder.addRay(makeReturn(20.1F));
  builder.addRay(makeNoReturn(0.0F, 1'000U, five));
  builder.addRay(makeNoReturn(0.0F, 1'000U, five));
  builder.addRay(makeNoReturn(0.0F, 1'000U, -five));
  builder.addRay(makeNoReturn(0.0F, 1'000U, -five));

  EXPECT_NEAR(builder.finalize().sectors[0].distance_m, 1.0F / std::tan(five), 1.0e-3F);
}

TEST(SectorMapBuilderTest, SteepOutOfBandReturnsDoNotShortenTheFreeDistanceOfHorizonRays)
{
  // The n_free largest values decide: two horizon rays (40 m) outvote two steep rays (0.58 m).
  const float sixty = 60.0F * kDegreesToRadians;
  SectorMapBuilder builder{kConfig};
  builder.reset(true);
  builder.addRay(makeReturn(41.0F, 0.0F, 1'000U, sixty));
  builder.addRay(makeReturn(41.0F, 0.0F, 1'000U, sixty));
  builder.addRay(makeNoReturn());
  builder.addRay(makeNoReturn());

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
