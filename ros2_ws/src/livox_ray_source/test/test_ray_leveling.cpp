#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "livox_ray_source/pose_history.hpp"
#include "livox_ray_source/ray_leveling.hpp"

using livox_ray_source::breaksFrameContinuity;
using livox_ray_source::LevelingConfig;
using livox_ray_source::LevelingStatus;
using livox_ray_source::PoseHistory;
using livox_ray_source::PoseHistoryConfig;
using livox_ray_source::PoseSample;
using livox_ray_source::RayLeveler;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;

namespace
{

// All values below are test fixtures. None is an approved platform value or threshold.
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kFixtureMountPitchDownRad = 10.0 * kDeg;  // the ADR 0020 example, as a fixture
constexpr double kFixtureOriginThresholdM = 0.4;
constexpr uint64_t kMs = 1'000'000U;
constexpr float kAngleTol = 1.0e-5F;
constexpr float kRangeTol = 1.0e-4F;

using Vec = std::array<double, 3>;
using Mat = std::array<std::array<double, 3>, 3>;

// The helpers below are an independent, closed-form implementation of the geometry, written the
// other way round (from a world point to a sensor ray), so that they check the code under test
// and do not repeat it.
Mat rx(double a)
{
  return {{{1, 0, 0}, {0, std::cos(a), -std::sin(a)}, {0, std::sin(a), std::cos(a)}}};
}
Mat ry(double a)
{
  return {{{std::cos(a), 0, std::sin(a)}, {0, 1, 0}, {-std::sin(a), 0, std::cos(a)}}};
}
Mat rz(double a)
{
  return {{{std::cos(a), -std::sin(a), 0}, {std::sin(a), std::cos(a), 0}, {0, 0, 1}}};
}
Mat mul(const Mat & a, const Mat & b)
{
  Mat c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {c[i][j] += a[i][k] * b[k][j];}
    }
  }
  return c;
}
Mat transpose(const Mat & a)
{
  Mat t{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {t[i][j] = a[j][i];}
  }
  return t;
}
Vec matVec(const Mat & a, const Vec & v)
{
  Vec r{};
  for (int i = 0; i < 3; ++i) {r[i] = a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2];}
  return r;
}

// Body FRD to NED for aerospace roll, pitch, yaw (positive pitch is nose up, positive roll is the
// right side down, yaw is clockwise seen from above).
Mat bodyFrdToNed(double roll, double pitch, double yaw)
{
  return mul(rz(yaw), mul(ry(pitch), rx(roll)));
}

std::array<double, 4> quaternionOf(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy};
}

PoseSample poseOf(
  double roll, double pitch, double yaw, const Vec & position = {0.0, 0.0, 0.0},
  uint8_t reset_counter = 0U, uint64_t time_ns = 1U)
{
  PoseSample s;
  s.timestamp_sample_ns = time_ns;
  s.q_wxyz = quaternionOf(roll, pitch, yaw);
  s.position_ned_m = position;
  s.reset_counter = reset_counter;
  return s;
}

// Mount angles of a fixture, in the order and sign of LevelingConfig.
struct Mount
{
  double roll_rad;
  double pitch_down_rad;
  double yaw_left_rad;
};

Mount mountOf(double roll, double pitch_down, double yaw_left)
{
  return Mount{roll, pitch_down, yaw_left};
}

LevelingConfig configOf(const Mount & mount, double origin_motion_threshold_m = kFixtureOriginThresholdM)
{
  return LevelingConfig{
    mount.roll_rad, mount.pitch_down_rad, mount.yaw_left_rad, origin_motion_threshold_m};
}

// Sensor FLU ray (azimuth, elevation, range) of a point given in the output frame convention:
// direct spherical conversion.
RaySample sensorReturn(double range_m, double azimuth_rad, double elevation_rad)
{
  return RaySample{
    1'000U, static_cast<float>(range_m), static_cast<float>(azimuth_rad),
    static_cast<float>(elevation_rad), false, RayReturnKind::kReturn};
}

// The ray a sensor on a vehicle with the given attitude and mount would measure for a point
// `p_ned` (NED, relative to the vehicle position at the ray time).
RaySample rayToNedPoint(
  const Vec & p_ned, double roll, double pitch, double yaw, const Mount & mount)
{
  const Vec p_frd = matVec(transpose(bodyFrdToNed(roll, pitch, yaw)), p_ned);
  const Vec p_body_flu{p_frd[0], -p_frd[1], -p_frd[2]};
  const Mat mount_flu = mul(rz(mount.yaw_left_rad), mul(ry(mount.pitch_down_rad), rx(mount.roll_rad)));
  const Vec p_s = matVec(transpose(mount_flu), p_body_flu);
  const double range = std::sqrt(p_s[0] * p_s[0] + p_s[1] * p_s[1] + p_s[2] * p_s[2]);
  return sensorReturn(
    range, std::atan2(p_s[1], p_s[0]), std::atan2(p_s[2], std::hypot(p_s[0], p_s[1])));
}

constexpr uint64_t kFixtureMaxGapNs = 1'000'000'000U;
constexpr uint64_t kRefNs = 500U;  // differs from the ray time (1000 ns) of the helpers

PoseSample at(PoseSample pose, uint64_t time_ns)
{
  pose.timestamp_sample_ns = time_ns;
  return pose;
}

PoseHistoryConfig fixtureHistoryConfig()
{
  return PoseHistoryConfig{8U, kFixtureMaxGapNs};
}

// A history with one pose at the reference time and `at_ray` at 900 ns and 1100 ns, so that every
// query between them (the ray time of the helpers is 1000 ns) gets `at_ray` and a query at the
// reference time gets `at_reference`.
class TwoPoseHistory : public PoseHistory
{
public:
  TwoPoseHistory(const PoseSample & at_reference, const PoseSample & at_ray)
  : PoseHistory(fixtureHistoryConfig())
  {
    push(at(at_reference, kRefNs));
    push(at(at_ray, 900U));
    push(at(at_ray, 1100U));
  }
};

// The same pose at every time of the helpers.
class ConstantPoseHistory : public TwoPoseHistory
{
public:
  explicit ConstantPoseHistory(const PoseSample & pose)
  : TwoPoseHistory(pose, pose) {}
};

const Mount kZeroMount = mountOf(0.0, 0.0, 0.0);

}  // namespace

// --- Configuration: sentinels and validity -------------------------------------------------

TEST(LevelingConfigTest, DefaultHoldsTheSentinelsAndIsNotValid)
{
  const LevelingConfig config;
  EXPECT_TRUE(std::isnan(config.roll_rad));
  EXPECT_TRUE(std::isnan(config.pitch_down_rad));
  EXPECT_TRUE(std::isnan(config.yaw_left_rad));
  EXPECT_TRUE(std::isnan(config.origin_motion_threshold_m));
  EXPECT_FALSE(config.isValid());
  const ConstantPoseHistory poses{poseOf(0, 0, 0)};
  EXPECT_THROW((RayLeveler{poses, config}), std::invalid_argument);
}

TEST(LevelingConfigTest, EachRequiredValueIsChecked)
{
  EXPECT_TRUE(configOf(kZeroMount).isValid());
  EXPECT_TRUE(configOf(mountOf(0.0, kFixtureMountPitchDownRad, 0.0)).isValid());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(configOf(mountOf(nan, 0.0, 0.0)).isValid());
  EXPECT_FALSE(configOf(mountOf(0.0, nan, 0.0)).isValid());
  EXPECT_FALSE(configOf(mountOf(0.0, 0.0, nan)).isValid());
  EXPECT_FALSE(configOf(mountOf(0.0, inf, 0.0)).isValid());
  EXPECT_FALSE(configOf(mountOf(0.0, kPi + 0.1, 0.0)).isValid());
  EXPECT_FALSE(configOf(mountOf(0.0, 0.0, -kPi - 0.1)).isValid());
  LevelingConfig c = configOf(kZeroMount);
  c.origin_motion_threshold_m = nan;
  EXPECT_FALSE(c.isValid());
  c.origin_motion_threshold_m = 0.0;
  EXPECT_FALSE(c.isValid());
  c.origin_motion_threshold_m = -1.0;
  EXPECT_FALSE(c.isValid());
  c.origin_motion_threshold_m = inf;
  EXPECT_FALSE(c.isValid());
}

// --- Frame conventions -----------------------------------------------------------------------

TEST(RayLevelerTest, ZeroPoseAndZeroMountReproduceTheFluRayDirectly)
{
  const ConstantPoseHistory poses{poseOf(0, 0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  for (const double az : {-2.5, -1.0, 0.0, 0.7, 3.0}) {
    for (const double el : {-0.4, 0.0, 0.3}) {
      RaySample ray = sensorReturn(7.5, az, el);
      ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
      EXPECT_NEAR(ray.range_m, 7.5F, kRangeTol);
      EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(az), kAngleTol);
      EXPECT_NEAR(ray.elevation_rad, static_cast<float>(el), kAngleTol);
    }
  }
}

TEST(RayLevelerTest, TheTimestampTheMaskAndTheKindAreKept)
{
  const ConstantPoseHistory poses{poseOf(0.1, -0.2, 0.3)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(5.0, 0.2, 0.1);
  ray.timestamp_ns = 1'050U;
  ray.masked = true;
  ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
  EXPECT_EQ(ray.timestamp_ns, 1'050U);
  EXPECT_TRUE(ray.masked);
  EXPECT_EQ(ray.return_kind, RayReturnKind::kReturn);
}

TEST(RayLevelerTest, MountPitchedDownPutsTheBoresightBelowTheHorizonWithALevelVehicle)
{
  // Convention: positive pitch_down_rad tilts the sensor boresight (sensor +x) DOWN, so with a
  // level vehicle a ray along the boresight has an elevation of minus the mount angle. A ray at
  // +mount angle in the sensor frame is the body horizon.
  const ConstantPoseHistory poses{poseOf(0, 0, 0)};
  RayLeveler leveler{poses, configOf(mountOf(0.0, kFixtureMountPitchDownRad, 0.0))};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);

  RaySample boresight = sensorReturn(10.0, 0.0, 0.0);
  ASSERT_EQ(leveler.level(boresight), LevelingStatus::kOk);
  EXPECT_NEAR(boresight.elevation_rad, static_cast<float>(-kFixtureMountPitchDownRad), kAngleTol);
  EXPECT_NEAR(boresight.azimuth_rad, 0.0F, kAngleTol);

  RaySample horizon = sensorReturn(10.0, 0.0, kFixtureMountPitchDownRad);
  ASSERT_EQ(leveler.level(horizon), LevelingStatus::kOk);
  EXPECT_NEAR(horizon.elevation_rad, 0.0F, kAngleTol);
  EXPECT_NEAR(horizon.azimuth_rad, 0.0F, kAngleTol);

  // Sideways the pitch of the mount does not change the elevation.
  RaySample left = sensorReturn(10.0, kPi / 2.0, 0.0);
  ASSERT_EQ(leveler.level(left), LevelingStatus::kOk);
  EXPECT_NEAR(left.elevation_rad, 0.0F, kAngleTol);
  EXPECT_NEAR(left.azimuth_rad, static_cast<float>(kPi / 2.0), kAngleTol);

  // Behind the vehicle the boresight tilt is a rise: the sensor's rear ray points up.
  RaySample behind = sensorReturn(10.0, kPi, 0.0);
  ASSERT_EQ(leveler.level(behind), LevelingStatus::kOk);
  EXPECT_NEAR(behind.elevation_rad, static_cast<float>(kFixtureMountPitchDownRad), kAngleTol);
}

TEST(RayLevelerTest, MountRollAndYawFollowTheirDocumentedSigns)
{
  // Roll about forward, positive = left side up: the sensor's left axis (+y) rises.
  {
    const ConstantPoseHistory poses{poseOf(0, 0, 0)};
    RayLeveler leveler{poses, configOf(mountOf(0.1, 0.0, 0.0))};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample left = sensorReturn(10.0, kPi / 2.0, 0.0);
    ASSERT_EQ(leveler.level(left), LevelingStatus::kOk);
    EXPECT_NEAR(left.elevation_rad, 0.1F, kAngleTol);
  }
  // Yaw about up, positive = boresight turned to the left.
  {
    const ConstantPoseHistory poses{poseOf(0, 0, 0)};
    RayLeveler leveler{poses, configOf(mountOf(0.0, 0.0, 0.2))};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample boresight = sensorReturn(10.0, 0.0, 0.0);
    ASSERT_EQ(leveler.level(boresight), LevelingStatus::kOk);
    EXPECT_NEAR(boresight.azimuth_rad, 0.2F, kAngleTol);
  }
}

TEST(RayLevelerTest, LateralAndVerticalSignsAreNotMirrored)
{
  // A point on the vehicle's left stays positive left, a point above stays positive up, for a
  // level vehicle with any heading (the heading is the reference heading too).
  for (const double yaw : {0.0, 0.5, -2.0, 3.0}) {
    const ConstantPoseHistory poses{poseOf(0, 0, yaw)};
    RayLeveler leveler{poses, configOf(kZeroMount)};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample left = sensorReturn(4.0, kPi / 2.0, 0.0);
    RaySample right = sensorReturn(4.0, -kPi / 2.0, 0.0);
    RaySample up = sensorReturn(4.0, 0.0, 0.5);
    RaySample down = sensorReturn(4.0, 0.0, -0.5);
    ASSERT_EQ(leveler.level(left), LevelingStatus::kOk);
    ASSERT_EQ(leveler.level(right), LevelingStatus::kOk);
    ASSERT_EQ(leveler.level(up), LevelingStatus::kOk);
    ASSERT_EQ(leveler.level(down), LevelingStatus::kOk);
    EXPECT_NEAR(left.azimuth_rad, static_cast<float>(kPi / 2.0), kAngleTol) << yaw;
    EXPECT_NEAR(right.azimuth_rad, static_cast<float>(-kPi / 2.0), kAngleTol) << yaw;
    EXPECT_NEAR(up.elevation_rad, 0.5F, kAngleTol) << yaw;
    EXPECT_NEAR(down.elevation_rad, -0.5F, kAngleTol) << yaw;
  }
}

TEST(RayLevelerTest, AnObstacleOnTheLeftOfARolledVehicleStaysOnTheLeft)
{
  // Rolled right side down by 30 degrees: the ray to a point 3 m to the left and level with the
  // vehicle has a body elevation above the horizon, but leveling must put it back at azimuth +90
  // degrees (left) and elevation 0.
  const double roll = 30.0 * kDeg;
  const Vec p_ned{0.0, -3.0, 0.0};  // yaw 0: west is the vehicle's left
  const ConstantPoseHistory poses{poseOf(roll, 0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = rayToNedPoint(p_ned, roll, 0.0, 0.0, kZeroMount);
  ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(kPi / 2.0), kAngleTol);
  EXPECT_NEAR(ray.elevation_rad, 0.0F, kAngleTol);
  EXPECT_NEAR(ray.range_m, 3.0F, kRangeTol);
}

TEST(RayLevelerTest, HeadingChangeSinceTheReferenceRotatesTheRay)
{
  // The vehicle faces east (yaw 90 degrees) at the ray time and north at the reference time. A
  // ray along the body forward axis points east, which is to the RIGHT of the reference heading:
  // azimuth minus 90 degrees (counter-clockwise positive).
  const TwoPoseHistory poses{poseOf(0, 0, 0), poseOf(0, 0, kPi / 2.0)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(6.0, 0.0, 0.0);
  ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(-kPi / 2.0), kAngleTol);
  EXPECT_NEAR(ray.range_m, 6.0F, kRangeTol);
  // A ray to the body's left points north, which is straight ahead at the reference heading.
  RaySample left = sensorReturn(6.0, kPi / 2.0, 0.0);
  ASSERT_EQ(leveler.level(left), LevelingStatus::kOk);
  EXPECT_NEAR(left.azimuth_rad, 0.0F, kAngleTol);
}

TEST(RayLevelerTest, ACommonHeadingOffsetOfBothPosesDoesNotMatter)
{
  const TwoPoseHistory poses{poseOf(0, 0, 1.0), poseOf(0, 0, 1.0)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(6.0, 0.3, 0.1);
  ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.azimuth_rad, 0.3F, kAngleTol);
  EXPECT_NEAR(ray.elevation_rad, 0.1F, kAngleTol);
}

// --- Leveling: sector-equivalent results for a known wall --------------------------------------

namespace
{

struct Attitude
{
  double roll;
  double pitch;
  double yaw;
};

// A flat wall 5 m ahead of the reference heading, 0.7 m above the vehicle height. For each wall
// azimuth the ray the tilted vehicle measures is built from the world point, leveled, and
// compared with the level vehicle: same azimuth, same horizontal range, same elevation.
void expectWallEquivalentToLevelVehicle(
  const Attitude & at_ray, const Attitude & at_reference, const Mount & mount)
{
  const Vec vehicle{12.0, -7.0, -20.0};
  const double psi = at_reference.yaw;
  const double wall_ahead_m = 5.0;
  const double wall_height_m = 0.7;
  const TwoPoseHistory poses{poseOf(at_reference.roll, at_reference.pitch, at_reference.yaw, vehicle),
    poseOf(at_ray.roll, at_ray.pitch, at_ray.yaw, vehicle)};
  RayLeveler leveler{poses, configOf(mount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  for (const double wall_azimuth_deg : {-60.0, -30.0, -5.0, 0.0, 12.0, 45.0, 70.0}) {
    const double alpha = wall_azimuth_deg * kDeg;
    const double lateral_left_m = wall_ahead_m * std::tan(alpha);
    // Forward and left of the reference heading, in NED (north, east, down).
    const Vec forward{std::cos(psi), std::sin(psi), 0.0};
    const Vec left{std::sin(psi), -std::cos(psi), 0.0};
    const Vec p_ned{
      wall_ahead_m * forward[0] + lateral_left_m * left[0],
      wall_ahead_m * forward[1] + lateral_left_m * left[1], -wall_height_m};
    RaySample ray = rayToNedPoint(p_ned, at_ray.roll, at_ray.pitch, at_ray.yaw, mount);
    ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
    const double horizontal_m = std::hypot(wall_ahead_m, lateral_left_m);
    EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(alpha), kAngleTol) << wall_azimuth_deg;
    EXPECT_NEAR(ray.range_m * std::cos(ray.elevation_rad), static_cast<float>(horizontal_m), kRangeTol)
      << wall_azimuth_deg;
    EXPECT_NEAR(
      ray.elevation_rad, static_cast<float>(std::atan2(wall_height_m, horizontal_m)), kAngleTol)
      << wall_azimuth_deg;
  }
}

}  // namespace

TEST(RayLevelerTest, LevelVehicleWallIsTheBaseline)
{
  expectWallEquivalentToLevelVehicle({0, 0, 0.4}, {0, 0, 0.4}, kZeroMount);
}

TEST(RayLevelerTest, PurePitchGivesTheSameWallAsALevelVehicle)
{
  expectWallEquivalentToLevelVehicle({0, 20.0 * kDeg, 0.4}, {0, 0, 0.4}, kZeroMount);
  expectWallEquivalentToLevelVehicle({0, -25.0 * kDeg, 0.4}, {0, 0, 0.4}, kZeroMount);
}

TEST(RayLevelerTest, PureRollGivesTheSameWallAsALevelVehicle)
{
  expectWallEquivalentToLevelVehicle({15.0 * kDeg, 0, 0.4}, {0, 0, 0.4}, kZeroMount);
  expectWallEquivalentToLevelVehicle({-30.0 * kDeg, 0, 0.4}, {0, 0, 0.4}, kZeroMount);
}

TEST(RayLevelerTest, RollPitchHeadingChangeAndMountTogetherGiveTheSameWall)
{
  // Different tilt at the reference time and at the ray time, the vehicle yawing during the frame,
  // the 10 degree mount fixture and a small mount roll and yaw. The reference heading is the
  // heading at the reference time.
  expectWallEquivalentToLevelVehicle(
    {12.0 * kDeg, -18.0 * kDeg, 0.4 + 0.15}, {-4.0 * kDeg, 6.0 * kDeg, 0.4},
    mountOf(0.02, kFixtureMountPitchDownRad, -0.03));
}

// --- Origin motion and interpolation ----------------------------------------------------------

TEST(RayLevelerTest, ReturnIsShiftedByTheOriginMotionSinceTheReference)
{
  // The vehicle is 1 m further north at the ray time. A point 5 m north of the reference
  // position is 4 m ahead of the vehicle at the ray time; seen from the reference origin it is
  // 5 m ahead again.
  const TwoPoseHistory poses{poseOf(0, 0, 0, {0.0, 0.0, 0.0}), poseOf(0, 0, 0, {1.0, 0.0, 0.0})};
  RayLeveler leveler{poses, configOf(kZeroMount, 2.0)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.0, 0.0);
  ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.range_m, 5.0F, kRangeTol);
  EXPECT_NEAR(ray.azimuth_rad, 0.0F, kAngleTol);
  // Moving east by 0.3 m: a point 4 m ahead is, from the reference origin, 0.3 m to the right.
  const TwoPoseHistory east{poseOf(0, 0, 0, {0.0, 0.0, 0.0}), poseOf(0, 0, 0, {0.0, 0.3, 0.0})};
  RayLeveler leveler_east{east, configOf(kZeroMount, 2.0)};
  ASSERT_EQ(leveler_east.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ahead = sensorReturn(4.0, 0.0, 0.0);
  ASSERT_EQ(leveler_east.level(ahead), LevelingStatus::kOk);
  EXPECT_NEAR(ahead.azimuth_rad, static_cast<float>(-std::atan2(0.3, 4.0)), kAngleTol);
  // Climbing by 0.3 m (NED z is -0.3): the point is 0.3 m below the reference origin's level.
  const TwoPoseHistory up{poseOf(0, 0, 0, {0.0, 0.0, 0.0}), poseOf(0, 0, 0, {0.0, 0.0, -0.3})};
  RayLeveler leveler_up{up, configOf(kZeroMount, 2.0)};
  ASSERT_EQ(leveler_up.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample level_ray = sensorReturn(4.0, 0.0, 0.0);
  ASSERT_EQ(leveler_up.level(level_ray), LevelingStatus::kOk);
  EXPECT_NEAR(level_ray.elevation_rad, static_cast<float>(std::atan2(0.3, 4.0)), kAngleTol);
}

TEST(RayLevelerTest, OriginMotionAboveTheThresholdDropsTheRay)
{
  const TwoPoseHistory over{poseOf(0, 0, 0),
    poseOf(0, 0, 0, {kFixtureOriginThresholdM + 0.01, 0.0, 0.0})};
  RayLeveler leveler{over, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  const RaySample before = ray;
  EXPECT_EQ(leveler.level(ray), LevelingStatus::kOriginMotionExceeded);
  EXPECT_EQ(ray.azimuth_rad, before.azimuth_rad);
  EXPECT_EQ(ray.range_m, before.range_m);
  EXPECT_TRUE(breaksFrameContinuity(LevelingStatus::kOriginMotionExceeded));

  // The distance is three-dimensional: a vertical move counts, and the compare is "above".
  const TwoPoseHistory vertical{poseOf(0, 0, 0), poseOf(0, 0, 0, {0.0, 0.0, -(kFixtureOriginThresholdM + 0.01)})};
  RayLeveler vertical_leveler{vertical, configOf(kZeroMount)};
  ASSERT_EQ(vertical_leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample v = sensorReturn(4.0, 0.0, 0.0);
  EXPECT_EQ(vertical_leveler.level(v), LevelingStatus::kOriginMotionExceeded);

  const TwoPoseHistory at_limit{poseOf(0, 0, 0), poseOf(0, 0, 0, {kFixtureOriginThresholdM, 0.0, 0.0})};
  RayLeveler limit_leveler{at_limit, configOf(kZeroMount)};
  ASSERT_EQ(limit_leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample l = sensorReturn(4.0, 0.0, 0.0);
  EXPECT_EQ(limit_leveler.level(l), LevelingStatus::kOk);
}

TEST(RayLevelerTest, NoReturnKeepsItsDirectionAndHasNoRangeButStillObeysTheOriginFilter)
{
  const TwoPoseHistory poses{poseOf(0, 0, 0), poseOf(0, 0, kPi / 2.0, {0.2, 0.0, 0.0})};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray{
    1'000U, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.1F, false,
    RayReturnKind::kQualifiedNoReturn};
  ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
  EXPECT_TRUE(std::isnan(ray.range_m));
  EXPECT_EQ(ray.return_kind, RayReturnKind::kQualifiedNoReturn);
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(-kPi / 2.0), kAngleTol);  // the heading turn
  EXPECT_NEAR(ray.elevation_rad, 0.1F, kAngleTol);

  const TwoPoseHistory far{poseOf(0, 0, 0), poseOf(0, 0, 0, {5.0, 0.0, 0.0})};
  RayLeveler far_leveler{far, configOf(kZeroMount)};
  ASSERT_EQ(far_leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample dropped{
    1'000U, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.1F, false,
    RayReturnKind::kQualifiedNoReturn};
  EXPECT_EQ(far_leveler.level(dropped), LevelingStatus::kOriginMotionExceeded);
}

TEST(RayLevelerTest, DeskewsWithTheInterpolatedPoseOfTheHistory)
{
  // From the reference time (t = 0.1 s) the vehicle turns from north to east in 100 ms while it
  // moves 2 m north. A ray at 150 ms sees yaw 45 degrees and 1 m of motion.
  PoseHistory history{PoseHistoryConfig{8U, 200U * kMs}};
  history.push(poseOf(0, 0, 0, {0.0, 0.0, 0.0}, 0U, 100U * kMs));
  history.push(poseOf(0, 0, kPi / 2.0, {2.0, 0.0, 0.0}, 0U, 200U * kMs));
  RayLeveler leveler{history, configOf(kZeroMount, 1.5)};
  ASSERT_EQ(leveler.beginFrame(100U * kMs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(5.0, 0.0, 0.0);
  ray.timestamp_ns = 150U * kMs;
  ASSERT_EQ(leveler.level(ray), LevelingStatus::kOk);
  const double x = 5.0 * std::cos(kPi / 4.0) + 1.0;
  const double y_left = -5.0 * std::sin(kPi / 4.0);  // east of the reference heading is right
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(std::atan2(y_left, x)), kAngleTol);
  EXPECT_NEAR(ray.range_m, static_cast<float>(std::hypot(x, y_left)), kRangeTol);
  EXPECT_EQ(ray.timestamp_ns, 150U * kMs);
}

TEST(RayLevelerTest, RaysOnTheExactBracketEdgesAreTransformed)
{
  PoseHistory history{PoseHistoryConfig{8U, 200U * kMs}};
  history.push(poseOf(0, 0, 0, {0.0, 0.0, 0.0}, 0U, 100U * kMs));
  history.push(poseOf(0, 0, 0, {0.3, 0.0, 0.0}, 0U, 200U * kMs));
  RayLeveler leveler{history, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(100U * kMs), LevelingStatus::kOk);  // reference on the first sample
  RaySample first = sensorReturn(5.0, 0.0, 0.0);
  first.timestamp_ns = 100U * kMs;
  EXPECT_EQ(leveler.level(first), LevelingStatus::kOk);
  EXPECT_NEAR(first.range_m, 5.0F, kRangeTol);
  RaySample last = sensorReturn(5.0, 0.0, 0.0);
  last.timestamp_ns = 200U * kMs;  // on the last sample
  EXPECT_EQ(leveler.level(last), LevelingStatus::kOk);
  EXPECT_NEAR(last.range_m, 5.3F, kRangeTol);
  RaySample after = sensorReturn(5.0, 0.0, 0.0);
  after.timestamp_ns = 200U * kMs + 1U;  // one nanosecond past the last sample
  EXPECT_EQ(leveler.level(after), LevelingStatus::kNoPose);
}

// --- Missing, invalid and reset poses ---------------------------------------------------------

TEST(RayLevelerTest, RayWithoutPoseIsDroppedAndReported)
{
  PoseHistory poses{fixtureHistoryConfig()};
  poses.push(at(poseOf(0, 0, 0), kRefNs));  // no sample after the reference: rays have no pose
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  const RaySample before = ray;
  EXPECT_EQ(leveler.level(ray), LevelingStatus::kNoPose);
  EXPECT_EQ(ray.azimuth_rad, before.azimuth_rad);
  EXPECT_EQ(ray.elevation_rad, before.elevation_rad);
  EXPECT_EQ(ray.range_m, before.range_m);
  EXPECT_TRUE(breaksFrameContinuity(LevelingStatus::kNoPose));
}

TEST(RayLevelerTest, NoPoseAtTheReferenceTimeFailsTheFrame)
{
  const PoseHistory poses{fixtureHistoryConfig()};  // empty: no pose at any time
  RayLeveler leveler{poses, configOf(kZeroMount)};
  EXPECT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kNoPose);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  EXPECT_EQ(leveler.level(ray), LevelingStatus::kNoReference);
  EXPECT_TRUE(breaksFrameContinuity(LevelingStatus::kNoReference));
}

TEST(RayLevelerTest, LevelBeforeBeginFrameHasNoReference)
{
  const ConstantPoseHistory poses{poseOf(0, 0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  EXPECT_EQ(leveler.level(ray), LevelingStatus::kNoReference);
}

TEST(RayLevelerTest, AFailedBeginFrameForgetsTheEarlierReference)
{
  PoseHistory poses{fixtureHistoryConfig()};
  poses.push(at(poseOf(0, 0, 0), kRefNs));  // no sample after the reference: rays have no pose
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  EXPECT_EQ(leveler.beginFrame(kRefNs + 1U), LevelingStatus::kNoPose);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  EXPECT_EQ(leveler.level(ray), LevelingStatus::kNoReference);
}

TEST(RayLevelerTest, ResetCounterChangeBetweenReferenceAndRayDropsTheRay)
{
  // Each lookup is valid, but a reset happened between the two times, so the position difference
  // means nothing.
  const TwoPoseHistory poses{poseOf(0, 0, 0, {0, 0, 0}, 3U), poseOf(0, 0, 0, {0, 0, 0}, 4U)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  EXPECT_EQ(leveler.level(ray), LevelingStatus::kResetSinceReference);
  EXPECT_TRUE(breaksFrameContinuity(LevelingStatus::kResetSinceReference));
}

TEST(RayLevelerTest, ResetCounterChangeInsideTheBracketOfTheHistoryDropsTheRay)
{
  PoseHistory history{PoseHistoryConfig{8U, 200U * kMs}};
  history.push(poseOf(0, 0, 0, {0, 0, 0}, 0U, 100U * kMs));
  history.push(poseOf(0, 0, 0, {0, 0, 0}, 0U, 150U * kMs));
  history.push(poseOf(0, 0, 0, {0, 0, 0}, 1U, 200U * kMs));
  RayLeveler leveler{history, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(100U * kMs), LevelingStatus::kOk);
  RaySample inside = sensorReturn(4.0, 0.0, 0.0);
  inside.timestamp_ns = 175U * kMs;  // bracket 150 ms (counter 0) and 200 ms (counter 1)
  EXPECT_EQ(leveler.level(inside), LevelingStatus::kNoPose);
  RaySample on_new = sensorReturn(4.0, 0.0, 0.0);
  on_new.timestamp_ns = 200U * kMs;  // on the first sample after the reset: counter differs
  EXPECT_EQ(leveler.level(on_new), LevelingStatus::kResetSinceReference);
}

TEST(RayLevelerTest, NaNQuaternionGivesNoPoseForTheRayAndForTheReference)
{
  PoseHistory history{PoseHistoryConfig{8U, 200U * kMs}};
  history.push(poseOf(0, 0, 0, {0, 0, 0}, 0U, 100U * kMs));
  PoseSample bad = poseOf(0, 0, 0, {0, 0, 0}, 0U, 150U * kMs);
  bad.q_wxyz[0] = std::numeric_limits<double>::quiet_NaN();
  history.push(bad);
  history.push(poseOf(0, 0, 0, {0, 0, 0}, 0U, 200U * kMs));
  RayLeveler leveler{history, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(100U * kMs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.0, 0.0);
  ray.timestamp_ns = 125U * kMs;
  EXPECT_EQ(leveler.level(ray), LevelingStatus::kNoPose);

  // The same with a NaN quaternion on both sides of the ray time.
  PoseSample nan_pose = poseOf(0, 0, 0);
  nan_pose.q_wxyz[2] = std::numeric_limits<double>::quiet_NaN();
  const TwoPoseHistory nan_ray_history{poseOf(0, 0, 0), nan_pose};
  RayLeveler nan_ray_leveler{nan_ray_history, configOf(kZeroMount)};
  ASSERT_EQ(nan_ray_leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample r = sensorReturn(4.0, 0.0, 0.0);
  EXPECT_EQ(nan_ray_leveler.level(r), LevelingStatus::kNoPose);
  // And NaN at the reference time fails the frame.
  const ConstantPoseHistory nan_reference{nan_pose};
  RayLeveler nan_leveler{nan_reference, configOf(kZeroMount)};
  EXPECT_EQ(nan_leveler.beginFrame(kRefNs), LevelingStatus::kNoPose);
}

TEST(RayLevelerTest, AttitudeWithAVerticalForwardAxisHasNoReferenceHeading)
{
  const ConstantPoseHistory poses{poseOf(0, kPi / 2.0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  EXPECT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kNoPose);
}

TEST(RayLevelerTest, InvalidRaysAreLeftUnchangedAndDoNotBreakContinuity)
{
  const ConstantPoseHistory poses{poseOf(0, 0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);

  RaySample no_time = sensorReturn(4.0, 0.2, 0.1);
  no_time.timestamp_ns = 0U;
  EXPECT_EQ(leveler.level(no_time), LevelingStatus::kInvalidRay);

  RaySample nan_azimuth = sensorReturn(4.0, std::numeric_limits<double>::quiet_NaN(), 0.1);
  EXPECT_EQ(leveler.level(nan_azimuth), LevelingStatus::kInvalidRay);

  RaySample nan_range = sensorReturn(std::numeric_limits<double>::quiet_NaN(), 0.2, 0.1);
  EXPECT_EQ(leveler.level(nan_range), LevelingStatus::kInvalidRay);

  RaySample invalid_kind = sensorReturn(4.0, 0.2, 0.1);
  invalid_kind.return_kind = RayReturnKind::kInvalid;
  const RaySample before = invalid_kind;
  EXPECT_EQ(leveler.level(invalid_kind), LevelingStatus::kInvalidRay);
  EXPECT_EQ(invalid_kind.azimuth_rad, before.azimuth_rad);

  EXPECT_FALSE(breaksFrameContinuity(LevelingStatus::kInvalidRay));
  EXPECT_FALSE(breaksFrameContinuity(LevelingStatus::kOk));
}
