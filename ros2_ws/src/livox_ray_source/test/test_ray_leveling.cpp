#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>
#include <stdexcept>

#include "livox_ray_source/gyro_track.hpp"
#include "livox_ray_source/pose_history.hpp"
#include "livox_ray_source/ray_leveling.hpp"

using livox_ray_source::breaksFrameContinuity;
using livox_ray_source::GyroSample;
using livox_ray_source::GyroTrack;
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
constexpr uint64_t kMs = 1'000'000U;
constexpr uint64_t kRefNs = 50U * kMs;  // the reference time of the fixtures
constexpr uint64_t kRayNs = 100U * kMs;  // the ray time of the helpers, 50 ms after the reference
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

LevelingConfig configOf(const Mount & mount, const Vec & lever_arm_flu_m = {0.0, 0.0, 0.0})
{
  return LevelingConfig{
    mount.roll_rad, mount.pitch_down_rad, mount.yaw_left_rad, lever_arm_flu_m};
}

// level() with the rise output ignored, for the tests that do not look at it.
LevelingStatus levelRay(const RayLeveler & leveler, RaySample & ray)
{
  double rise_m = 0.0;
  return leveler.level(ray, rise_m);
}

// Sensor FLU ray (azimuth, elevation, range) of a point given in the output frame convention:
// direct spherical conversion.
RaySample sensorReturn(double range_m, double azimuth_rad, double elevation_rad)
{
  return RaySample{
    kRayNs, static_cast<float>(range_m), static_cast<float>(azimuth_rad),
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

PoseSample at(PoseSample pose, uint64_t time_ns)
{
  pose.timestamp_sample_ns = time_ns;
  return pose;
}

PoseHistoryConfig fixtureHistoryConfig()
{
  return PoseHistoryConfig{8U, kFixtureMaxGapNs};
}

const PoseHistoryConfig kFixtureGyroConfig{64U, 20U * kMs};

Mat rotationOfQuaternion(const std::array<double, 4> & q)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
    {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
    {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

// Rotation vector (axis times angle) of a rotation matrix, for angles below pi.
Vec rotationVectorOf(const Mat & r)
{
  const double c = std::max(-1.0, std::min(1.0, 0.5 * (r[0][0] + r[1][1] + r[2][2] - 1.0)));
  const double angle = std::acos(c);
  if (angle < 1.0e-12) {
    return {0.0, 0.0, 0.0};
  }
  const double k = angle / (2.0 * std::sin(angle));
  return {k * (r[2][1] - r[1][2]), k * (r[0][2] - r[2][0]), k * (r[1][0] - r[0][1])};
}

// A gyro track that turns at a constant rate from before `from_ns` to after `to_ns`.
void pushConstantRate(GyroTrack & gyro, const Vec & rate_flu_rad_s, uint64_t from_ns, uint64_t to_ns)
{
  for (uint64_t t = from_ns - 10U * kMs; t <= to_ns + 10U * kMs; t += 5U * kMs) {
    GyroSample s;
    s.timestamp_ns = t;
    s.rate_flu_rad_s = rate_flu_rad_s;
    gyro.push(s);
  }
}

// The inputs of ADR 0023 for a vehicle that has the pose `at_reference` at the reference time and the pose
// `at_ray` at the ray time: the FMU pose at the reference time, with the velocity that moves the body from
// one position to the other, and a gyro that turns at the constant rate that takes the sensor from one
// attitude to the other (dR = (F M)^T R_ref^T R_ray (F M), written here independently of the code).
class Motion : public PoseHistory
{
public:
  Motion(
    const PoseSample & at_reference, const PoseSample & at_ray, const Mount & mount = Mount{0.0, 0.0, 0.0},
    uint64_t reference_ns = kRefNs, uint64_t ray_ns = kRayNs)
  : PoseHistory(fixtureHistoryConfig()), _gyro(kFixtureGyroConfig)
  {
    const double dt_s = static_cast<double>(ray_ns - reference_ns) * 1.0e-9;
    PoseSample reference = at(at_reference, reference_ns);
    for (std::size_t i = 0U; i < 3U; ++i) {
      reference.velocity_ned_m_s[i] = (at_ray.position_ned_m[i] - at_reference.position_ned_m[i]) / dt_s;
    }
    push(reference);
    const Mat f{{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};
    const Mat a = mul(f, mul(rz(mount.yaw_left_rad), mul(ry(mount.pitch_down_rad), rx(mount.roll_rad))));
    const Mat d_r = mul(
      transpose(a), mul(transpose(rotationOfQuaternion(at_reference.q_wxyz)),
      mul(rotationOfQuaternion(at_ray.q_wxyz), a)));
    const Vec angle = rotationVectorOf(d_r);
    pushConstantRate(_gyro, {angle[0] / dt_s, angle[1] / dt_s, angle[2] / dt_s}, reference_ns, ray_ns);
  }

  [[nodiscard]] const GyroTrack * gyro() const {return &_gyro;}

private:
  GyroTrack _gyro;
};

// The same pose at every time of the helpers.
class ConstantMotion : public Motion
{
public:
  explicit ConstantMotion(const PoseSample & pose)
  : Motion(pose, pose) {}
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
  EXPECT_TRUE(std::isnan(config.lever_arm_flu_m[0]));
  EXPECT_TRUE(std::isnan(config.lever_arm_flu_m[1]));
  EXPECT_TRUE(std::isnan(config.lever_arm_flu_m[2]));
  EXPECT_FALSE(config.isValid());
  const ConstantMotion poses{poseOf(0, 0, 0)};
  EXPECT_THROW((RayLeveler{poses, config, poses.gyro()}), std::invalid_argument);
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
  for (std::size_t i = 0U; i < 3U; ++i) {
    c = configOf(kZeroMount);
    c.lever_arm_flu_m[i] = nan;
    EXPECT_FALSE(c.isValid()) << i;
    c.lever_arm_flu_m[i] = inf;
    EXPECT_FALSE(c.isValid()) << i;
  }
  EXPECT_TRUE(configOf(kZeroMount, {0.12, 0.0, 0.14}).isValid());
}

// --- Frame conventions -----------------------------------------------------------------------

TEST(RayLevelerTest, ZeroPoseAndZeroMountReproduceTheFluRayDirectly)
{
  const ConstantMotion poses{poseOf(0, 0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  for (const double az : {-2.5, -1.0, 0.0, 0.7, 3.0}) {
    for (const double el : {-0.4, 0.0, 0.3}) {
      RaySample ray = sensorReturn(7.5, az, el);
      ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
      EXPECT_NEAR(ray.range_m, 7.5F, kRangeTol);
      EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(az), kAngleTol);
      EXPECT_NEAR(ray.elevation_rad, static_cast<float>(el), kAngleTol);
    }
  }
}

TEST(RayLevelerTest, TheTimestampTheMaskAndTheKindAreKept)
{
  const ConstantMotion poses{poseOf(0.1, -0.2, 0.3)};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(5.0, 0.2, 0.1);
  ray.timestamp_ns = kRayNs + 1'050U;
  ray.masked = true;
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
  EXPECT_EQ(ray.timestamp_ns, kRayNs + 1'050U);
  EXPECT_TRUE(ray.masked);
  EXPECT_EQ(ray.return_kind, RayReturnKind::kReturn);
}

TEST(RayLevelerTest, MountPitchedDownPutsTheBoresightBelowTheHorizonWithALevelVehicle)
{
  // Convention: positive pitch_down_rad tilts the sensor boresight (sensor +x) DOWN, so with a
  // level vehicle a ray along the boresight has an elevation of minus the mount angle. A ray at
  // +mount angle in the sensor frame is the body horizon.
  const ConstantMotion poses{poseOf(0, 0, 0)};
  RayLeveler leveler{poses, configOf(mountOf(0.0, kFixtureMountPitchDownRad, 0.0)), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);

  RaySample boresight = sensorReturn(10.0, 0.0, 0.0);
  ASSERT_EQ(levelRay(leveler, boresight), LevelingStatus::kOk);
  EXPECT_NEAR(boresight.elevation_rad, static_cast<float>(-kFixtureMountPitchDownRad), kAngleTol);
  EXPECT_NEAR(boresight.azimuth_rad, 0.0F, kAngleTol);

  RaySample horizon = sensorReturn(10.0, 0.0, kFixtureMountPitchDownRad);
  ASSERT_EQ(levelRay(leveler, horizon), LevelingStatus::kOk);
  EXPECT_NEAR(horizon.elevation_rad, 0.0F, kAngleTol);
  EXPECT_NEAR(horizon.azimuth_rad, 0.0F, kAngleTol);

  // Sideways the pitch of the mount does not change the elevation.
  RaySample left = sensorReturn(10.0, kPi / 2.0, 0.0);
  ASSERT_EQ(levelRay(leveler, left), LevelingStatus::kOk);
  EXPECT_NEAR(left.elevation_rad, 0.0F, kAngleTol);
  EXPECT_NEAR(left.azimuth_rad, static_cast<float>(kPi / 2.0), kAngleTol);

  // Behind the vehicle the boresight tilt is a rise: the sensor's rear ray points up.
  RaySample behind = sensorReturn(10.0, kPi, 0.0);
  ASSERT_EQ(levelRay(leveler, behind), LevelingStatus::kOk);
  EXPECT_NEAR(behind.elevation_rad, static_cast<float>(kFixtureMountPitchDownRad), kAngleTol);
}

TEST(RayLevelerTest, MountRollAndYawFollowTheirDocumentedSigns)
{
  // Roll about forward, positive = left side up: the sensor's left axis (+y) rises.
  {
    const ConstantMotion poses{poseOf(0, 0, 0)};
    RayLeveler leveler{poses, configOf(mountOf(0.1, 0.0, 0.0)), poses.gyro()};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample left = sensorReturn(10.0, kPi / 2.0, 0.0);
    ASSERT_EQ(levelRay(leveler, left), LevelingStatus::kOk);
    EXPECT_NEAR(left.elevation_rad, 0.1F, kAngleTol);
  }
  // Yaw about up, positive = boresight turned to the left.
  {
    const ConstantMotion poses{poseOf(0, 0, 0)};
    RayLeveler leveler{poses, configOf(mountOf(0.0, 0.0, 0.2)), poses.gyro()};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample boresight = sensorReturn(10.0, 0.0, 0.0);
    ASSERT_EQ(levelRay(leveler, boresight), LevelingStatus::kOk);
    EXPECT_NEAR(boresight.azimuth_rad, 0.2F, kAngleTol);
  }
}

TEST(RayLevelerTest, LateralAndVerticalSignsAreNotMirrored)
{
  // A point on the vehicle's left stays positive left, a point above stays positive up, for a
  // level vehicle with any heading (the heading is the reference heading too).
  for (const double yaw : {0.0, 0.5, -2.0, 3.0}) {
    const ConstantMotion poses{poseOf(0, 0, yaw)};
    RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample left = sensorReturn(4.0, kPi / 2.0, 0.0);
    RaySample right = sensorReturn(4.0, -kPi / 2.0, 0.0);
    RaySample up = sensorReturn(4.0, 0.0, 0.5);
    RaySample down = sensorReturn(4.0, 0.0, -0.5);
    ASSERT_EQ(levelRay(leveler, left), LevelingStatus::kOk);
    ASSERT_EQ(levelRay(leveler, right), LevelingStatus::kOk);
    ASSERT_EQ(levelRay(leveler, up), LevelingStatus::kOk);
    ASSERT_EQ(levelRay(leveler, down), LevelingStatus::kOk);
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
  const ConstantMotion poses{poseOf(roll, 0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = rayToNedPoint(p_ned, roll, 0.0, 0.0, kZeroMount);
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(kPi / 2.0), kAngleTol);
  EXPECT_NEAR(ray.elevation_rad, 0.0F, kAngleTol);
  EXPECT_NEAR(ray.range_m, 3.0F, kRangeTol);
}

TEST(RayLevelerTest, HeadingChangeSinceTheReferenceRotatesTheRay)
{
  // The vehicle faces east (yaw 90 degrees) at the ray time and north at the reference time. A
  // ray along the body forward axis points east, which is to the RIGHT of the reference heading:
  // azimuth minus 90 degrees (counter-clockwise positive).
  const Motion poses{poseOf(0, 0, 0), poseOf(0, 0, kPi / 2.0)};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(6.0, 0.0, 0.0);
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(-kPi / 2.0), kAngleTol);
  EXPECT_NEAR(ray.range_m, 6.0F, kRangeTol);
  // A ray to the body's left points north, which is straight ahead at the reference heading.
  RaySample left = sensorReturn(6.0, kPi / 2.0, 0.0);
  ASSERT_EQ(levelRay(leveler, left), LevelingStatus::kOk);
  EXPECT_NEAR(left.azimuth_rad, 0.0F, kAngleTol);
}

TEST(RayLevelerTest, ACommonHeadingOffsetOfBothPosesDoesNotMatter)
{
  const Motion poses{poseOf(0, 0, 1.0), poseOf(0, 0, 1.0)};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(6.0, 0.3, 0.1);
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
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
  const Motion poses{poseOf(at_reference.roll, at_reference.pitch, at_reference.yaw, vehicle),
    poseOf(at_ray.roll, at_ray.pitch, at_ray.yaw, vehicle), mount};
  RayLeveler leveler{poses, configOf(mount), poses.gyro()};
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
    ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
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
  const Motion poses{poseOf(0, 0, 0, {0.0, 0.0, 0.0}), poseOf(0, 0, 0, {1.0, 0.0, 0.0})};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.0, 0.0);
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.range_m, 5.0F, kRangeTol);
  EXPECT_NEAR(ray.azimuth_rad, 0.0F, kAngleTol);
  // Moving east by 0.3 m: a point 4 m ahead is, from the reference origin, 0.3 m to the right.
  const Motion east{poseOf(0, 0, 0, {0.0, 0.0, 0.0}), poseOf(0, 0, 0, {0.0, 0.3, 0.0})};
  RayLeveler leveler_east{east, configOf(kZeroMount), east.gyro()};
  ASSERT_EQ(leveler_east.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ahead = sensorReturn(4.0, 0.0, 0.0);
  ASSERT_EQ(levelRay(leveler_east, ahead), LevelingStatus::kOk);
  EXPECT_NEAR(ahead.azimuth_rad, static_cast<float>(-std::atan2(0.3, 4.0)), kAngleTol);
  // Climbing by 0.3 m (NED z is -0.3): the point is 0.3 m below the reference origin's level.
  const Motion up{poseOf(0, 0, 0, {0.0, 0.0, 0.0}), poseOf(0, 0, 0, {0.0, 0.0, -0.3})};
  RayLeveler leveler_up{up, configOf(kZeroMount), up.gyro()};
  ASSERT_EQ(leveler_up.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample level_ray = sensorReturn(4.0, 0.0, 0.0);
  ASSERT_EQ(levelRay(leveler_up, level_ray), LevelingStatus::kOk);
  EXPECT_NEAR(level_ray.elevation_rad, static_cast<float>(std::atan2(0.3, 4.0)), kAngleTol);
}

TEST(RayLevelerTest, NeitherReturnNorNoReturnIsDroppedForMotion)
{
  // 12 m/s over 100 ms is 1.2 m; here 4 m ahead and 4 m up: far above any threshold the old rule had.
  const double far_m = 4.0;
  const Motion poses{poseOf(0, 0, 0), poseOf(0, 0, 0, {far_m, 0.0, -far_m})};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.0, 0.0);
  double rise_m = 0.0;
  ASSERT_EQ(leveler.level(ray, rise_m), LevelingStatus::kOk);
  // The end point is 4 m ahead of the vehicle, which is far_m ahead of and far_m above the
  // reference origin: from the reference origin it is (4 + far, 0, +far).
  const double x = 4.0 + far_m;
  EXPECT_NEAR(ray.range_m, static_cast<float>(std::hypot(x, far_m)), 1.0e-4F);
  EXPECT_NEAR(ray.elevation_rad, static_cast<float>(std::atan2(far_m, x)), kAngleTol);
  EXPECT_NEAR(rise_m, far_m, 1.0e-9);

  RaySample no_return{
    kRayNs, std::numeric_limits<float>::quiet_NaN(), 0.2F, 0.1F, false,
    RayReturnKind::kQualifiedNoReturn};
  ASSERT_EQ(leveler.level(no_return, rise_m), LevelingStatus::kOk);
  EXPECT_NEAR(no_return.azimuth_rad, 0.2F, kAngleTol);  // direction only, nothing to shift
  EXPECT_NEAR(no_return.elevation_rad, 0.1F, kAngleTol);
  EXPECT_NEAR(rise_m, far_m, 1.0e-9);
}

TEST(RayLevelerTest, RiseIsPositiveWhenTheOriginIsHigherAndNegativeWhenLower)
{
  for (const double up_m : {0.3, -0.2, 0.0}) {
    const Motion poses{poseOf(0, 0, 0), poseOf(0, 0, 0, {1.0, 2.0, -up_m})};
    RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample ray = sensorReturn(4.0, 0.0, 0.0);
    double rise_m = 99.0;
    ASSERT_EQ(leveler.level(ray, rise_m), LevelingStatus::kOk);
    EXPECT_NEAR(rise_m, up_m, 1.0e-9) << up_m;
  }
}

TEST(RayLevelerTest, LeverArmMovesTheOriginWithTheAttitude)
{
  // Sensor 0.12 m ahead and 0.14 m above the body origin. The reference is level; at the ray time the
  // vehicle is pitched nose up by 10 degrees at the same position: the sensor is then higher by
  // 0.12 sin(10) + 0.14 cos(10) - 0.14 = 0.0187 m and 0.12 cos(10) - 0.14 sin(10) - 0.12 ahead.
  const double pitch = 10.0 * kDeg;
  const Motion poses{poseOf(0, 0, 0), poseOf(0, pitch, 0)};
  const Vec lever{0.12, 0.0, 0.14};
  RayLeveler leveler{poses, configOf(kZeroMount, lever), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.0, 0.0);
  double rise_m = 0.0;
  ASSERT_EQ(leveler.level(ray, rise_m), LevelingStatus::kOk);
  const double expected_rise = 0.12 * std::sin(pitch) + 0.14 * std::cos(pitch) - 0.14;
  EXPECT_NEAR(rise_m, expected_rise, 1.0e-9);

  // A pure rotation about the sensor itself would keep the sensor in place: with no lever arm the
  // origin is the body origin and nothing moves.
  RayLeveler no_lever{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(no_lever.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample r2 = sensorReturn(4.0, 0.0, 0.0);
  ASSERT_EQ(no_lever.level(r2, rise_m), LevelingStatus::kOk);
  EXPECT_NEAR(rise_m, 0.0, 1.0e-12);
}

TEST(RayLevelerTest, ReturnWhoseEndPointFallsOnTheReferenceOriginIsDroppedNotKept)
{
  // The vehicle is 0.25 m behind its reference position at the ray time and the return is 0.25 m
  // ahead: seen from the reference origin the range is 0. The sector map rejects such a ray, so
  // the transform must not report it as usable (it would be lost without breaking continuity and
  // another ray could then certify the sector clear). It is reported as a dropped ray.
  const Motion poses{poseOf(0, 0, 0), poseOf(0, 0, 0, {-0.25, 0.0, 0.0})};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(0.25, 0.0, 0.0);
  const RaySample before = ray;
  EXPECT_EQ(levelRay(leveler, ray), LevelingStatus::kDegenerateReturn);
  EXPECT_TRUE(breaksFrameContinuity(LevelingStatus::kDegenerateReturn));
  EXPECT_EQ(ray.range_m, before.range_m);
  EXPECT_EQ(ray.azimuth_rad, before.azimuth_rad);
  EXPECT_EQ(ray.elevation_rad, before.elevation_rad);
}

TEST(RayLevelerTest, ReturnTooCloseToTheReferenceOriginStaysAValidReturn)
{
  // A tiny positive range, also below the minimum distance of the sector map, is a valid return
  // (the map reports it as too close); only a range of 0 is dropped.
  for (const double remaining_m : {1.0e-4, 0.05, 0.5 * obstacle_sector_map::kMinDistanceM}) {
    const Motion poses{poseOf(0, 0, 0), poseOf(0, 0, 0, {-0.25, 0.0, 0.0})};
    RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample ray = sensorReturn(0.25 + remaining_m, 0.0, 0.0);
    ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk) << remaining_m;
    EXPECT_NEAR(ray.range_m, static_cast<float>(remaining_m), 1.0e-6F);
    EXPECT_GT(ray.range_m, 0.0F);
    EXPECT_LT(ray.range_m, obstacle_sector_map::kMinDistanceM);
    EXPECT_EQ(ray.return_kind, RayReturnKind::kReturn);
  }
}

TEST(RayLevelerTest, NoReturnKeepsItsDirectionAndHasNoRange)
{
  const Motion poses{poseOf(0, 0, 0), poseOf(0, 0, kPi / 2.0, {0.2, 0.0, 0.0})};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray{
    kRayNs, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.1F, false,
    RayReturnKind::kQualifiedNoReturn};
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
  EXPECT_TRUE(std::isnan(ray.range_m));
  EXPECT_EQ(ray.return_kind, RayReturnKind::kQualifiedNoReturn);
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(-kPi / 2.0), kAngleTol);  // the heading turn
  EXPECT_NEAR(ray.elevation_rad, 0.1F, kAngleTol);

}

TEST(RayLevelerTest, DeskewsWithTheGyroAndTheReferenceVelocity)
{
  // From the reference time (t = 0.1 s) the vehicle turns from north towards east at 900 deg/s while it
  // moves north at 20 m/s. A ray at 150 ms sees yaw 45 degrees and 1 m of motion.
  const Motion motion{poseOf(0, 0, 0, {0.0, 0.0, 0.0}), poseOf(0, 0, kPi / 4.0, {1.0, 0.0, 0.0}),
    kZeroMount, 100U * kMs, 150U * kMs};
  RayLeveler leveler{motion, configOf(kZeroMount), motion.gyro()};
  ASSERT_EQ(leveler.beginFrame(100U * kMs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(5.0, 0.0, 0.0);
  ray.timestamp_ns = 150U * kMs;
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
  const double x = 5.0 * std::cos(kPi / 4.0) + 1.0;
  const double y_left = -5.0 * std::sin(kPi / 4.0);  // east of the reference heading is right
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(std::atan2(y_left, x)), kAngleTol);
  EXPECT_NEAR(ray.range_m, static_cast<float>(std::hypot(x, y_left)), kRangeTol);
  EXPECT_EQ(ray.timestamp_ns, 150U * kMs);
}

TEST(RayLevelerTest, PitchAndYawInsideTheFrameHaveTheGyroSign)
{
  // Written from the sensor rates directly, not from the fixture's matrices: with a zero mount and a level
  // reference, a negative rate about the sensor's left axis lifts the boresight (nose up), and a positive
  // rate about up turns it to the left (ADR 0023: writing the rule without F would flip both).
  const double angle = 10.0 * kDeg;
  const double dt_s = 0.05;
  for (const int axis : {1, 2}) {
    PoseHistory poses{fixtureHistoryConfig()};
    PoseSample reference = at(poseOf(0, 0, 0), kRefNs);
    reference.velocity_ned_m_s = {0.0, 0.0, 0.0};
    poses.push(reference);
    GyroTrack gyro{kFixtureGyroConfig};
    Vec rate{0.0, 0.0, 0.0};
    rate[static_cast<std::size_t>(axis)] = (axis == 1 ? -angle : angle) / dt_s;
    pushConstantRate(gyro, rate, kRefNs, kRayNs);
    RayLeveler leveler{poses, configOf(kZeroMount), &gyro};
    ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
    RaySample boresight = sensorReturn(10.0, 0.0, 0.0);
    ASSERT_EQ(levelRay(leveler, boresight), LevelingStatus::kOk);
    if (axis == 1) {
      EXPECT_NEAR(boresight.elevation_rad, static_cast<float>(angle), kAngleTol);  // nose up: the ray rises
      EXPECT_NEAR(boresight.azimuth_rad, 0.0F, kAngleTol);
    } else {
      EXPECT_NEAR(boresight.azimuth_rad, static_cast<float>(angle), kAngleTol);  // turned left
      EXPECT_NEAR(boresight.elevation_rad, 0.0F, kAngleTol);
    }
  }
}

TEST(RayLevelerTest, ARayAtTheReferenceTimeNeedsNoGyroAndNoVelocity)
{
  // The Gazebo cloud stamps every ray of a frame with the reference time (ADR 0023 decision 6).
  PoseHistory poses{fixtureHistoryConfig()};
  poses.push(at(poseOf(0, 20.0 * kDeg, 0), kRefNs));  // velocity unknown (NaN)
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = rayToNedPoint({5.0, 0.0, -0.7}, 0.0, 20.0 * kDeg, 0.0, kZeroMount);
  ray.timestamp_ns = kRefNs;
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.elevation_rad, static_cast<float>(std::atan2(0.7, 5.0)), kAngleTol);
}

// --- Missing, invalid and reset poses ---------------------------------------------------------

TEST(RayLevelerTest, ARayAwayFromTheReferenceWithoutGyroOrVelocityIsDropped)
{
  PoseHistory poses{fixtureHistoryConfig()};
  PoseSample reference = at(poseOf(0, 0, 0), kRefNs);
  poses.push(reference);
  // No gyro.
  RayLeveler no_gyro{poses, configOf(kZeroMount)};
  ASSERT_EQ(no_gyro.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  const RaySample before = ray;
  EXPECT_EQ(levelRay(no_gyro, ray), LevelingStatus::kNoPose);
  EXPECT_EQ(ray.azimuth_rad, before.azimuth_rad);
  EXPECT_EQ(ray.elevation_rad, before.elevation_rad);
  EXPECT_EQ(ray.range_m, before.range_m);
  EXPECT_TRUE(breaksFrameContinuity(LevelingStatus::kNoPose));
  // A gyro, but the FMU velocity at the reference is unknown.
  GyroTrack gyro{kFixtureGyroConfig};
  pushConstantRate(gyro, {0.0, 0.0, 0.0}, kRefNs, kRayNs);
  RayLeveler no_velocity{poses, configOf(kZeroMount), &gyro};
  ASSERT_EQ(no_velocity.beginFrame(kRefNs), LevelingStatus::kOk);
  EXPECT_EQ(levelRay(no_velocity, ray), LevelingStatus::kNoPose);
  // A gyro that does not reach the ray time (beyond its tail).
  PoseHistory with_velocity{fixtureHistoryConfig()};
  reference.velocity_ned_m_s = {0.0, 0.0, 0.0};
  with_velocity.push(reference);
  GyroTrack short_gyro{kFixtureGyroConfig};
  pushConstantRate(short_gyro, {0.0, 0.0, 0.0}, kRefNs, kRefNs + 10U * kMs);  // last sample at 70 ms
  RayLeveler short_leveler{with_velocity, configOf(kZeroMount), &short_gyro};
  ASSERT_EQ(short_leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  EXPECT_EQ(levelRay(short_leveler, ray), LevelingStatus::kNoPose);  // 100 ms: 30 ms past the newest
}

TEST(RayLevelerTest, NoPoseAtTheReferenceTimeFailsTheFrame)
{
  const PoseHistory poses{fixtureHistoryConfig()};  // empty: no pose at any time
  RayLeveler leveler{poses, configOf(kZeroMount)};
  EXPECT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kNoPose);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  EXPECT_EQ(levelRay(leveler, ray), LevelingStatus::kNoReference);
  EXPECT_TRUE(breaksFrameContinuity(LevelingStatus::kNoReference));
}

TEST(RayLevelerTest, LevelBeforeBeginFrameHasNoReference)
{
  const ConstantMotion poses{poseOf(0, 0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  EXPECT_EQ(levelRay(leveler, ray), LevelingStatus::kNoReference);
}

TEST(RayLevelerTest, AFailedBeginFrameForgetsTheEarlierReference)
{
  PoseHistory poses{fixtureHistoryConfig()};
  poses.push(at(poseOf(0, 0, 0), kRefNs));  // no sample after the reference
  RayLeveler leveler{poses, configOf(kZeroMount)};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  EXPECT_EQ(leveler.beginFrame(kRefNs + 1U), LevelingStatus::kNoPose);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  ray.timestamp_ns = kRefNs + 1U;
  EXPECT_EQ(levelRay(leveler, ray), LevelingStatus::kNoReference);
}

TEST(RayLevelerTest, AGyroRestartBetweenReferenceAndRayDropsTheRay)
{
  // The gyro lost samples (a gap above its limit) between the two times: the two orientations belong to two
  // integrations and cannot be combined.
  PoseHistory poses{fixtureHistoryConfig()};
  PoseSample reference = at(poseOf(0, 0, 0), kRefNs);
  reference.velocity_ned_m_s = {0.0, 0.0, 0.0};
  poses.push(reference);
  GyroTrack gyro{kFixtureGyroConfig};
  pushConstantRate(gyro, {0.0, 0.0, 1.0}, kRefNs, kRefNs + 5U * kMs);     // up to 65 ms
  pushConstantRate(gyro, {0.0, 0.0, 1.0}, kRayNs, kRayNs);                // from 90 ms: 25 ms gap
  RayLeveler leveler{poses, configOf(kZeroMount), &gyro};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  EXPECT_EQ(levelRay(leveler, ray), LevelingStatus::kResetSinceReference);
  EXPECT_TRUE(breaksFrameContinuity(LevelingStatus::kResetSinceReference));
  RaySample in_gap = sensorReturn(4.0, 0.2, 0.1);
  in_gap.timestamp_ns = 80U * kMs;  // bracket 65 ms (old segment) and 90 ms (new segment)
  EXPECT_EQ(levelRay(leveler, in_gap), LevelingStatus::kNoPose);
}

TEST(RayLevelerTest, TheFmuPoseIsUsedOnlyAtTheReferenceTime)
{
  // ADR 0023: a reset or a NaN sample of the FMU after the reference does not change the rays, which are
  // expressed relative to the reference time.
  PoseHistory poses{fixtureHistoryConfig()};
  PoseSample reference = at(poseOf(0, 0, 0, {0, 0, 0}, 3U), kRefNs);
  reference.velocity_ned_m_s = {0.0, 0.0, 0.0};
  poses.push(reference);
  PoseSample nan_later = at(poseOf(0, 0, 0, {9, 9, 9}, 4U), 80U * kMs);
  nan_later.q_wxyz[0] = std::numeric_limits<double>::quiet_NaN();
  poses.push(nan_later);
  poses.push(at(poseOf(0, 1.0, 2.0, {9, 9, 9}, 4U), 120U * kMs));
  GyroTrack gyro{kFixtureGyroConfig};
  pushConstantRate(gyro, {0.0, 0.0, 0.0}, kRefNs, kRayNs);
  RayLeveler leveler{poses, configOf(kZeroMount), &gyro};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);
  RaySample ray = sensorReturn(4.0, 0.2, 0.1);
  ASSERT_EQ(levelRay(leveler, ray), LevelingStatus::kOk);
  EXPECT_NEAR(ray.azimuth_rad, 0.2F, kAngleTol);
  EXPECT_NEAR(ray.elevation_rad, 0.1F, kAngleTol);
  EXPECT_NEAR(ray.range_m, 4.0F, kRangeTol);
}

TEST(RayLevelerTest, NaNQuaternionAtTheReferenceFailsTheFrame)
{
  PoseSample nan_pose = poseOf(0, 0, 0);
  nan_pose.q_wxyz[2] = std::numeric_limits<double>::quiet_NaN();
  const ConstantMotion nan_reference{nan_pose};
  RayLeveler nan_leveler{nan_reference, configOf(kZeroMount), nan_reference.gyro()};
  EXPECT_EQ(nan_leveler.beginFrame(kRefNs), LevelingStatus::kNoPose);
}

TEST(RayLevelerTest, AttitudeWithAVerticalForwardAxisHasNoReferenceHeading)
{
  const ConstantMotion poses{poseOf(0, kPi / 2.0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  EXPECT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kNoPose);
}

TEST(RayLevelerTest, InvalidRaysAreLeftUnchangedAndDoNotBreakContinuity)
{
  const ConstantMotion poses{poseOf(0, 0, 0)};
  RayLeveler leveler{poses, configOf(kZeroMount), poses.gyro()};
  ASSERT_EQ(leveler.beginFrame(kRefNs), LevelingStatus::kOk);

  RaySample no_time = sensorReturn(4.0, 0.2, 0.1);
  no_time.timestamp_ns = 0U;
  EXPECT_EQ(levelRay(leveler, no_time), LevelingStatus::kInvalidRay);

  RaySample nan_azimuth = sensorReturn(4.0, std::numeric_limits<double>::quiet_NaN(), 0.1);
  EXPECT_EQ(levelRay(leveler, nan_azimuth), LevelingStatus::kInvalidRay);

  RaySample nan_range = sensorReturn(std::numeric_limits<double>::quiet_NaN(), 0.2, 0.1);
  EXPECT_EQ(levelRay(leveler, nan_range), LevelingStatus::kInvalidRay);

  // A zero or negative range is malformed (the sector map rejects it); the transform must not
  // turn it into a valid positive range.
  for (const double bad_range_m : {0.0, -50.0}) {
    RaySample bad_range = sensorReturn(bad_range_m, 0.2, 0.1);
    const RaySample bad_before = bad_range;
    EXPECT_EQ(levelRay(leveler, bad_range), LevelingStatus::kInvalidRay) << bad_range_m;
    EXPECT_EQ(bad_range.range_m, bad_before.range_m);
    EXPECT_EQ(bad_range.azimuth_rad, bad_before.azimuth_rad);
  }

  RaySample invalid_kind = sensorReturn(4.0, 0.2, 0.1);
  invalid_kind.return_kind = RayReturnKind::kInvalid;
  const RaySample before = invalid_kind;
  EXPECT_EQ(levelRay(leveler, invalid_kind), LevelingStatus::kInvalidRay);
  EXPECT_EQ(invalid_kind.azimuth_rad, before.azimuth_rad);

  EXPECT_FALSE(breaksFrameContinuity(LevelingStatus::kInvalidRay));
  EXPECT_FALSE(breaksFrameContinuity(LevelingStatus::kOk));
}
