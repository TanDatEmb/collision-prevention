#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include "livox_ray_source/self_mask.hpp"

using livox_ray_source::loadSelfMask;
using livox_ray_source::MaskCylinder;
using livox_ray_source::SelfMask;
using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;

namespace
{

// Test fixture geometry, not an approved platform value.
constexpr MaskCylinder kCylinder{1.0, 0.0, 0.2, -0.1, 0.1};
constexpr double kMaxDistanceM = 40.0;
// A sensor at the body origin with no rotation: the body frame is the sensor frame.
const livox_ray_source::LevelingConfig kAtSensor{0.0, 0.0, 0.0, {0.0, 0.0, 0.0}};
// The X500 bench mount of config/platforms/x500.yaml: 10 degrees down, sensor at (0.12, 0, 0.14) m.
const livox_ray_source::LevelingConfig kX500Mount{0.0, 0.1745, 0.0, {0.12, 0.0, 0.14}};

// The sensor-frame ray that ends at the body point p (inverse of the mount).
RaySample rayTo(
  const livox_ray_source::LevelingConfig & mount, const std::array<double, 3> & p, bool is_return)
{
  const auto r = livox_ray_source::mountRotation(mount);
  const double bx = p[0] - mount.lever_arm_flu_m[0];
  const double by = p[1] - mount.lever_arm_flu_m[1];
  const double bz = p[2] - mount.lever_arm_flu_m[2];
  const double sx = r[0] * bx + r[3] * by + r[6] * bz;  // R^T (p - L)
  const double sy = r[1] * bx + r[4] * by + r[7] * bz;
  const double sz = r[2] * bx + r[5] * by + r[8] * bz;
  const double range = std::sqrt(sx * sx + sy * sy + sz * sz);
  RaySample ray{
    1'000U,
    static_cast<float>(range),
    static_cast<float>(std::atan2(sy, sx)),
    static_cast<float>(std::asin(sz / range)),
    false,
    RayReturnKind::kReturn};
  if (!is_return) {
    ray.range_m = std::numeric_limits<float>::quiet_NaN();
    ray.return_kind = RayReturnKind::kQualifiedNoReturn;
  }
  return ray;
}

RaySample makeReturn(float range_m, float azimuth_rad, float elevation_rad)
{
  return RaySample{1'000U, range_m, azimuth_rad, elevation_rad, false, RayReturnKind::kReturn};
}

RaySample makeNoReturn(float azimuth_rad, float elevation_rad)
{
  return RaySample{1'000U, std::numeric_limits<float>::quiet_NaN(), azimuth_rad, elevation_rad,
                   false,  RayReturnKind::kQualifiedNoReturn};
}

std::string writeYaml(const std::string & name, const std::string & text)
{
  const auto path =
    std::filesystem::temp_directory_path() / ("self_mask_fixture_" + name + ".yaml");
  std::ofstream(path) << text;
  return path.string();
}

}  // namespace

TEST(SelfMaskTest, ReturnIsMaskedByItsEndPoint)
{
  const SelfMask mask{{kCylinder}, kAtSensor};
  EXPECT_TRUE(mask.isMasked(makeReturn(1.0F, 0.0F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeReturn(0.5F, 0.0F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeReturn(1.5F, 0.0F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeReturn(1.0F, 0.5F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskTest, ReturnAboveTheZRangeIsNotMasked)
{
  const SelfMask mask{{kCylinder}, kAtSensor};
  // Inside the radius horizontally (x = 0.877) but z = 0.479 is above z_max.
  EXPECT_FALSE(mask.isMasked(makeReturn(1.0F, 0.0F, 0.5F), kMaxDistanceM));
}

TEST(SelfMaskTest, ReturnBeyondTheCylinderIsNotMaskedEvenThoughItsRayCrosses)
{
  const SelfMask mask{{kCylinder}, kAtSensor};
  EXPECT_FALSE(mask.isMasked(makeReturn(3.0F, 0.0F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskTest, NoReturnIsMaskedWhenItsRaySegmentCrossesACylinder)
{
  const SelfMask mask{{kCylinder}, kAtSensor};
  EXPECT_TRUE(mask.isMasked(makeNoReturn(0.0F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeNoReturn(0.5F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeNoReturn(3.14159265F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskTest, NoReturnPassingAboveTheCylinderIsNotMasked)
{
  const SelfMask mask{{kCylinder}, kAtSensor};
  EXPECT_FALSE(mask.isMasked(makeNoReturn(0.0F, 0.5F), kMaxDistanceM));
}

TEST(SelfMaskTest, NoReturnSegmentEndingBeforeTheCylinderIsNotMasked)
{
  const SelfMask mask{{kCylinder}, kAtSensor};
  EXPECT_FALSE(mask.isMasked(makeNoReturn(0.0F, 0.0F), 0.5));
  EXPECT_TRUE(mask.isMasked(makeNoReturn(0.0F, 0.0F), 1.0));
}

TEST(SelfMaskTest, NoReturnFromInsideACylinderIsMasked)
{
  const SelfMask mask{{MaskCylinder{0.0, 0.0, 0.3, -0.5, 0.5}}, kAtSensor};
  EXPECT_TRUE(mask.isMasked(makeNoReturn(0.0F, 1.5F), kMaxDistanceM));
  EXPECT_TRUE(mask.isMasked(makeNoReturn(2.0F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskTest, InvalidRayKindsAndNonFiniteInputsAreNotMasked)
{
  const SelfMask mask{{kCylinder}, kAtSensor};
  auto invalid = makeNoReturn(0.0F, 0.0F);
  invalid.return_kind = RayReturnKind::kInvalid;
  EXPECT_FALSE(mask.isMasked(invalid, kMaxDistanceM));
  EXPECT_FALSE(
    mask.isMasked(makeNoReturn(std::numeric_limits<float>::infinity(), 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeNoReturn(0.0F, 0.0F), std::numeric_limits<double>::quiet_NaN()));
}

TEST(SelfMaskTest, ApplySetsMaskedAndNeverClearsIt)
{
  const SelfMask mask{{kCylinder}, kAtSensor};

  auto crossing = makeNoReturn(0.0F, 0.0F);
  mask.apply(crossing, kMaxDistanceM);
  EXPECT_TRUE(crossing.masked);

  auto already = makeReturn(1.5F, 0.0F, 0.0F);
  already.masked = true;
  mask.apply(already, kMaxDistanceM);
  EXPECT_TRUE(already.masked);

  auto clear = makeReturn(1.5F, 0.0F, 0.0F);
  mask.apply(clear, kMaxDistanceM);
  EXPECT_FALSE(clear.masked);
}

TEST(SelfMaskTest, RejectsInvalidCylinders)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SelfMask({}, kAtSensor), std::invalid_argument);
  EXPECT_THROW(
    SelfMask({MaskCylinder{0.0, 0.0, 0.0, -0.1, 0.1}}, kAtSensor), std::invalid_argument);
  EXPECT_THROW(SelfMask({MaskCylinder{0.0, 0.0, 0.2, 0.1, 0.1}}, kAtSensor), std::invalid_argument);
  EXPECT_THROW(
    SelfMask({MaskCylinder{nan, 0.0, 0.2, -0.1, 0.1}}, kAtSensor), std::invalid_argument);
}

TEST(SelfMaskLoaderTest, LoadsCylindersFromYaml)
{
  const auto path = writeYaml(
    "valid",
    "self_mask:\n  cylinders:\n    - center_m: [1.0, 0.0]\n      radius_m: 0.2\n"
    "      z_min_m: -0.1\n      z_max_m: 0.1\n");
  const auto mask = loadSelfMask(path, kAtSensor);
  EXPECT_TRUE(mask.isMasked(makeReturn(1.0F, 0.0F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskLoaderTest, RejectsNullMissingEmptyAndNonNumericValues)
{
  EXPECT_THROW(
    loadSelfMask(
      writeYaml(
        "null",
        "self_mask:\n  cylinders:\n    - center_m: [null, null]\n      radius_m: null\n"
        "      z_min_m: null\n      z_max_m: null\n"),
      kAtSensor),
    std::invalid_argument);
  EXPECT_THROW(
    loadSelfMask(
      writeYaml(
        "missing", "self_mask:\n  cylinders:\n    - center_m: [1.0, 0.0]\n      radius_m: 0.2\n"),
      kAtSensor),
    std::invalid_argument);
  EXPECT_THROW(
    loadSelfMask(writeYaml("empty", "self_mask:\n  cylinders: []\n"), kAtSensor),
    std::invalid_argument);
  EXPECT_THROW(
    loadSelfMask(writeYaml("nolist", "sensor:\n  model: x\n"), kAtSensor), std::invalid_argument);
  EXPECT_THROW(
    loadSelfMask(
      writeYaml(
        "text",
        "self_mask:\n  cylinders:\n    - center_m: [1.0, 0.0]\n      radius_m: wide\n"
        "      z_min_m: -0.1\n      z_max_m: 0.1\n"),
      kAtSensor),
    std::invalid_argument);
  EXPECT_THROW(
    loadSelfMask(
      writeYaml(
        "threevalues",
        "self_mask:\n  cylinders:\n    - center_m: [1.0, 0.0, 0.0]\n      radius_m: 0.2\n"
        "      z_min_m: -0.1\n      z_max_m: 0.1\n"),
      kAtSensor),
    std::invalid_argument);
  EXPECT_THROW(loadSelfMask("/nonexistent/self_mask.yaml", kAtSensor), std::invalid_argument);
}

TEST(SelfMaskBodyFrameTest, TheMountCarriesTheRayIntoTheBodyFrame)
{
  // a propeller disc of the X500 (OPEN-046), body frame
  const MaskCylinder prop{0.174, -0.174, 0.1565, 0.034, 0.0794};
  const SelfMask body{{prop}, kX500Mount};
  const SelfMask sensor_only{{prop}, kAtSensor};

  const auto at_prop = rayTo(kX500Mount, {0.174, -0.174, 0.05}, true);
  EXPECT_TRUE(body.isMasked(at_prop, kMaxDistanceM));
  // the same sensor-frame ray read without the mount ends 0.09 m below the sensor: not in the disc
  EXPECT_FALSE(sensor_only.isMasked(at_prop, kMaxDistanceM));

  // a no-return toward the disc crosses it; one level ahead passes above it
  EXPECT_TRUE(body.isMasked(rayTo(kX500Mount, {0.174, -0.174, 0.05}, false), kMaxDistanceM));
  EXPECT_FALSE(body.isMasked(rayTo(kX500Mount, {5.0, 0.0, 0.14}, false), kMaxDistanceM));
  // a return beyond the disc on the same bearing is not the airframe
  EXPECT_FALSE(body.isMasked(rayTo(kX500Mount, {2.0, -2.0, -0.3}, true), kMaxDistanceM));
}

TEST(SelfMaskBodyFrameTest, AMountThatIsNotValidIsRefused)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    SelfMask({kCylinder}, livox_ray_source::LevelingConfig{0.0, nan, 0.0, {0.0, 0.0, 0.0}}),
    std::invalid_argument);
  EXPECT_THROW(SelfMask({kCylinder}, livox_ray_source::LevelingConfig{}), std::invalid_argument);
}

TEST(SelfMaskLoaderTest, RepositoryPlatformConfigHoldsTheFourPropellerCylinders)
{
  // the values decided on 2026-11-22 (OPEN-046), with the mount of the same file
  const auto mask = loadSelfMask(X500_YAML_PATH, kX500Mount);
  for (const double x : {0.174, -0.174}) {
    for (const double y : {0.174, -0.174}) {
      EXPECT_TRUE(mask.isMasked(rayTo(kX500Mount, {x, y, 0.05}, true), kMaxDistanceM))
        << x << " " << y;
    }
  }
  EXPECT_FALSE(mask.isMasked(rayTo(kX500Mount, {2.0, 0.0, 0.0}, true), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(rayTo(kX500Mount, {0.174, -0.174, 0.2}, true), kMaxDistanceM));
}
