#include <gtest/gtest.h>

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
  const SelfMask mask{{kCylinder}};
  EXPECT_TRUE(mask.isMasked(makeReturn(1.0F, 0.0F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeReturn(0.5F, 0.0F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeReturn(1.5F, 0.0F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeReturn(1.0F, 0.5F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskTest, ReturnAboveTheZRangeIsNotMasked)
{
  const SelfMask mask{{kCylinder}};
  // Inside the radius horizontally (x = 0.877) but z = 0.479 is above z_max.
  EXPECT_FALSE(mask.isMasked(makeReturn(1.0F, 0.0F, 0.5F), kMaxDistanceM));
}

TEST(SelfMaskTest, ReturnBeyondTheCylinderIsNotMaskedEvenThoughItsRayCrosses)
{
  const SelfMask mask{{kCylinder}};
  EXPECT_FALSE(mask.isMasked(makeReturn(3.0F, 0.0F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskTest, NoReturnIsMaskedWhenItsRaySegmentCrossesACylinder)
{
  const SelfMask mask{{kCylinder}};
  EXPECT_TRUE(mask.isMasked(makeNoReturn(0.0F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeNoReturn(0.5F, 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeNoReturn(3.14159265F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskTest, NoReturnPassingAboveTheCylinderIsNotMasked)
{
  const SelfMask mask{{kCylinder}};
  EXPECT_FALSE(mask.isMasked(makeNoReturn(0.0F, 0.5F), kMaxDistanceM));
}

TEST(SelfMaskTest, NoReturnSegmentEndingBeforeTheCylinderIsNotMasked)
{
  const SelfMask mask{{kCylinder}};
  EXPECT_FALSE(mask.isMasked(makeNoReturn(0.0F, 0.0F), 0.5));
  EXPECT_TRUE(mask.isMasked(makeNoReturn(0.0F, 0.0F), 1.0));
}

TEST(SelfMaskTest, NoReturnFromInsideACylinderIsMasked)
{
  const SelfMask mask{{MaskCylinder{0.0, 0.0, 0.3, -0.5, 0.5}}};
  EXPECT_TRUE(mask.isMasked(makeNoReturn(0.0F, 1.5F), kMaxDistanceM));
  EXPECT_TRUE(mask.isMasked(makeNoReturn(2.0F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskTest, InvalidRayKindsAndNonFiniteInputsAreNotMasked)
{
  const SelfMask mask{{kCylinder}};
  auto invalid = makeNoReturn(0.0F, 0.0F);
  invalid.return_kind = RayReturnKind::kInvalid;
  EXPECT_FALSE(mask.isMasked(invalid, kMaxDistanceM));
  EXPECT_FALSE(
    mask.isMasked(makeNoReturn(std::numeric_limits<float>::infinity(), 0.0F), kMaxDistanceM));
  EXPECT_FALSE(mask.isMasked(makeNoReturn(0.0F, 0.0F), std::numeric_limits<double>::quiet_NaN()));
}

TEST(SelfMaskTest, ApplySetsMaskedAndNeverClearsIt)
{
  const SelfMask mask{{kCylinder}};

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
  EXPECT_THROW(SelfMask({}), std::invalid_argument);
  EXPECT_THROW(SelfMask({MaskCylinder{0.0, 0.0, 0.0, -0.1, 0.1}}), std::invalid_argument);
  EXPECT_THROW(SelfMask({MaskCylinder{0.0, 0.0, 0.2, 0.1, 0.1}}), std::invalid_argument);
  EXPECT_THROW(SelfMask({MaskCylinder{nan, 0.0, 0.2, -0.1, 0.1}}), std::invalid_argument);
}

TEST(SelfMaskLoaderTest, LoadsCylindersFromYaml)
{
  const auto path = writeYaml(
    "valid",
    "self_mask:\n  cylinders:\n    - center_m: [1.0, 0.0, 0.0]\n      radius_m: 0.2\n"
    "      z_min_m: -0.1\n      z_max_m: 0.1\n");
  const auto mask = loadSelfMask(path);
  EXPECT_TRUE(mask.isMasked(makeReturn(1.0F, 0.0F, 0.0F), kMaxDistanceM));
}

TEST(SelfMaskLoaderTest, RejectsNullMissingEmptyAndNonNumericValues)
{
  EXPECT_THROW(
    loadSelfMask(writeYaml(
      "null",
      "self_mask:\n  cylinders:\n    - center_m: [null, null, null]\n      radius_m: null\n"
      "      z_min_m: null\n      z_max_m: null\n")),
    std::invalid_argument);
  EXPECT_THROW(
    loadSelfMask(writeYaml(
      "missing",
      "self_mask:\n  cylinders:\n    - center_m: [1.0, 0.0, 0.0]\n      radius_m: 0.2\n")),
    std::invalid_argument);
  EXPECT_THROW(
    loadSelfMask(writeYaml("empty", "self_mask:\n  cylinders: []\n")), std::invalid_argument);
  EXPECT_THROW(loadSelfMask(writeYaml("nolist", "sensor:\n  model: x\n")), std::invalid_argument);
  EXPECT_THROW(
    loadSelfMask(writeYaml(
      "text",
      "self_mask:\n  cylinders:\n    - center_m: [1.0, 0.0, 0.0]\n      radius_m: wide\n"
      "      z_min_m: -0.1\n      z_max_m: 0.1\n")),
    std::invalid_argument);
  EXPECT_THROW(loadSelfMask("/nonexistent/self_mask.yaml"), std::invalid_argument);
}

TEST(SelfMaskLoaderTest, RepositoryPlatformConfigIsRefusedWhileValuesAreTbd)
{
  EXPECT_THROW(loadSelfMask(X500_YAML_PATH), std::invalid_argument);
}
