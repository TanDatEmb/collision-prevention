#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "obstacle_sector_node/platform_config.hpp"

using obstacle_sector_node::loadGyroHistoryConfig;
using obstacle_sector_node::loadLevelingSettings;
using obstacle_sector_node::loadSectorMapConfig;

namespace
{

// Test fixture values, not approved platform values.
std::string writeYaml(const std::string & name, const std::string & text)
{
  const auto path =
    std::filesystem::temp_directory_path() / ("sector_config_fixture_" + name + ".yaml");
  std::ofstream(path) << text;
  return path.string();
}

bool rejects(const std::string & path)
{
  try {
    static_cast<void>(loadSectorMapConfig(path));
  }
  catch (const std::invalid_argument &) {
    return true;
  }
  return false;
}

}  // namespace

TEST(PlatformConfigTest, LoadsTheSectorMapValues)
{
  const auto config = loadSectorMapConfig(writeYaml(
    "valid",
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 0.8\n  obstacle_window_m: 0.5\n"
    "  obstacle_min_returns: 3\n  free_min_rays: 2\n"
    "  certified_range_m: 12.0\n  attitude_allowance_rad: 0.01\n"));

  EXPECT_FLOAT_EQ(config.height_up_m, 1.0F);
  EXPECT_FLOAT_EQ(config.height_down_m, 0.8F);
  EXPECT_FLOAT_EQ(config.obstacle_window_m, 0.5F);
  EXPECT_EQ(config.obstacle_min_returns, 3U);
  EXPECT_EQ(config.free_min_rays, 2U);
  EXPECT_FLOAT_EQ(config.certified_range_m, 12.0F);
  EXPECT_FLOAT_EQ(config.attitude_allowance_rad, 0.01F);
}

TEST(PlatformConfigTest, TheCertifiedRangeAndTheAllowanceAreRequiredAndChecked)
{
  const std::string base =
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n";
  EXPECT_TRUE(rejects(writeYaml("range_missing", base + "  attitude_allowance_rad: 0.01\n")));
  EXPECT_TRUE(rejects(writeYaml("allowance_missing", base + "  certified_range_m: 12.0\n")));
  EXPECT_TRUE(rejects(
    writeYaml("range_null", base + "  certified_range_m: null\n  attitude_allowance_rad: 0.01\n")));
  for (const std::string range : {"0.05", "40.5", "-3"}) {
    EXPECT_TRUE(rejects(writeYaml(
      "range" + range,
      base + "  certified_range_m: " + range + "\n  attitude_allowance_rad: 0.01\n")));
  }
  EXPECT_TRUE(rejects(writeYaml(
    "allowance_negative", base + "  certified_range_m: 12.0\n  attitude_allowance_rad: -0.01\n")));
}

TEST(PlatformConfigTest, RejectsNullMissingAndNonNumericValues)
{
  EXPECT_TRUE(rejects(writeYaml(
    "null",
    "sector_map:\n  height_up_m: null\n  height_down_m: null\n  obstacle_window_m: null\n"
    "  obstacle_min_returns: null\n  free_min_rays: null\n")));
  EXPECT_TRUE(rejects(writeYaml(
    "missing",
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n")));
  EXPECT_TRUE(rejects(writeYaml(
    "text",
    "sector_map:\n  height_up_m: high\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n")));
  EXPECT_TRUE(rejects("/nonexistent/platform.yaml"));
}

TEST(PlatformConfigTest, RejectsABadCountOrBand)
{
  const std::string prefix =
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n"
    "  certified_range_m: 12.0\n  attitude_allowance_rad: 0.01\n";

  for (const std::string count : {"0", "2.5", "-1"}) {
    EXPECT_TRUE(rejects(writeYaml(
      "obstacles" + count,
      prefix + "  obstacle_min_returns: " + count + "\n  free_min_rays: 2\n")));
    EXPECT_TRUE(rejects(writeYaml(
      "free" + count, prefix + "  obstacle_min_returns: 2\n  free_min_rays: " + count + "\n")));
  }

  EXPECT_TRUE(rejects(writeYaml(
    "band",
    "sector_map:\n  height_up_m: -1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n"
    "  certified_range_m: 12.0\n  attitude_allowance_rad: 0.01\n")));
  EXPECT_TRUE(rejects(writeYaml(
    "window",
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.0\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n"
    "  certified_range_m: 12.0\n  attitude_allowance_rad: 0.01\n")));
}

TEST(PlatformConfigTest, TheRearStartIsOptionalAndMustBeAnAngleFromZeroToPi)
{
  const std::string base =
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n"
    "  certified_range_m: 12.0\n  attitude_allowance_rad: 0.01\n";
  EXPECT_NEAR(
    loadSectorMapConfig(writeYaml("rear_absent", base)).rear_start_rad, 3.14159265F, 1e-6F);
  EXPECT_NEAR(
    loadSectorMapConfig(writeYaml("rear_null", base + "  rear_start_rad: null\n")).rear_start_rad,
    3.14159265F, 1e-6F);
  EXPECT_NEAR(
    loadSectorMapConfig(writeYaml("rear_set", base + "  rear_start_rad: 2.3492\n")).rear_start_rad,
    2.3492F, 1e-5F);
  for (const std::string value : {"0", "-1", "3.5", ".nan"}) {
    EXPECT_TRUE(rejects(writeYaml("rear" + value, base + "  rear_start_rad: " + value + "\n")));
  }
}

TEST(PlatformConfigTest, RepositoryPlatformConfigHoldsTheApprovedBenchValues)
{
  // G4 bench values approved by the owner on 2026-11-24 (#207)
  const auto config = loadSectorMapConfig(X500_YAML_PATH);
  EXPECT_FLOAT_EQ(config.height_up_m, 0.25F);
  EXPECT_FLOAT_EQ(config.height_down_m, 0.60F);
  EXPECT_FLOAT_EQ(config.obstacle_window_m, 0.5F);
  EXPECT_EQ(config.obstacle_min_returns, 3U);
  EXPECT_EQ(config.free_min_rays, 5U);
  EXPECT_FLOAT_EQ(config.certified_range_m, 20.0F);
  EXPECT_FLOAT_EQ(config.attitude_allowance_rad, 0.0157F);
}

namespace
{

std::string levelingYaml(
  const std::string & mount_xyz, const std::string & mount_rpy, const std::string & depth,
  const std::string & gap_ms)
{
  return "sensor:\n  mount_pose_in_base_link:\n    xyz_m: " + mount_xyz +
         "\n    rpy_rad: " + mount_rpy + "\npose_history:\n  depth: " + depth +
         "\n  max_bracket_gap_ms: " + gap_ms + "\n";
}

bool levelingRejects(const std::string & path)
{
  try {
    static_cast<void>(loadLevelingSettings(path));
  }
  catch (const std::invalid_argument &) {
    return true;
  }
  return false;
}

}  // namespace

TEST(LevelingSettingsTest, LoadsTheMountAndThePoseHistory)
{
  // Fixture values, not platform values.
  const auto settings = loadLevelingSettings(writeYaml(
    "leveling_valid", levelingYaml("[0.1, -0.2, 0.3]", "[0.01, 0.2, -0.03]", "16", "50")));
  EXPECT_DOUBLE_EQ(settings.leveling.lever_arm_flu_m[0], 0.1);
  EXPECT_DOUBLE_EQ(settings.leveling.lever_arm_flu_m[1], -0.2);
  EXPECT_DOUBLE_EQ(settings.leveling.lever_arm_flu_m[2], 0.3);
  EXPECT_DOUBLE_EQ(settings.leveling.roll_rad, 0.01);
  EXPECT_DOUBLE_EQ(settings.leveling.pitch_down_rad, 0.2);
  EXPECT_DOUBLE_EQ(settings.leveling.yaw_left_rad, -0.03);
  EXPECT_EQ(settings.history.depth, 16U);
  EXPECT_EQ(settings.history.max_bracket_gap_ns, 50'000'000U);
}

TEST(LevelingSettingsTest, RejectsNullMissingAndInvalidValues)
{
  const std::string xyz = "[0.1, 0.0, 0.1]";
  const std::string rpy = "[0.0, 0.2, 0.0]";
  EXPECT_FALSE(levelingRejects(writeYaml("lv_ok", levelingYaml(xyz, rpy, "16", "50"))));
  EXPECT_TRUE(levelingRejects(writeYaml("lv_xyz_null", levelingYaml("null", rpy, "16", "50"))));
  EXPECT_TRUE(
    levelingRejects(writeYaml("lv_xyz_short", levelingYaml("[0.1, 0.0]", rpy, "16", "50"))));
  EXPECT_TRUE(levelingRejects(
    writeYaml("lv_xyz_null_item", levelingYaml("[0.1, null, 0.1]", rpy, "16", "50"))));
  EXPECT_TRUE(levelingRejects(writeYaml("lv_rpy_null", levelingYaml(xyz, "null", "16", "50"))));
  EXPECT_TRUE(
    levelingRejects(writeYaml("lv_rpy_big", levelingYaml(xyz, "[0.0, 4.0, 0.0]", "16", "50"))));
  EXPECT_TRUE(levelingRejects(writeYaml("lv_depth_null", levelingYaml(xyz, rpy, "null", "50"))));
  EXPECT_TRUE(levelingRejects(writeYaml("lv_depth_one", levelingYaml(xyz, rpy, "1", "50"))));
  EXPECT_TRUE(levelingRejects(writeYaml("lv_depth_frac", levelingYaml(xyz, rpy, "2.5", "50"))));
  EXPECT_TRUE(levelingRejects(writeYaml("lv_gap_null", levelingYaml(xyz, rpy, "16", "null"))));
  EXPECT_TRUE(levelingRejects(writeYaml("lv_gap_zero", levelingYaml(xyz, rpy, "16", "0"))));
  EXPECT_TRUE(levelingRejects(writeYaml("lv_gap_neg", levelingYaml(xyz, rpy, "16", "-5"))));
  EXPECT_TRUE(levelingRejects("/nonexistent/platform.yaml"));
}

TEST(LevelingSettingsTest, RepositoryPlatformConfigHoldsTheMountAndThePoseHistory)
{
  // The mount (owner, 2026-11-21) and the bench pose history (owner, 2026-11-24, #207)
  const auto settings = loadLevelingSettings(X500_YAML_PATH);
  EXPECT_DOUBLE_EQ(settings.leveling.lever_arm_flu_m[0], 0.12);
  EXPECT_DOUBLE_EQ(settings.leveling.lever_arm_flu_m[2], 0.14);
  EXPECT_DOUBLE_EQ(settings.leveling.pitch_down_rad, 0.1745);
  EXPECT_EQ(settings.history.depth, 64U);
  EXPECT_EQ(settings.history.max_bracket_gap_ns, 50'000'000U);
}

TEST(GyroHistoryConfigTest, LoadsTheGyroHistoryAndRefusesMissingOrBadValues)
{
  // Fixture values, not platform values.
  const auto config = loadGyroHistoryConfig(
    writeYaml("gyro_valid", "gyro_history:\n  depth: 64\n  max_bracket_gap_ms: 10\n"));
  EXPECT_EQ(config.depth, 64U);
  EXPECT_EQ(config.max_bracket_gap_ns, 10'000'000U);
  const auto refused = [](const std::string & name, const std::string & text) {
    try {
      static_cast<void>(loadGyroHistoryConfig(writeYaml(name, text)));
    }
    catch (const std::invalid_argument &) {
      return true;
    }
    return false;
  };
  EXPECT_TRUE(refused("gyro_missing", "pose_history:\n  depth: 64\n  max_bracket_gap_ms: 10\n"));
  EXPECT_TRUE(refused("gyro_null", "gyro_history:\n  depth: null\n  max_bracket_gap_ms: 10\n"));
  EXPECT_TRUE(refused("gyro_depth", "gyro_history:\n  depth: 1\n  max_bracket_gap_ms: 10\n"));
  EXPECT_TRUE(refused("gyro_gap", "gyro_history:\n  depth: 64\n  max_bracket_gap_ms: 0\n"));
  EXPECT_TRUE(refused("gyro_empty", std::string{}));
  EXPECT_THROW(
    static_cast<void>(loadGyroHistoryConfig(X500_YAML_PATH)), std::invalid_argument);  // TBD
}

TEST(GyroHistoryConfigTest, RepositoryLivoxBenchProfileUsesTheOwnerSelectedBenchValues)
{
  const auto config = loadGyroHistoryConfig(X500_LIVOX_BENCH_YAML_PATH);
  EXPECT_EQ(config.depth, 128U);
  EXPECT_EQ(config.max_bracket_gap_ns, 10'000'000U);
}
