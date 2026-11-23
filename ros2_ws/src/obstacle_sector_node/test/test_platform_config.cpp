#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "obstacle_sector_node/platform_config.hpp"

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
    "  obstacle_min_returns: 3\n  free_min_rays: 2\n"));

  EXPECT_FLOAT_EQ(config.height_up_m, 1.0F);
  EXPECT_FLOAT_EQ(config.height_down_m, 0.8F);
  EXPECT_FLOAT_EQ(config.obstacle_window_m, 0.5F);
  EXPECT_EQ(config.obstacle_min_returns, 3U);
  EXPECT_EQ(config.free_min_rays, 2U);
}

TEST(PlatformConfigTest, RejectsNullMissingAndNonNumericValues)
{
  EXPECT_TRUE(rejects(writeYaml(
    "null",
    "sector_map:\n  height_up_m: null\n  height_down_m: null\n  obstacle_window_m: null\n"
    "  obstacle_min_returns: null\n  free_min_rays: null\n")));
  EXPECT_TRUE(rejects(writeYaml(
    "missing", "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n")));
  EXPECT_TRUE(rejects(writeYaml(
    "text",
    "sector_map:\n  height_up_m: high\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n")));
  EXPECT_TRUE(rejects("/nonexistent/platform.yaml"));
}

TEST(PlatformConfigTest, RejectsABadCountOrBand)
{
  const std::string prefix =
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n";

  for (const std::string count : {"0", "2.5", "-1"}) {
    EXPECT_TRUE(rejects(writeYaml(
      "obstacles" + count, prefix + "  obstacle_min_returns: " + count + "\n  free_min_rays: 2\n")));
    EXPECT_TRUE(rejects(writeYaml(
      "free" + count, prefix + "  obstacle_min_returns: 2\n  free_min_rays: " + count + "\n")));
  }

  EXPECT_TRUE(rejects(writeYaml(
    "band",
    "sector_map:\n  height_up_m: -1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n")));
  EXPECT_TRUE(rejects(writeYaml(
    "window",
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.0\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n")));
}

TEST(PlatformConfigTest, TheRearStartIsOptionalAndMustBeAnAngleFromZeroToPi)
{
  const std::string base =
    "sector_map:\n  height_up_m: 1.0\n  height_down_m: 1.0\n  obstacle_window_m: 0.5\n"
    "  obstacle_min_returns: 2\n  free_min_rays: 2\n";
  EXPECT_NEAR(loadSectorMapConfig(writeYaml("rear_absent", base)).rear_start_rad, 3.14159265F, 1e-6F);
  EXPECT_NEAR(
    loadSectorMapConfig(writeYaml("rear_null", base + "  rear_start_rad: null\n")).rear_start_rad,
    3.14159265F, 1e-6F);
  EXPECT_NEAR(
    loadSectorMapConfig(writeYaml("rear_set", base + "  rear_start_rad: 2.3492\n")).rear_start_rad, 2.3492F,
    1e-5F);
  for (const std::string value : {"0", "-1", "3.5", ".nan"}) {
    EXPECT_TRUE(rejects(writeYaml("rear" + value, base + "  rear_start_rad: " + value + "\n")));
  }
}

TEST(PlatformConfigTest, RepositoryPlatformConfigIsRefusedWhileValuesAreTbd)
{
  EXPECT_TRUE(rejects(X500_YAML_PATH));
}
