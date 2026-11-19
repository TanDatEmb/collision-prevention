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
    "sensor:\n  collision_elevation_min_rad: -0.2\n  collision_elevation_max_rad: 0.4\n"
    "sector_map:\n  coverage_min_rays_per_sector: 2\n"));

  EXPECT_FLOAT_EQ(config.min_elevation_rad, -0.2F);
  EXPECT_FLOAT_EQ(config.max_elevation_rad, 0.4F);
  EXPECT_EQ(config.coverage_min_rays_per_sector, 2U);
}

TEST(PlatformConfigTest, RejectsNullMissingAndNonNumericValues)
{
  EXPECT_TRUE(rejects(writeYaml(
    "null",
    "sensor:\n  collision_elevation_min_rad: null\n  collision_elevation_max_rad: null\n"
    "sector_map:\n  coverage_min_rays_per_sector: null\n")));
  EXPECT_TRUE(rejects(writeYaml(
    "missing",
    "sensor:\n  collision_elevation_min_rad: -0.2\n  collision_elevation_max_rad: 0.4\n")));
  EXPECT_TRUE(rejects(writeYaml(
    "text",
    "sensor:\n  collision_elevation_min_rad: low\n  collision_elevation_max_rad: 0.4\n"
    "sector_map:\n  coverage_min_rays_per_sector: 2\n")));
  EXPECT_TRUE(rejects("/nonexistent/platform.yaml"));
}

TEST(PlatformConfigTest, RejectsABadCoverageCountOrBand)
{
  const std::string good_band =
    "sensor:\n  collision_elevation_min_rad: -0.2\n  collision_elevation_max_rad: 0.4\n";

  for (const std::string count : {"0", "2.5", "-1"}) {
    EXPECT_TRUE(rejects(writeYaml(
      "count" + count,
      good_band + "sector_map:\n  coverage_min_rays_per_sector: " + count + "\n")));
  }

  EXPECT_TRUE(rejects(writeYaml(
    "band",
    "sensor:\n  collision_elevation_min_rad: 0.4\n  collision_elevation_max_rad: -0.2\n"
    "sector_map:\n  coverage_min_rays_per_sector: 2\n")));
}

TEST(PlatformConfigTest, RepositoryPlatformConfigIsRefusedWhileValuesAreTbd)
{
  EXPECT_TRUE(rejects(X500_YAML_PATH));
}
