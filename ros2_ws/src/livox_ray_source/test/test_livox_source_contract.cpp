#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "livox_ray_source/livox_sdk_sample_source.hpp"
#include "livox_ray_source/mid360s_config.hpp"

namespace
{

livox_ray_source::LivoxDeviceIdentity device(uint32_t handle, const char * serial)
{
  livox_ray_source::LivoxDeviceIdentity result;
  result.handle = handle;
  result.device_type = 35U;
  result.serial = serial;
  result.lidar_ip = "192.168.1.100";
  return result;
}

TEST(LivoxDeviceRegistryTest, ASingleDetectedDeviceBecomesThePrimary)
{
  livox_ray_source::LivoxDeviceRegistry registry;
  registry.observe(device(1U, "A"));
  EXPECT_FALSE(registry.multipleDeviceFault());
  ASSERT_TRUE(registry.primaryDevice().has_value());
  EXPECT_EQ(registry.primaryDevice()->serial, "A");
  EXPECT_TRUE(registry.accepts(1U));
}

TEST(LivoxDeviceRegistryTest, MultipleDevicesRequireAnExplicitPrimary)
{
  livox_ray_source::LivoxDeviceRegistry registry;
  registry.observe(device(1U, "A"));
  registry.observe(device(2U, "B"));
  EXPECT_TRUE(registry.multipleDeviceFault());
  EXPECT_FALSE(registry.accepts(1U));
  EXPECT_FALSE(registry.accepts(2U));
  EXPECT_EQ(registry.devices().size(), 2U);
}

TEST(LivoxDeviceRegistryTest, ConfiguredPrimaryIsTheOnlySafetyStream)
{
  livox_ray_source::LivoxDeviceRegistry registry("B");
  registry.observe(device(1U, "A"));
  registry.observe(device(2U, "B"));
  EXPECT_FALSE(registry.multipleDeviceFault());
  EXPECT_FALSE(registry.accepts(1U));
  EXPECT_TRUE(registry.accepts(2U));
}

TEST(LivoxDeviceRegistryTest, ConfiguredPrimaryMustBeObservedBeforeSafetyIsAccepted)
{
  livox_ray_source::LivoxDeviceRegistry registry("missing");
  registry.observe(device(1U, "other"));
  EXPECT_FALSE(registry.multipleDeviceFault());
  EXPECT_FALSE(registry.primaryDevice().has_value());
  EXPECT_FALSE(registry.accepts(1U));
}

TEST(LivoxDeviceRegistryTest, HandleIdentityReplacementCannotKeepTheOldSafetySelection)
{
  livox_ray_source::LivoxDeviceRegistry configured("A");
  configured.observe(device(1U, "A"));
  configured.observe(device(1U, "B"));
  EXPECT_FALSE(configured.primaryDevice().has_value());
  EXPECT_FALSE(configured.accepts(1U));

  livox_ray_source::LivoxDeviceRegistry automatic;
  automatic.observe(device(1U, "A"));
  automatic.observe(device(1U, "B"));
  EXPECT_TRUE(automatic.multipleDeviceFault());
  EXPECT_FALSE(automatic.primaryDevice().has_value());
  EXPECT_FALSE(automatic.accepts(1U));
}

TEST(LivoxDeviceRegistryTest, ClearStartsAFreshDetectionEpoch)
{
  livox_ray_source::LivoxDeviceRegistry registry;
  registry.observe(device(1U, "A"));
  registry.observe(device(2U, "B"));
  ASSERT_TRUE(registry.multipleDeviceFault());
  registry.clear();
  registry.observe(device(2U, "B"));
  EXPECT_FALSE(registry.multipleDeviceFault());
  EXPECT_TRUE(registry.accepts(2U));
}

TEST(LivoxHealthTest, AZeroTimestampOrCurrentLayoutMismatchIsNotUsable)
{
  livox_ray_source::LivoxDeviceHealth health;
  health.last_point_timestamp_ns = 1'000U;
  EXPECT_TRUE(livox_ray_source::hasUsablePointObservation(health));

  health.last_point_timestamp_ns = 0U;
  EXPECT_FALSE(livox_ray_source::hasUsablePointObservation(health));

  health.last_point_timestamp_ns = 1'000U;
  health.current_point_layout_mismatch = true;
  EXPECT_FALSE(livox_ray_source::hasUsablePointObservation(health));
}

TEST(Mid360SConfigTest, ParsesOptionalMid360SDriverSettings)
{
  const auto config = livox_ray_source::parseMid360SConfig(R"json({
    "collision_prevention": {
      "point_data_type": "cartesian_high",
      "scan_pattern": "repetitive_low_frame_rate",
      "blind_spot_mm": 120,
      "dual_emit": true,
      "pps_sync": "special",
      "install_attitude": {
        "roll_deg": 1.5,
        "pitch_deg": -10.0,
        "yaw_deg": 2.25,
        "x_mm": 120,
        "y_mm": -30,
        "z_mm": 80
      },
      "imu": {"rate_hz": 500, "accel_range_g": 16, "gyro_range_dps": 500}
    }
  })json");
  EXPECT_EQ(config.point_data_type, livox_ray_source::Mid360SPointDataType::kCartesianHigh);
  ASSERT_TRUE(config.scan_pattern.has_value());
  EXPECT_EQ(*config.scan_pattern, livox_ray_source::Mid360SScanPattern::kRepetitiveLowFrameRate);
  ASSERT_TRUE(config.blind_spot_mm.has_value());
  EXPECT_EQ(*config.blind_spot_mm, 120U);
  ASSERT_TRUE(config.dual_emit.has_value());
  EXPECT_TRUE(*config.dual_emit);
  ASSERT_TRUE(config.pps_sync_mode.has_value());
  EXPECT_EQ(*config.pps_sync_mode, livox_ray_source::Mid360SPpsSyncMode::kSpecial);
  ASSERT_TRUE(config.install_attitude.has_value());
  EXPECT_FLOAT_EQ(config.install_attitude->roll_deg, 1.5F);
  EXPECT_FLOAT_EQ(config.install_attitude->pitch_deg, -10.0F);
  EXPECT_FLOAT_EQ(config.install_attitude->yaw_deg, 2.25F);
  EXPECT_EQ(config.install_attitude->x_mm, 120);
  EXPECT_EQ(config.install_attitude->y_mm, -30);
  EXPECT_EQ(config.install_attitude->z_mm, 80);
  ASSERT_TRUE(config.imu_range.has_value());
  EXPECT_EQ(config.imu_range->output_rate, livox_ray_source::Mid360SImuRate::k500Hz);
  EXPECT_EQ(config.imu_range->accel_range, livox_ray_source::Mid360SAccelRange::k16G);
  EXPECT_EQ(config.imu_range->gyro_range, livox_ray_source::Mid360SGyroRange::k500Dps);
}

TEST(Mid360SConfigTest, MissingDriverObjectUsesSafetySphericalDefaults)
{
  const auto config = livox_ray_source::parseMid360SConfig(R"json({"Mid360s": {}})json");
  EXPECT_EQ(config.point_data_type, livox_ray_source::Mid360SPointDataType::kSpherical);
  EXPECT_FALSE(config.scan_pattern.has_value());
  EXPECT_FALSE(config.imu_range.has_value());
  EXPECT_FALSE(config.install_attitude.has_value());
}

TEST(Mid360SConfigTest, RejectsUnsupportedEnumValues)
{
  EXPECT_THROW(
    (void)livox_ray_source::parseMid360SConfig(
      R"json({"collision_prevention":{"point_data_type":"mid40"}})json"),
    std::invalid_argument);
  EXPECT_THROW(
    (void)livox_ray_source::parseMid360SConfig(
      R"json({"collision_prevention":{"imu":{"rate_hz":60}}})json"),
    std::invalid_argument);
  EXPECT_THROW(
    (void)livox_ray_source::parseMid360SConfig(
      R"json({"collision_prevention":{"install_attitude":{"roll_deg":0,"pitch_deg":0,"yaw_deg":0,"x_mm":0,"y_mm":0}}})json"),
    std::invalid_argument);
}

}  // namespace
