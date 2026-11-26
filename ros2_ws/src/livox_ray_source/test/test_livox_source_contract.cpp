#include <gtest/gtest.h>

#include <string>

#include "livox_ray_source/livox_sdk_sample_source.hpp"

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

}  // namespace
