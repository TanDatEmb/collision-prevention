#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "obstacle_sector_node/livox_telemetry.hpp"

namespace
{

livox_ray_source::LivoxPointSample point(
  float range_m, obstacle_sector_map::RayReturnKind kind, uint8_t reflectivity, uint8_t tag,
  uint64_t timestamp_ns)
{
  livox_ray_source::LivoxPointSample sample;
  sample.device.serial = "ARMCP550031900";
  sample.sample.ray.range_m = range_m;
  sample.sample.ray.azimuth_rad = 0.0F;
  sample.sample.ray.elevation_rad = 0.0F;
  sample.sample.ray.timestamp_ns = timestamp_ns;
  sample.sample.ray.return_kind = kind;
  sample.metadata.reflectivity = reflectivity;
  sample.metadata.tag = tag;
  return sample;
}

TEST(LivoxTelemetryTest, ImuMessageConvertsAccelerationFromGAndPreservesPtpTime)
{
  livox_ray_source::LivoxImuSample sample;
  sample.device.serial = "ARMCP550031900";
  sample.sample.gyro.timestamp_ns = 1'795'000'000'123'456'789U;
  sample.sample.gyro.rate_flu_rad_s = {0.1, -0.2, 0.3};
  sample.sample.acceleration_flu_g = {0.0, -0.5, 1.0};

  const auto message = obstacle_sector_node::livoxImuToMessage(sample, "livox_ARMCP550031900");
  EXPECT_EQ(message.header.stamp.sec, 1'795'000'000);
  EXPECT_EQ(message.header.stamp.nanosec, 123'456'789U);
  EXPECT_EQ(message.header.frame_id, "livox_ARMCP550031900");
  EXPECT_DOUBLE_EQ(message.angular_velocity.x, 0.1);
  EXPECT_DOUBLE_EQ(message.linear_acceleration.y, -0.5 * 9.80665);
  EXPECT_EQ(message.orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(message.angular_velocity_covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(message.linear_acceleration_covariance[0], 0.0);
}

TEST(LivoxTelemetryTest, PointCloudContainsOnlyReturnsAndRawMetadata)
{
  const std::vector<livox_ray_source::LivoxPointSample> samples{
    point(2.0F, obstacle_sector_map::RayReturnKind::kReturn, 17U, 3U, 1'000'000U),
    point(
      std::numeric_limits<float>::quiet_NaN(),
      obstacle_sector_map::RayReturnKind::kQualifiedNoReturn, 0U, 9U, 1'000'100U)};

  const auto message = obstacle_sector_node::livoxPointsToMessage(samples, "livox_ARMCP550031900");
  EXPECT_EQ(message.height, 1U);
  ASSERT_EQ(message.width, 1U);
  ASSERT_EQ(message.point_step, 20U);
  ASSERT_EQ(message.fields.size(), 6U);
  EXPECT_EQ(message.header.frame_id, "livox_ARMCP550031900");
  EXPECT_EQ(message.fields[3].name, "reflectivity");
  EXPECT_EQ(message.fields[4].name, "tag");
  uint8_t reflectivity = 0U;
  uint8_t tag = 0U;
  std::memcpy(&reflectivity, message.data.data() + 12U, sizeof(reflectivity));
  std::memcpy(&tag, message.data.data() + 13U, sizeof(tag));
  EXPECT_EQ(reflectivity, 17U);
  EXPECT_EQ(tag, 3U);
}

}  // namespace
