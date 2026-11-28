#pragma once

#include <string>
#include <vector>

#include "livox_ray_source/livox_sdk_sample_source.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace obstacle_sector_node
{

/// Converts one validated Livox IMU sample into the standard ROS message. Acceleration is converted
/// from the Livox wire unit g to m/s²; orientation is intentionally unknown because this driver does
/// not fuse accelerometer data.
[[nodiscard]] sensor_msgs::msg::Imu livoxImuToMessage(
  const livox_ray_source::LivoxImuSample & sample, const std::string & frame_id);

/// Converts valid-return Livox samples into a PointCloud2. Qualified no-return and invalid rays are
/// omitted from the geometric cloud but remain visible in adapter diagnostics. Fields are x/y/z
/// float32, reflectivity uint8, tag uint8, and signed int32 time_offset_ns relative to the first return.
[[nodiscard]] sensor_msgs::msg::PointCloud2 livoxPointsToMessage(
  const std::vector<livox_ray_source::LivoxPointSample> & samples, const std::string & frame_id);

}  // namespace obstacle_sector_node
