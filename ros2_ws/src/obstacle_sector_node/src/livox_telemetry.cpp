#include "obstacle_sector_node/livox_telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace obstacle_sector_node
{
namespace
{

constexpr double kStandardGravityMps2{9.80665};
constexpr uint32_t kNanosecondsPerSecond{1'000'000'000U};

builtin_interfaces::msg::Time rosTime(uint64_t timestamp_ns)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(timestamp_ns / kNanosecondsPerSecond);
  result.nanosec = static_cast<uint32_t>(timestamp_ns % kNanosecondsPerSecond);
  return result;
}

template <typename T>
void writeAt(std::vector<uint8_t> & data, std::size_t offset, const T & value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

bool validReturn(const livox_ray_source::LivoxPointSample & sample)
{
  return sample.sample.ray.return_kind == obstacle_sector_map::RayReturnKind::kReturn &&
         std::isfinite(sample.sample.ray.range_m) && std::isfinite(sample.sample.ray.azimuth_rad) &&
         std::isfinite(sample.sample.ray.elevation_rad) && sample.sample.ray.timestamp_ns != 0U;
}

sensor_msgs::msg::PointField field(const std::string & name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1U;
  return result;
}

}  // namespace

sensor_msgs::msg::Imu livoxImuToMessage(
  const livox_ray_source::LivoxImuSample & sample, const std::string & frame_id)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = rosTime(sample.sample.gyro.timestamp_ns);
  message.header.frame_id = frame_id;
  message.angular_velocity.x = sample.sample.gyro.rate_flu_rad_s[0];
  message.angular_velocity.y = sample.sample.gyro.rate_flu_rad_s[1];
  message.angular_velocity.z = sample.sample.gyro.rate_flu_rad_s[2];
  message.linear_acceleration.x = sample.sample.acceleration_flu_g[0] * kStandardGravityMps2;
  message.linear_acceleration.y = sample.sample.acceleration_flu_g[1] * kStandardGravityMps2;
  message.linear_acceleration.z = sample.sample.acceleration_flu_g[2] * kStandardGravityMps2;
  message.orientation_covariance[0] = -1.0;
  // The samples are measured, but no covariance calibration is available. ROS uses an all-zero
  // covariance to mean "unknown"; -1 would tell consumers to discard these populated estimates.
  return message;
}

sensor_msgs::msg::PointCloud2 livoxPointsToMessage(
  const std::vector<livox_ray_source::LivoxPointSample> & samples, const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = frame_id;
  message.height = 1U;
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = 20U;
  message.fields = {
    field("x", 0U, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4U, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8U, sensor_msgs::msg::PointField::FLOAT32),
    field("reflectivity", 12U, sensor_msgs::msg::PointField::UINT8),
    field("tag", 13U, sensor_msgs::msg::PointField::UINT8),
    field("time_offset_ns", 16U, sensor_msgs::msg::PointField::INT32)};

  std::vector<const livox_ray_source::LivoxPointSample *> returns;
  returns.reserve(samples.size());
  for (const auto & sample : samples) {
    if (validReturn(sample)) {
      returns.push_back(&sample);
    }
  }
  if (returns.empty()) {
    message.row_step = 0U;
    return message;
  }

  const uint64_t first_timestamp_ns = returns.front()->sample.ray.timestamp_ns;
  message.header.stamp = rosTime(first_timestamp_ns);
  message.width = static_cast<uint32_t>(returns.size());
  message.row_step = message.width * message.point_step;
  message.data.resize(message.row_step);
  for (std::size_t index = 0U; index < returns.size(); ++index) {
    const auto & sample = *returns[index];
    const float range = sample.sample.ray.range_m;
    const float cos_elevation = std::cos(sample.sample.ray.elevation_rad);
    const float x = range * cos_elevation * std::cos(sample.sample.ray.azimuth_rad);
    const float y = range * cos_elevation * std::sin(sample.sample.ray.azimuth_rad);
    const float z = range * std::sin(sample.sample.ray.elevation_rad);
    const int64_t raw_offset = static_cast<int64_t>(sample.sample.ray.timestamp_ns) -
                               static_cast<int64_t>(first_timestamp_ns);
    const int64_t clamped_offset = std::clamp(
      raw_offset, static_cast<int64_t>(std::numeric_limits<int32_t>::min()),
      static_cast<int64_t>(std::numeric_limits<int32_t>::max()));
    const std::size_t at = index * message.point_step;
    writeAt(message.data, at, x);
    writeAt(message.data, at + 4U, y);
    writeAt(message.data, at + 8U, z);
    writeAt(message.data, at + 12U, sample.metadata.reflectivity);
    writeAt(message.data, at + 13U, sample.metadata.tag);
    const auto offset_ns = static_cast<int32_t>(clamped_offset);
    writeAt(message.data, at + 16U, offset_ns);
  }
  return message;
}

}  // namespace obstacle_sector_node
