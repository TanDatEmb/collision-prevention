#include "livox_ray_source/spherical_packet.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace livox_ray_source
{

namespace
{

using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;

constexpr std::size_t kHeaderBytes{36U};
constexpr std::size_t kPointBytes{10U};
constexpr std::size_t kCartesianHighPointBytes{14U};
constexpr std::size_t kCartesianLowPointBytes{8U};
constexpr uint8_t kSphericalDataType{3U};
constexpr uint8_t kCartesianHighDataType{1U};
constexpr uint8_t kCartesianLowDataType{2U};
constexpr uint8_t kImuDataType{0U};
constexpr std::size_t kImuSampleBytes{24U};  // six float32
constexpr uint8_t kPtpTimeType{1U};
constexpr uint32_t kMaxTheta{18000U};  // 0.01 degree
constexpr uint32_t kMaxPhi{36000U};    // 0.01 degree
constexpr double kPi{3.14159265358979323846};
constexpr double kRadiansPerHundredthDegree{kPi / 18000.0};
constexpr uint64_t kNanosecondsPerTenthMicrosecond{100U};

// The lidar and the host are little endian; the layout is fixed by the protocol.
template <typename T>
T readLe(const uint8_t * at)
{
  T value;
  std::memcpy(&value, at, sizeof(T));
  return value;
}

}  // namespace

DecodedImuPacket decodeImuPacket(const uint8_t * data, std::size_t size)
{
  DecodedImuPacket packet;
  if (data == nullptr || size != kHeaderBytes + kImuSampleBytes || data[10] != kImuDataType) {
    return packet;
  }
  packet.layout_ok = true;
  packet.time_type = data[11];
  const uint64_t packet_time_ns = readLe<uint64_t>(data + 28);
  if (packet.time_type != kPtpTimeType || packet_time_ns == 0U) {
    return packet;
  }
  packet.gyro.timestamp_ns = packet_time_ns;
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    packet.gyro.rate_flu_rad_s[axis] =
      static_cast<double>(readLe<float>(data + kHeaderBytes + axis * sizeof(float)));
    packet.acceleration_flu_g[axis] =
      static_cast<double>(readLe<float>(data + kHeaderBytes + (axis + 3U) * sizeof(float)));
  }
  return packet;
}

bool ImuSample::isValid() const
{
  return gyro.isValid() && std::isfinite(acceleration_flu_g[0]) &&
         std::isfinite(acceleration_flu_g[1]) && std::isfinite(acceleration_flu_g[2]);
}

bool DecodedImuPacket::accelerationIsValid() const { return toImuSample().isValid(); }

ImuSample DecodedImuPacket::toImuSample() const { return ImuSample{gyro, acceleration_flu_g}; }

DecodedPacket decodeSphericalPacket(const uint8_t * data, std::size_t size)
{
  DecodedPacket packet;
  if (data == nullptr || size < kHeaderBytes) {
    packet.status = DecodeStatus::kTooShort;
    return packet;
  }
  const uint16_t time_interval = readLe<uint16_t>(data + 3);
  const uint16_t dot_num = readLe<uint16_t>(data + 5);
  packet.udp_cnt = readLe<uint16_t>(data + 7);
  const uint8_t data_type = data[10];
  packet.time_type = data[11];
  packet.data_type = data_type;
  const uint64_t packet_time_ns = readLe<uint64_t>(data + 28);

  if (data_type != kSphericalDataType) {
    packet.status = DecodeStatus::kNotSpherical;
    return packet;
  }
  if (size != kHeaderBytes + static_cast<std::size_t>(dot_num) * kPointBytes) {
    packet.status = DecodeStatus::kLengthMismatch;
    return packet;
  }
  if (dot_num == 0U) {
    packet.status = DecodeStatus::kNoPoints;
    return packet;
  }

  const bool time_usable = packet.time_type == kPtpTimeType && packet_time_ns != 0U;
  const uint64_t interval_ns =
    static_cast<uint64_t>(time_interval) * kNanosecondsPerTenthMicrosecond;
  packet.rays.reserve(dot_num);
  packet.point_metadata.reserve(dot_num);
  for (std::size_t index = 0U; index < dot_num; ++index) {
    const uint8_t * point = data + kHeaderBytes + index * kPointBytes;
    const uint32_t depth_mm = readLe<uint32_t>(point);
    const uint16_t theta = readLe<uint16_t>(point + 4);
    const uint16_t phi = readLe<uint16_t>(point + 6);
    const uint8_t reflectivity = point[8];
    const uint8_t tag = point[9];

    RaySample ray;
    ray.azimuth_rad = static_cast<float>(static_cast<double>(phi) * kRadiansPerHundredthDegree);
    ray.elevation_rad =
      static_cast<float>(kPi / 2.0 - static_cast<double>(theta) * kRadiansPerHundredthDegree);
    ray.range_m = static_cast<float>(static_cast<double>(depth_mm) / 1000.0);
    ray.masked = false;
    const bool angles_valid = theta <= kMaxTheta && phi <= kMaxPhi;
    // Depth 0 with a valid direction is a shot with no return (issue #172): no object in range. No vendor
    // document defines it, but the vendor convention is (0, theta, phi) in spherical data for "no detected
    // object ahead or the detected object exceeds the range", the vendor driver forwards it unfiltered, and a
    // real Mid-360S sends them (48.7 % of the points of a room scene) at a fixed 200,000 points/s. The frame
    // gates of ADR 0004 (packet continuity, PTP time, the ray count) still decide whether a sector is clear.
    if (!angles_valid) {
      ray.return_kind = RayReturnKind::kInvalid;
    }
    else if (depth_mm == 0U) {
      ray.return_kind = RayReturnKind::kQualifiedNoReturn;
      ray.range_m = std::numeric_limits<float>::quiet_NaN();
    }
    else {
      ray.return_kind = RayReturnKind::kReturn;
    }
    ray.timestamp_ns =
      time_usable ? packet_time_ns + (dot_num > 1U ? (index * interval_ns) / (dot_num - 1U) : 0U)
                  : 0U;
    packet.rays.push_back(ray);
    packet.point_metadata.push_back(
      PointMetadata{reflectivity, tag, static_cast<uint8_t>(index % 4U)});
  }
  packet.status = DecodeStatus::kOk;
  return packet;
}

namespace
{

template <typename CoordinateReader>
DecodedPacket decodeCartesianPacket(
  const uint8_t * data, std::size_t size, uint8_t expected_type, std::size_t point_bytes,
  CoordinateReader read_coordinates)
{
  DecodedPacket packet;
  if (data == nullptr || size < kHeaderBytes) {
    packet.status = DecodeStatus::kTooShort;
    return packet;
  }

  const uint16_t time_interval = readLe<uint16_t>(data + 3U);
  const uint16_t dot_num = readLe<uint16_t>(data + 5U);
  packet.udp_cnt = readLe<uint16_t>(data + 7U);
  packet.data_type = data[10];
  packet.time_type = data[11];
  const uint64_t packet_time_ns = readLe<uint64_t>(data + 28U);
  if (packet.data_type != expected_type) {
    packet.status = DecodeStatus::kNotSpherical;
    return packet;
  }
  if (size != kHeaderBytes + static_cast<std::size_t>(dot_num) * point_bytes) {
    packet.status = DecodeStatus::kLengthMismatch;
    return packet;
  }
  if (dot_num == 0U) {
    packet.status = DecodeStatus::kNoPoints;
    return packet;
  }

  const bool time_usable = packet.time_type == kPtpTimeType && packet_time_ns != 0U;
  const uint64_t interval_ns =
    static_cast<uint64_t>(time_interval) * kNanosecondsPerTenthMicrosecond;
  packet.rays.reserve(dot_num);
  packet.point_metadata.reserve(dot_num);
  for (std::size_t index = 0U; index < dot_num; ++index) {
    const uint8_t * point = data + kHeaderBytes + index * point_bytes;
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;
    uint8_t reflectivity = 0U;
    uint8_t tag = 0U;
    read_coordinates(point, x_m, y_m, z_m, reflectivity, tag);

    const double horizontal_m = std::hypot(x_m, y_m);
    const double range_m = std::hypot(horizontal_m, z_m);
    const bool has_direction = range_m > 0.0 && std::isfinite(range_m);
    RaySample ray;
    ray.timestamp_ns =
      time_usable ? packet_time_ns + (dot_num > 1U ? (index * interval_ns) / (dot_num - 1U) : 0U)
                  : 0U;
    ray.masked = false;
    ray.range_m =
      has_direction ? static_cast<float>(range_m) : std::numeric_limits<float>::quiet_NaN();
    ray.azimuth_rad = has_direction ? static_cast<float>(std::atan2(y_m, x_m)) : 0.0F;
    ray.elevation_rad = has_direction ? static_cast<float>(std::atan2(z_m, horizontal_m)) : 0.0F;
    ray.return_kind = has_direction ? RayReturnKind::kReturn : RayReturnKind::kInvalid;
    packet.rays.push_back(ray);
    packet.point_metadata.push_back(
      PointMetadata{reflectivity, tag, static_cast<uint8_t>(index % 4U)});
  }
  packet.status = DecodeStatus::kOk;
  return packet;
}

}  // namespace

DecodedPacket decodeCartesianHighPacket(const uint8_t * data, std::size_t size)
{
  return decodeCartesianPacket(
    data, size, kCartesianHighDataType, kCartesianHighPointBytes,
    [](
      const uint8_t * point, double & x_m, double & y_m, double & z_m, uint8_t & reflectivity,
      uint8_t & tag) {
      x_m = static_cast<double>(readLe<int32_t>(point)) / 1000.0;
      y_m = static_cast<double>(readLe<int32_t>(point + 4U)) / 1000.0;
      z_m = static_cast<double>(readLe<int32_t>(point + 8U)) / 1000.0;
      reflectivity = point[12U];
      tag = point[13U];
    });
}

DecodedPacket decodeCartesianLowPacket(const uint8_t * data, std::size_t size)
{
  return decodeCartesianPacket(
    data, size, kCartesianLowDataType, kCartesianLowPointBytes,
    [](
      const uint8_t * point, double & x_m, double & y_m, double & z_m, uint8_t & reflectivity,
      uint8_t & tag) {
      x_m = static_cast<double>(readLe<int16_t>(point)) / 100.0;
      y_m = static_cast<double>(readLe<int16_t>(point + 2U)) / 100.0;
      z_m = static_cast<double>(readLe<int16_t>(point + 4U)) / 100.0;
      reflectivity = point[6U];
      tag = point[7U];
    });
}

DecodedPacket decodePointPacket(const uint8_t * data, std::size_t size)
{
  if (data == nullptr || size < kHeaderBytes) {
    return decodeSphericalPacket(data, size);
  }
  switch (data[10U]) {
    case kCartesianHighDataType:
      return decodeCartesianHighPacket(data, size);
    case kCartesianLowDataType:
      return decodeCartesianLowPacket(data, size);
    case kSphericalDataType:
      return decodeSphericalPacket(data, size);
    default:
      return decodeSphericalPacket(data, size);
  }
}

bool UdpContinuityTracker::update(uint16_t udp_cnt)
{
  const bool is_break =
    _has_previous && udp_cnt != static_cast<uint16_t>(static_cast<uint16_t>(_previous + 1U));
  _has_previous = true;
  _previous = udp_cnt;
  return is_break;
}

std::size_t PacketSampler::append(
  const DecodedPacket & packet, bool udp_break, std::vector<StreamSample> & out,
  std::vector<PointMetadata> * metadata)
{
  _pending_break = _pending_break || udp_break;
  std::size_t dropped = 0U;
  for (std::size_t index = 0U; index < packet.rays.size(); ++index) {
    const auto & ray = packet.rays[index];
    if (ray.timestamp_ns == 0U) {
      ++dropped;
      _pending_break = true;
      continue;
    }
    out.push_back(StreamSample{ray, _pending_break});
    if (metadata != nullptr && index < packet.point_metadata.size()) {
      metadata->push_back(packet.point_metadata[index]);
    }
    _pending_break = false;
  }
  return dropped;
}

}  // namespace livox_ray_source
