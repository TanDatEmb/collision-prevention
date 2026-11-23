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
constexpr uint8_t kSphericalDataType{3U};
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
  for (std::size_t index = 0U; index < dot_num; ++index) {
    const uint8_t * point = data + kHeaderBytes + index * kPointBytes;
    const uint32_t depth_mm = readLe<uint32_t>(point);
    const uint16_t theta = readLe<uint16_t>(point + 4);
    const uint16_t phi = readLe<uint16_t>(point + 6);

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
    } else if (depth_mm == 0U) {
      ray.return_kind = RayReturnKind::kQualifiedNoReturn;
      ray.range_m = std::numeric_limits<float>::quiet_NaN();
    } else {
      ray.return_kind = RayReturnKind::kReturn;
    }
    ray.timestamp_ns =
      time_usable ? packet_time_ns + (dot_num > 1U ? (index * interval_ns) / (dot_num - 1U) : 0U)
                  : 0U;
    packet.rays.push_back(ray);
  }
  packet.status = DecodeStatus::kOk;
  return packet;
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
  const DecodedPacket & packet, bool udp_break, std::vector<StreamSample> & out)
{
  _pending_break = _pending_break || udp_break;
  std::size_t dropped = 0U;
  for (const auto & ray : packet.rays) {
    if (ray.timestamp_ns == 0U) {
      ++dropped;
      _pending_break = true;
      continue;
    }
    out.push_back(StreamSample{ray, _pending_break});
    _pending_break = false;
  }
  return dropped;
}

}  // namespace livox_ray_source
