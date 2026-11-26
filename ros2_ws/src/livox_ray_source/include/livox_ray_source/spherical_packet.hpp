#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "livox_ray_source/gyro_track.hpp"
#include "livox_ray_source/ray_source.hpp"
#include "obstacle_sector_map/types.hpp"

namespace livox_ray_source
{

/// Why a packet was not decoded.
enum class DecodeStatus : uint8_t
{
  kOk,
  kTooShort,        ///< fewer bytes than the 36 byte header
  kNotSpherical,    ///< `data_type` is not 3 (kept for the spherical-only API)
  kLengthMismatch,  ///< the size is not 36 + 10 * dot_num
  kNoPoints,        ///< `dot_num` is 0
};

/// Per-point fields that are not part of the safety map contract.
struct PointMetadata
{
  uint8_t reflectivity{0U};
  uint8_t tag{0U};
  uint8_t line{0U};
};

struct DecodedPacket
{
  DecodeStatus status{DecodeStatus::kTooShort};
  uint16_t udp_cnt{0U};
  uint8_t time_type{0U};  ///< 0 no sync (time since power on), 1 PTP, 2 GPS
  uint8_t data_type{0U};
  std::vector<obstacle_sector_map::RaySample> rays;
  std::vector<PointMetadata> point_metadata;
};

/// One complete Livox IMU sample. Acceleration is kept in the SDK's raw unit (g) until a ROS
/// boundary converts it to m/s²; the collision-prevention leveler consumes only `gyro`.
struct ImuSample
{
  GyroSample gyro{};
  std::array<double, 3> acceleration_flu_g{
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  [[nodiscard]] bool isValid() const;
};

/// Decodes one Livox Mid-360(S) point packet of data type 3 (single return, spherical): the 36 byte
/// header and `dot_num` points of depth (u32, mm), theta (u16), phi (u16), reflectivity (u8), tag
/// (u8). Layout: Livox Mid-360 protocol document 1.2.1.3, checked against the packets of a real
/// Mid-360S (96 points of 10 bytes, 996 bytes).
///
/// - Direction as the vendor driver (`livox_ros_driver2`, `ProcessSphericalPoint`) turns it into x,
///   y, z: theta and phi are in 0.01 degree; theta is the polar angle from +z and phi increases
///   from +x towards +y. So the ray azimuth is phi (counter-clockwise, FLU) and the elevation is
///   90 degrees minus theta. Values outside theta [0, 18000] and phi [0, 36000] are invalid.
/// - A depth of 0 with a valid direction is a qualified no-return (issue #172): the vendor convention
///   is (0, theta, phi) for no object in range, the vendor driver forwards such points unfiltered, and
///   a real Mid-360S sends them. No vendor document defines it beyond that, so a blocked or dirty
///   window may read the same (open, bench test). The range of a no-return is NaN. A point with a
///   direction outside the range is invalid. Tags are not used.
/// - Time: only PTP time (`time_type` 1) is used. The packet time is the time of the first point and
///   the points are equally spaced over `time_interval` (0.1 us), first to last (a real packet has
///   96 points, 475 us from first to last and 480 us between packets: 5.0 us per point). Any other
///   time type gives timestamp 0, which the sector map rejects (SYS-CP-016, ADR 0003); the
///   direction of the points is kept.
///
/// Input is the whole UDP payload.
[[nodiscard]] DecodedPacket decodeSphericalPacket(const uint8_t * data, std::size_t size);

/// Decodes a Mid-360(S) Cartesian high-resolution packet (x/y/z in millimetres, 32-bit signed).
/// Cartesian packets have no direction for an all-zero point, so that point is invalid rather than
/// a qualified no-return. The safety path therefore remains conservative when a sensor is configured
/// away from spherical mode: returns can be used, but a Cartesian zero point cannot certify free space.
[[nodiscard]] DecodedPacket decodeCartesianHighPacket(const uint8_t * data, std::size_t size);

/// Decodes a Mid-360(S) Cartesian low-resolution packet (x/y/z in centimetres, 16-bit signed).
[[nodiscard]] DecodedPacket decodeCartesianLowPacket(const uint8_t * data, std::size_t size);

/// Dispatches a supported Mid-360(S) point packet by its `data_type` (spherical, Cartesian high or
/// Cartesian low). IMU packets and unknown point layouts are rejected with `kNotSpherical`.
[[nodiscard]] DecodedPacket decodePointPacket(const uint8_t * data, std::size_t size);

/// One decoded Livox IMU packet (data type 0).
struct DecodedImuPacket
{
  bool layout_ok{false};  ///< data type 0 and exactly 36 + 24 bytes
  uint8_t time_type{0U};  ///< 0 no sync (time since power on), 1 PTP, 2 GPS
  GyroSample gyro{};      ///< valid only with layout_ok and PTP time (`time_type` 1, non-zero time)
  std::array<double, 3> acceleration_flu_g{
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  /// True only when all three raw accelerometer axes are finite and the packet has usable gyro time.
  [[nodiscard]] bool accelerationIsValid() const;

  [[nodiscard]] ImuSample toImuSample() const;
};

/// Decodes one Livox Mid-360(S) IMU packet: the 36 byte header (the same layout and time stamp as the
/// point packets) and one sample of six little-endian floats, gyro x, y, z in rad/s then acceleration
/// x, y, z in g (`LivoxLidarImuRawPoint`; units measured on a real Mid-360S, ADR 0019). Only PTP time is
/// used, as for the points (ADR 0003); the acceleration is not used (ADR 0023). Input is the whole UDP
/// payload.
[[nodiscard]] DecodedImuPacket decodeImuPacket(const uint8_t * data, std::size_t size);

/// Packet loss on the point stream through the 16 bit `udp_cnt`: a packet is a break when its
/// counter is not the previous one plus one (a gap, a duplicate, a step back); the counter wraps.
/// The first packet is not a break.
class UdpContinuityTracker
{
public:
  [[nodiscard]] bool update(uint16_t udp_cnt);

private:
  bool _has_previous{false};
  uint16_t _previous{0U};
};

/// Turns a decoded packet into stream samples for a frame segmenter.
///
/// A ray without a usable time (timestamp 0: the lidar is not PTP synchronized) is dropped here:
/// the segmenter only closes a frame on a ray with a time, so it would collect such rays without
/// bound (200,000 points per second), and the sector map would reject them anyway (SYS-CP-016,
/// ADR 0003). The drop is a gap in the stream, so the next ray that is handed on carries
/// `continuity_break`, as does the first ray after a `udp_cnt` gap; a break is never lost because
/// the packet it belonged to had no usable ray.
class PacketSampler
{
public:
  /// Appends the rays of `packet` that have a time to `out`; `udp_break` is true when packets were
  /// lost just before this one. When `metadata` is supplied, it receives point metadata in the same
  /// order as the forwarded samples. Returns the number of rays dropped.
  std::size_t append(
    const DecodedPacket & packet, bool udp_break, std::vector<StreamSample> & out,
    std::vector<PointMetadata> * metadata = nullptr);

private:
  bool _pending_break{false};
};

}  // namespace livox_ray_source
