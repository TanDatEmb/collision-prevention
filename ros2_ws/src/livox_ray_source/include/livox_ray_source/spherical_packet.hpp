#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "obstacle_sector_map/types.hpp"

namespace livox_ray_source
{

/// Why a packet was not decoded.
enum class DecodeStatus : uint8_t
{
  kOk,
  kTooShort,        ///< fewer bytes than the 36 byte header
  kNotSpherical,    ///< `data_type` is not 3 (IMU, Cartesian 32 or 16 bit)
  kLengthMismatch,  ///< the size is not 36 + 10 * dot_num
  kNoPoints,        ///< `dot_num` is 0
};

struct DecodedPacket
{
  DecodeStatus status{DecodeStatus::kTooShort};
  uint16_t udp_cnt{0U};
  uint8_t time_type{0U};  ///< 0 no sync (time since power on), 1 PTP, 2 GPS
  std::vector<obstacle_sector_map::RaySample> rays;
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
/// - A depth of 0 is invalid, never a return and never free: the protocol does not say what it
///   means and the rule for a qualified no-return is open (OPEN-022, ADR 0004). Tags are not used.
/// - Time: only PTP time (`time_type` 1) is used. The packet time is the time of the first point and
///   the points are equally spaced over `time_interval` (0.1 us), first to last (a real packet has
///   96 points, 475 us from first to last and 480 us between packets: 5.0 us per point). Any other
///   time type gives timestamp 0, which the sector map rejects (SYS-CP-016, ADR 0003); the
///   direction of the points is kept.
///
/// Input is the whole UDP payload.
[[nodiscard]] DecodedPacket decodeSphericalPacket(const uint8_t * data, std::size_t size);

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

}  // namespace livox_ray_source
