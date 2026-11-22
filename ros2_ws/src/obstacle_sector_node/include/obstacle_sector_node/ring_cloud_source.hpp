#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "livox_ray_source/ray_source.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace obstacle_sector_node
{

/// Layout of an organised multi-ring cloud: `width` azimuth steps by `height` rings, ring-major
/// (index = ring * width + azimuth step). Azimuth is counter-clockwise from the sensor front (FLU),
/// elevation is positive up, both in the SENSOR frame; the mount and the leveling are applied later
/// (ADR 0020 D1). Every field has no default: a layout is configuration of the sensor or of the
/// simulated sensor, never a guess (NaN and zero are invalid).
struct RingCloudLayout
{
  uint32_t azimuth_count{0U};
  double azimuth_min_rad{std::numeric_limits<double>::quiet_NaN()};
  double azimuth_step_rad{std::numeric_limits<double>::quiet_NaN()};
  uint32_t ring_count{0U};
  double elevation_min_rad{std::numeric_limits<double>::quiet_NaN()};
  double elevation_step_rad{std::numeric_limits<double>::quiet_NaN()};
  /// Sensor range limits. A point nearer than `range_min_m` is invalid; a point at or beyond
  /// `range_max_m` and a no-hit are a qualified no-return only when `range_max_m` is at least the map's
  /// own range (`obstacle_sector_map::kMaxDistanceM`), as for the planar scan.
  float range_min_m{std::numeric_limits<float>::quiet_NaN()};
  float range_max_m{std::numeric_limits<float>::quiet_NaN()};
  /// Self-check of the index: a finite point whose direction differs from its index direction by more
  /// than this angle is invalid (a bench value of the simulated sensor, not a safety threshold).
  double max_index_angle_error_rad{std::numeric_limits<double>::quiet_NaN()};

  /// All values finite and positive, counts at least 1, `range_min_m < range_max_m`.
  [[nodiscard]] bool valid() const;
};

/// A frame and what the adapter saw in the cloud it came from; the counts are for logs and tests.
struct RingCloudResult
{
  struct Stats
  {
    /// False when the cloud does not match the layout (width, height, fields, byte order, data size).
    /// The frame then has no rays, so nothing in it can be read as free space.
    bool shape_ok{true};
    std::size_t points{0U};
    std::size_t returns{0U};
    std::size_t no_returns{0U};
    std::size_t invalid{0U};
    /// Finite points whose direction does not match their index (counted in `invalid` too).
    std::size_t index_mismatch{0U};
  };

  livox_ray_source::RayFrame frame;
  Stats stats;
};

/// One organised multi-ring `PointCloud2` as one ray frame.
///
/// Azimuth and elevation of every ray come from the INDEX and the layout, never from x, y, z: a
/// no-hit point has infinite coordinates (HZ-CP-013), and the stock `ring` field of the Gazebo cloud
/// is always 0. The cloud gives only the range and the no-hit state.
/// - All three coordinates finite: a return when the range is in `[range_min_m, range_max_m)` and the
///   point direction matches the index direction; at or beyond `range_max_m`: see the layout.
/// - Infinite coordinates: the sign of the dominant component of the index direction tells whether the
///   range is `+inf` (nothing seen, a no-return) or `-inf` (nearer than the minimum, invalid). Any NaN,
///   or a dominant component that is not infinite, is invalid.
/// - The ray time is the header time (a zero header time stays zero and is rejected downstream,
///   SYS-CP-016).
/// Throws std::invalid_argument for an invalid layout.
[[nodiscard]] RingCloudResult ringCloudToFrame(
  const sensor_msgs::msg::PointCloud2 & cloud, const RingCloudLayout & layout,
  bool packet_continuity_valid);

}  // namespace obstacle_sector_node
