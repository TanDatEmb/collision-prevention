#include "obstacle_sector_node/ring_cloud_source.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>

namespace obstacle_sector_node
{

namespace
{

using obstacle_sector_map::RayReturnKind;
using obstacle_sector_map::RaySample;

constexpr uint8_t kFloat32{sensor_msgs::msg::PointField::FLOAT32};

struct FieldOffsets
{
  uint32_t x{0U};
  uint32_t y{0U};
  uint32_t z{0U};
};

bool findFloatField(
  const sensor_msgs::msg::PointCloud2 & cloud, const char * name, uint32_t & offset)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      offset = field.offset;
      return field.datatype == kFloat32 && field.count == 1U;
    }
  }
  return false;
}

float readFloat(const uint8_t * bytes)
{
  float value = 0.0F;
  std::memcpy(&value, bytes, sizeof(float));
  return value;
}

bool isPositiveFinite(double value) { return std::isfinite(value) && value > 0.0; }

}  // namespace

bool RingCloudLayout::valid() const
{
  return azimuth_count >= 1U && ring_count >= 1U && std::isfinite(azimuth_min_rad) &&
         isPositiveFinite(azimuth_step_rad) && std::isfinite(elevation_min_rad) &&
         isPositiveFinite(elevation_step_rad) && std::isfinite(range_min_m) && range_min_m > 0.0F &&
         std::isfinite(range_max_m) && range_min_m < range_max_m &&
         isPositiveFinite(max_index_angle_error_rad);
}

RingCloudResult ringCloudToFrame(
  const sensor_msgs::msg::PointCloud2 & cloud, const RingCloudLayout & layout,
  bool packet_continuity_valid)
{
  if (!layout.valid()) {
    throw std::invalid_argument("ringCloudToFrame needs a valid layout");
  }

  RingCloudResult result;
  result.frame.packet_continuity_valid = packet_continuity_valid;

  FieldOffsets offsets;
  const bool fields_ok = findFloatField(cloud, "x", offsets.x) &&
                         findFloatField(cloud, "y", offsets.y) &&
                         findFloatField(cloud, "z", offsets.z);
  const std::size_t point_step = cloud.point_step;
  const std::size_t row_step = cloud.row_step;
  const std::size_t last_field_end = static_cast<std::size_t>(
                                       std::max({offsets.x, offsets.y, offsets.z})) +
                                     sizeof(float);
  const bool shape_ok =
    fields_ok && !cloud.is_bigendian && cloud.width == layout.azimuth_count &&
    cloud.height == layout.ring_count && point_step >= last_field_end &&
    row_step >= static_cast<std::size_t>(cloud.width) * point_step &&
    cloud.data.size() >= static_cast<std::size_t>(cloud.height) * row_step;
  if (!shape_ok) {
    result.stats.shape_ok = false;
    result.frame.packet_continuity_valid = false;
    return result;
  }

  const uint64_t header_ns = static_cast<uint64_t>(cloud.header.stamp.sec) * 1'000'000'000U +
                             static_cast<uint64_t>(cloud.header.stamp.nanosec);
  const bool scan_covers_map_range =
    layout.range_max_m >= obstacle_sector_map::kMaxDistanceM;

  result.frame.rays.reserve(static_cast<std::size_t>(layout.azimuth_count) * layout.ring_count);
  for (uint32_t ring = 0U; ring < layout.ring_count; ++ring) {
    const double elevation = layout.elevation_min_rad + ring * layout.elevation_step_rad;
    const double cos_el = std::cos(elevation);
    const double sin_el = std::sin(elevation);
    for (uint32_t step = 0U; step < layout.azimuth_count; ++step) {
      const double azimuth = layout.azimuth_min_rad + step * layout.azimuth_step_rad;
      // Index direction in the sensor frame (FLU).
      const std::array<double, 3> direction{
        cos_el * std::cos(azimuth), cos_el * std::sin(azimuth), sin_el};

      const uint8_t * point =
        cloud.data.data() + static_cast<std::size_t>(ring) * row_step +
        static_cast<std::size_t>(step) * point_step;
      const std::array<double, 3> xyz{
        static_cast<double>(readFloat(point + offsets.x)),
        static_cast<double>(readFloat(point + offsets.y)),
        static_cast<double>(readFloat(point + offsets.z))};

      RaySample ray;
      ray.timestamp_ns = header_ns;
      ray.azimuth_rad = static_cast<float>(azimuth);
      ray.elevation_rad = static_cast<float>(elevation);
      ray.masked = false;
      ray.range_m = std::numeric_limits<float>::quiet_NaN();
      ray.return_kind = RayReturnKind::kInvalid;

      const bool any_nan = std::isnan(xyz[0]) || std::isnan(xyz[1]) || std::isnan(xyz[2]);
      const bool all_finite =
        std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]);
      if (all_finite) {
        const double range = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
        // Angle between the point direction and the index direction, exact for small angles.
        const double dot =
          xyz[0] * direction[0] + xyz[1] * direction[1] + xyz[2] * direction[2];
        const std::array<double, 3> cross{
          xyz[1] * direction[2] - xyz[2] * direction[1],
          xyz[2] * direction[0] - xyz[0] * direction[2],
          xyz[0] * direction[1] - xyz[1] * direction[0]};
        const double cross_norm =
          std::sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
        const double angle_error = std::atan2(cross_norm, dot);
        ray.range_m = static_cast<float>(range);
        if (range <= 0.0) {
          // a point at the origin has no direction
        }
        else if (angle_error > layout.max_index_angle_error_rad) {
          ++result.stats.index_mismatch;
        }
        else if (range < static_cast<double>(layout.range_min_m)) {
          // nearer than the minimum range
        }
        else if (range >= static_cast<double>(layout.range_max_m)) {
          if (scan_covers_map_range) {
            ray.return_kind = RayReturnKind::kQualifiedNoReturn;
          }
        }
        else {
          ray.return_kind = RayReturnKind::kReturn;
        }
      }
      else if (!any_nan) {
        // Infinite coordinates: the range is +inf or -inf, and the sign of the dominant component of
        // the index direction tells which. The dominant component must be the infinite one.
        std::size_t dominant = 0U;
        for (std::size_t axis = 1U; axis < 3U; ++axis) {
          if (std::abs(direction[axis]) > std::abs(direction[dominant])) {
            dominant = axis;
          }
        }
        if (std::isinf(xyz[dominant])) {
          const bool plus_infinity = (xyz[dominant] > 0.0) == (direction[dominant] > 0.0);
          if (plus_infinity) {
            ray.range_m = std::numeric_limits<float>::infinity();
            if (scan_covers_map_range) {
              ray.return_kind = RayReturnKind::kQualifiedNoReturn;
            }
          }
        }
      }

      switch (ray.return_kind) {
        case RayReturnKind::kReturn:
          ++result.stats.returns;
          break;
        case RayReturnKind::kQualifiedNoReturn:
          ++result.stats.no_returns;
          break;
        default:
          ++result.stats.invalid;
          break;
      }
      ++result.stats.points;
      result.frame.rays.push_back(ray);
    }
  }
  return result;
}

}  // namespace obstacle_sector_node
