#include "px4_obstacle_bridge/encoder.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace px4_obstacle_bridge
{
namespace
{

using obstacle_sector_map::kSectorCount;
using obstacle_sector_map::SectorState;

constexpr float kCentimetresPerMetre{100.0F};
constexpr float kIncrementDegrees{360.0F / static_cast<float>(kSectorCount)};
constexpr uint16_t kUnknownDistance{std::numeric_limits<uint16_t>::max()};
constexpr uint16_t kLargestUnambiguousMaxDistance{kUnknownDistance - 2U};

std::optional<uint16_t> metresToWireCentimetres(float distance_m)
{
  const float scaled_distance = distance_m * kCentimetresPerMetre;

  if (
    !std::isfinite(scaled_distance) || scaled_distance < 0.0F ||
    scaled_distance > static_cast<float>(kLargestUnambiguousMaxDistance)) {
    return std::nullopt;
  }

  return static_cast<uint16_t>(std::floor(scaled_distance));
}

}  // namespace

std::optional<px4_msgs::msg::ObstacleDistance> encodeObstacleDistance(
  const obstacle_sector_map::SectorMap & sector_map,
  const obstacle_sector_map::SectorMapConfig & config)
{
  if (!config.isValid() || sector_map.sample_timestamp_ns == 0U) {
    return std::nullopt;
  }

  const uint64_t timestamp_us = sector_map.sample_timestamp_ns / 1'000U;

  if (timestamp_us == 0U) {
    return std::nullopt;
  }

  const auto minimum_cm = metresToWireCentimetres(config.min_distance_m);
  const auto maximum_cm = metresToWireCentimetres(config.max_distance_m);

  if (
    !minimum_cm.has_value() || !maximum_cm.has_value() || *minimum_cm == 0U ||
    *minimum_cm >= *maximum_cm) {
    return std::nullopt;
  }

  px4_msgs::msg::ObstacleDistance message{};
  message.timestamp = timestamp_us;
  message.frame = px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD;
  message.sensor_type = px4_msgs::msg::ObstacleDistance::MAV_DISTANCE_SENSOR_LASER;
  message.increment = kIncrementDegrees;
  message.min_distance = *minimum_cm;
  message.max_distance = *maximum_cm;
  message.angle_offset = 0.0F;
  message.distances.fill(kUnknownDistance);

  for (std::size_t flu_index = 0U; flu_index < kSectorCount; ++flu_index) {
    const auto & sector = sector_map.sectors[flu_index];
    const std::size_t frd_index = (kSectorCount - flu_index) % kSectorCount;

    switch (sector.state) {
      case SectorState::kUnknown:
        message.distances[frd_index] = kUnknownDistance;
        break;

      case SectorState::kObservedClear:
        if (
          !sector_map.packet_continuity_valid ||
          sector.clear_ray_count < config.coverage_min_rays_per_sector) {
          return std::nullopt;
        }

        message.distances[frd_index] = static_cast<uint16_t>(*maximum_cm + 1U);
        break;

      case SectorState::kTooClose:
        message.distances[frd_index] = 0U;
        break;

      case SectorState::kOccupied: {
        if (
          !std::isfinite(sector.distance_m) || sector.distance_m < config.min_distance_m ||
          sector.distance_m > config.max_distance_m) {
          return std::nullopt;
        }

        const auto occupied_cm = metresToWireCentimetres(sector.distance_m);

        if (!occupied_cm.has_value()) {
          return std::nullopt;
        }

        message.distances[frd_index] = *occupied_cm;
        break;
      }

      default:
        return std::nullopt;
    }
  }

  return message;
}

}  // namespace px4_obstacle_bridge
