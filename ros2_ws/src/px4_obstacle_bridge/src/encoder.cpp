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
constexpr uint16_t kMinDistanceCm{
  static_cast<uint16_t>(obstacle_sector_map::kMinDistanceM * kCentimetresPerMetre)};
constexpr uint16_t kMaxDistanceCm{
  static_cast<uint16_t>(obstacle_sector_map::kMaxDistanceM * kCentimetresPerMetre)};
static_assert(kMinDistanceCm > 0U && kMinDistanceCm < kMaxDistanceCm);
static_assert(kMaxDistanceCm <= kLargestUnambiguousMaxDistance);  // max + 1 stays below unknown

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

  px4_msgs::msg::ObstacleDistance message{};
  message.timestamp = timestamp_us;
  message.frame = px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD;
  message.sensor_type = px4_msgs::msg::ObstacleDistance::MAV_DISTANCE_SENSOR_LASER;
  message.increment = kIncrementDegrees;
  message.min_distance = kMinDistanceCm;
  message.max_distance = kMaxDistanceCm;
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
        // Clear means the certified free distance reaches the qualified range (ADR 0020 D2).
        if (
          !sector_map.packet_continuity_valid || !std::isfinite(sector.free_distance_m) ||
          sector.free_distance_m < obstacle_sector_map::kMaxDistanceM -
                                     obstacle_sector_map::kFreeDistanceToleranceM) {
          return std::nullopt;
        }

        message.distances[frd_index] = static_cast<uint16_t>(kMaxDistanceCm + 1U);
        break;

      case SectorState::kTooClose:
        message.distances[frd_index] = 0U;
        break;

      case SectorState::kOccupied: {
        if (
          !std::isfinite(sector.distance_m) ||
          sector.distance_m < obstacle_sector_map::kMinDistanceM ||
          sector.distance_m > obstacle_sector_map::kMaxDistanceM) {
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
