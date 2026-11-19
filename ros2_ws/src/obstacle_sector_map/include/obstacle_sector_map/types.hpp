#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace obstacle_sector_map
{

inline constexpr std::size_t kSectorCount{72U};
inline constexpr float kHalfPiRad{1.57079632679489661923F};
// Float rounding tolerance for the sector-boundary test. Fixed, not a tuning parameter.
inline constexpr float kSectorBoundaryEpsilonRad{2.0e-6F};
static_assert(
  kSectorBoundaryEpsilonRad > 0.0F &&
  kSectorBoundaryEpsilonRad < kHalfPiRad / static_cast<float>(kSectorCount));

enum class RayReturnKind : uint8_t
{
  kReturn,
  kQualifiedNoReturn,
  kInvalid,
};

enum class SectorState : uint8_t
{
  kUnknown,
  kObservedClear,
  kOccupied,
  kTooClose,
};

struct RaySample
{
  uint64_t timestamp_ns{0U};
  float range_m{std::numeric_limits<float>::quiet_NaN()};
  float azimuth_rad{std::numeric_limits<float>::quiet_NaN()};
  float elevation_rad{std::numeric_limits<float>::quiet_NaN()};
  bool masked{false};
  RayReturnKind return_kind{RayReturnKind::kInvalid};
};

struct SectorObservation
{
  SectorState state{SectorState::kUnknown};
  float distance_m{std::numeric_limits<float>::quiet_NaN()};
  uint32_t contributing_ray_count{0U};
  uint32_t clear_ray_count{0U};  // qualified no-return rays only
};

struct SectorMap
{
  uint64_t sample_timestamp_ns{0U};
  bool packet_continuity_valid{false};
  std::array<SectorObservation, kSectorCount> sectors{};
  uint32_t accepted_ray_count{0U};
  uint32_t rejected_ray_count{0U};
};

struct SectorMapConfig
{
  float min_distance_m;
  float max_distance_m;
  float min_elevation_rad;
  float max_elevation_rad;
  uint32_t coverage_min_rays_per_sector;

  [[nodiscard]] bool isValid() const
  {
    return std::isfinite(min_distance_m) && std::isfinite(max_distance_m) &&
           std::isfinite(min_elevation_rad) && std::isfinite(max_elevation_rad) &&
           min_distance_m > 0.0F && max_distance_m > min_distance_m &&
           min_elevation_rad > -kHalfPiRad && max_elevation_rad < kHalfPiRad &&
           min_elevation_rad < max_elevation_rad && coverage_min_rays_per_sector > 0U;
  }
};

}  // namespace obstacle_sector_map
