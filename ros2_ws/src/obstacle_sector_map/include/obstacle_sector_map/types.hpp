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
// Detection range limits of the map and of the wire (ICD-CP-03): 10 cm and 40 m. Fixed for every
// platform; they are not configuration.
inline constexpr float kMinDistanceM{0.10F};
inline constexpr float kMaxDistanceM{40.0F};
static_assert(kMinDistanceM > 0.0F && kMinDistanceM < kMaxDistanceM);
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

// Rounding tolerance for "the free distance reaches the qualified range". Fixed, not a tuning parameter.
inline constexpr float kFreeDistanceToleranceM{1.0e-3F};

struct SectorObservation
{
  SectorState state{SectorState::kUnknown};
  // The distance sent for the sector: the smaller of the confirmed obstacle distance and the certified
  // free distance (ADR 0020 D2). NaN for unknown and clear.
  float distance_m{std::numeric_limits<float>::quiet_NaN()};
  uint32_t contributing_ray_count{0U};
  uint32_t clear_ray_count{0U};  // rays that certify free distance: no-returns and returns outside the band
  // Diagnostics of the two parts of distance_m; NaN when the part does not exist.
  float obstacle_distance_m{std::numeric_limits<float>::quiet_NaN()};
  float free_distance_m{std::numeric_limits<float>::quiet_NaN()};
};

struct SectorMap
{
  uint64_t sample_timestamp_ns{0U};
  bool packet_continuity_valid{false};
  std::array<SectorObservation, kSectorCount> sectors{};
  uint32_t accepted_ray_count{0U};
  uint32_t rejected_ray_count{0U};
};

// Required platform values with no default (ADR 0020 D2; OPEN-005, 023, 036, 041).
struct SectorMapConfig
{
  float height_up_m;                // h_up: the band reaches this far above the vehicle, in the level frame
  float height_down_m;              // h_down: and this far below it
  float obstacle_window_m;          // dr: width of the range window that confirms an obstacle
  uint32_t obstacle_min_returns;    // n_obstacle: returns a window must hold to be an obstacle
  uint32_t free_min_rays;           // n_free: the n_free-th largest free distance of each half is certified

  [[nodiscard]] bool isValid() const
  {
    return std::isfinite(height_up_m) && height_up_m > 0.0F && std::isfinite(height_down_m) &&
           height_down_m > 0.0F && std::isfinite(obstacle_window_m) && obstacle_window_m > 0.0F &&
           obstacle_min_returns > 0U && free_min_rays > 0U;
  }
};

}  // namespace obstacle_sector_map
