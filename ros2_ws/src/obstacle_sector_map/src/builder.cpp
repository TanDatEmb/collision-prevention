#include "obstacle_sector_map/builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace obstacle_sector_map
{
namespace
{

constexpr float kPi{3.14159265358979323846F};
constexpr float kTwoPi{2.0F * kPi};
constexpr float kSectorWidthRad{kTwoPi / static_cast<float>(kSectorCount)};

// Whether a sector belongs to the rear class: its centre bearing from the front (0 to pi) is beyond `rear_start_rad`.
bool sectorIsRear(std::size_t index, float rear_start_rad)
{
  const float centre = static_cast<float>(index) * kSectorWidthRad;
  const float bearing = std::min(centre, kTwoPi - centre);
  return bearing > rear_start_rad;
}

float wrapZeroToTwoPi(float angle_rad)
{
  float wrapped = std::fmod(angle_rad, kTwoPi);

  if (wrapped < 0.0F) {
    wrapped += kTwoPi;
  }

  return wrapped;
}

std::size_t nearestSectorIndex(float wrapped_azimuth_rad)
{
  const auto rounded_index = static_cast<std::size_t>(
    std::floor((wrapped_azimuth_rad + 0.5F * kSectorWidthRad) / kSectorWidthRad));
  return rounded_index % kSectorCount;
}

bool liesOnSectorBoundary(float wrapped_azimuth_rad)
{
  const double sector_width_rad = static_cast<double>(kSectorWidthRad);
  const double wrapped_rad = static_cast<double>(wrapped_azimuth_rad);
  const double lower_sector = std::floor(wrapped_rad / sector_width_rad);
  const double boundary_rad = (lower_sector + 0.5) * sector_width_rad;
  return std::fabs(wrapped_rad - boundary_rad) <= static_cast<double>(kSectorBoundaryEpsilonRad);
}

float horizontalProjectedRange(const RaySample & ray)
{
  return ray.range_m * std::cos(ray.elevation_rad);
}

// Near edge of the nearest window of width `window_m` that holds at least `min_returns` ranges;
// NaN when there is none. A few isolated returns never fill a window.
float confirmedObstacleDistance(std::vector<float> ranges, float window_m, uint32_t min_returns)
{
  if (ranges.size() < min_returns) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::sort(ranges.begin(), ranges.end());
  const std::size_t last_start = ranges.size() - min_returns;

  for (std::size_t start = 0U; start <= last_start; ++start) {
    if (ranges[start + min_returns - 1U] - ranges[start] <= window_m) {
      return ranges[start];
    }
  }

  return std::numeric_limits<float>::quiet_NaN();
}

}  // namespace

SectorMapBuilder::SectorMapBuilder(const SectorMapConfig & config) : _config(config)
{
  if (!_config.isValid()) {
    throw std::invalid_argument("invalid sector-map configuration");
  }

  reset(false);
}

void SectorMapBuilder::reset(bool packet_continuity_valid)
{
  _packet_continuity_valid = packet_continuity_valid;
  _sample_timestamp_ns = 0U;
  _accepted_ray_count = 0U;
  _rejected_ray_count = 0U;

  for (auto & accumulator : _accumulators) {
    accumulator.contributing_ray_count = 0U;
    accumulator.clear_ray_count = 0U;
    accumulator.too_close = false;
    accumulator.up_ray_count = 0U;
    accumulator.down_ray_count = 0U;
    accumulator.obstacle_ranges_m.clear();
  }
}

void SectorMapBuilder::rejectRay() { ++_rejected_ray_count; }

bool SectorMapBuilder::accepts(const RaySample & ray)
{
  const bool direction_valid = std::isfinite(ray.azimuth_rad) && std::isfinite(ray.elevation_rad) &&
                               std::fabs(ray.elevation_rad) < kHalfPiRad;
  const bool metadata_valid = ray.timestamp_ns != 0U && !ray.masked;
  const bool return_valid =
    ray.return_kind == RayReturnKind::kQualifiedNoReturn ||
    (ray.return_kind == RayReturnKind::kReturn && std::isfinite(ray.range_m) && ray.range_m > 0.0F);

  return direction_valid && metadata_valid && return_valid;
}

void SectorMapBuilder::addRay(const RaySample & ray)
{
  if (!accepts(ray)) {
    rejectRay();
    return;
  }

  ++_accepted_ray_count;
  _sample_timestamp_ns = _sample_timestamp_ns == 0U
                           ? ray.timestamp_ns
                           : std::min(_sample_timestamp_ns, ray.timestamp_ns);

  const float wrapped_azimuth_rad = wrapZeroToTwoPi(ray.azimuth_rad);
  const std::size_t nearest = nearestSectorIndex(wrapped_azimuth_rad);

  if (ray.return_kind == RayReturnKind::kReturn) {
    const float horizontal_range_m = horizontalProjectedRange(ray);
    const float height_m = ray.range_m * std::sin(ray.elevation_rad);
    // ADR 0024 decision 3: with an attitude error up to the allowance the return may lie anywhere in
    // [height - slack, height + slack]; it is obstacle evidence when that interval meets the band.
    const float slack_m = horizontal_range_m * std::tan(_config.attitude_allowance_rad);
    const bool in_band =
      height_m - slack_m <= _config.height_up_m && height_m + slack_m >= -_config.height_down_m;

    if (in_band && horizontal_range_m <= kMaxDistanceM) {
      // Obstacle evidence: a return inside the band within the qualified range. On a sector boundary it
      // counts for both neighbours.
      if (liesOnSectorBoundary(wrapped_azimuth_rad)) {
        const auto lower_index =
          static_cast<std::size_t>(std::floor(wrapped_azimuth_rad / kSectorWidthRad)) % kSectorCount;
        addObstacleReturn(lower_index, horizontal_range_m, height_m);
        addObstacleReturn((lower_index + 1U) % kSectorCount, horizontal_range_m, height_m);
      } else {
        addObstacleReturn(nearest, horizontal_range_m, height_m);
      }
      return;
    }
  }

  // Not obstacle evidence: the ray proves the band empty along its path (ADR 0020 D2).
  addFreeEvidence(nearest, ray);
}

void SectorMapBuilder::addObstacleReturn(
  std::size_t sector_index, float horizontal_range_m, float height_m)
{
  auto & accumulator = _accumulators.at(sector_index);
  ++accumulator.contributing_ray_count;
  // The ray was seen in the half it lies in (a return at the horizon is in both halves).
  if (height_m >= 0.0F) {
    ++accumulator.up_ray_count;
  }
  if (height_m <= 0.0F) {
    ++accumulator.down_ray_count;
  }

  if (horizontal_range_m < kMinDistanceM) {
    accumulator.too_close = true;
    return;
  }

  accumulator.obstacle_ranges_m.push_back(horizontal_range_m);
}

void SectorMapBuilder::addFreeEvidence(std::size_t sector_index, const RaySample & ray)
{
  // Not obstacle evidence. Under ADR 0024 a ray certifies no distance of its own: it counts for the coverage
  // of the half it points into (a horizon ray counts in both).
  auto & accumulator = _accumulators.at(sector_index);
  ++accumulator.contributing_ray_count;
  ++accumulator.clear_ray_count;
  if (ray.elevation_rad >= 0.0F) {
    ++accumulator.up_ray_count;
  }
  if (ray.elevation_rad <= 0.0F) {
    ++accumulator.down_ray_count;
  }
}

SectorMap SectorMapBuilder::finalize() const
{
  SectorMap map{};
  map.sample_timestamp_ns = _sample_timestamp_ns;
  map.packet_continuity_valid = _packet_continuity_valid;
  map.accepted_ray_count = _accepted_ray_count;
  map.rejected_ray_count = _rejected_ray_count;

  const float nan = std::numeric_limits<float>::quiet_NaN();

  for (std::size_t index = 0U; index < kSectorCount; ++index) {
    const auto & accumulator = _accumulators[index];
    auto & sector = map.sectors[index];
    sector.contributing_ray_count = accumulator.contributing_ray_count;
    sector.clear_ray_count = accumulator.clear_ray_count;
    const bool rear = sectorIsRear(index, _config.rear_start_rad);
    const bool both_halves_seen = accumulator.up_ray_count >= _config.free_min_rays &&
                                  (rear || accumulator.down_ray_count >= _config.free_min_rays);
    sector.up_ray_count = accumulator.up_ray_count;
    sector.down_ray_count = accumulator.down_ray_count;
    sector.rear_only = rear;
    sector.coverage_ok = both_halves_seen;

    if (accumulator.too_close) {
      sector.state = SectorState::kTooClose;
      sector.distance_m = 0.0F;
      continue;
    }

    const float obstacle_m = confirmedObstacleDistance(
      accumulator.obstacle_ranges_m, _config.obstacle_window_m, _config.obstacle_min_returns);
    sector.obstacle_distance_m = obstacle_m;
    const bool has_obstacle = std::isfinite(obstacle_m);

    // Coverage (SYS-CP-029, issue #148): a half of the band that holds fewer than n_free rays, free or
    // obstacle, is outside the field of view (the vehicle is pitched or rolled past the margin), not
    // filled by a wall. The sector is then unknown, cap 0, even when an obstacle is confirmed farther
    // away: with no evidence in a half the map must not let the cap rise on the data it still has.
    // A frame without packet continuity keeps the old rule (credible obstacle returns stay occupied).
    // Rear class (issue #141): beyond rear_start_rad the upper half alone is required and certifies; obstacles
    // below the lidar there are not seen when the lower half is out of view, a stated non-claim (ADR 0020 D3).
    if (_packet_continuity_valid && !both_halves_seen) {
      continue;
    }

    // Free distance (ADR 0024 decision 1): a covered sector is certified free to the range the sampling
    // resolves, the platform value certified_range_m. It needs packet continuity, like the old clear rule.
    float free_m = nan;

    if (_packet_continuity_valid) {
      free_m = _config.certified_range_m;
      sector.free_distance_m = free_m;
    }

    const bool free_certified = std::isfinite(free_m) && free_m > 0.0F;

    if (!has_obstacle && !free_certified) {
      continue;  // nothing confirmed and nothing certified: unknown, the cap is 0
    }

    if (!has_obstacle && free_m >= kMaxDistanceM - kFreeDistanceToleranceM) {
      sector.state = SectorState::kObservedClear;
      continue;
    }

    const float distance_m =
      has_obstacle ? (free_certified ? std::min(obstacle_m, free_m) : obstacle_m) : free_m;

    if (distance_m < kMinDistanceM) {
      sector.state = SectorState::kTooClose;
      sector.distance_m = 0.0F;
    } else {
      sector.state = SectorState::kOccupied;
      sector.distance_m = distance_m;
    }
  }

  return map;
}

}  // namespace obstacle_sector_map
