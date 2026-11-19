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

bool liesOnSectorBoundary(float wrapped_azimuth_rad, float epsilon_rad)
{
  const double sector_width_rad = static_cast<double>(kSectorWidthRad);
  const double wrapped_rad = static_cast<double>(wrapped_azimuth_rad);
  const double lower_sector = std::floor(wrapped_rad / sector_width_rad);
  const double boundary_rad = (lower_sector + 0.5) * sector_width_rad;
  const double effective_epsilon_rad =
    std::max(static_cast<double>(epsilon_rad), static_cast<double>(kMinSectorBoundaryEpsilonRad));
  return std::fabs(wrapped_rad - boundary_rad) <= effective_epsilon_rad;
}

float horizontalProjectedRange(const RaySample & ray)
{
  return ray.range_m * std::cos(ray.elevation_rad);
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
  _accumulators = {};
}

void SectorMapBuilder::rejectRay() { ++_rejected_ray_count; }

void SectorMapBuilder::addRay(const RaySample & ray)
{
  const bool direction_valid = std::isfinite(ray.azimuth_rad) && std::isfinite(ray.elevation_rad) &&
                               ray.elevation_rad >= _config.min_elevation_rad &&
                               ray.elevation_rad <= _config.max_elevation_rad;
  const bool metadata_valid = ray.timestamp_ns != 0U && !ray.masked;
  const bool return_valid =
    ray.return_kind == RayReturnKind::kQualifiedNoReturn ||
    (ray.return_kind == RayReturnKind::kReturn && std::isfinite(ray.range_m) && ray.range_m > 0.0F);

  if (!direction_valid || !metadata_valid || !return_valid) {
    rejectRay();
    return;
  }

  ++_accepted_ray_count;
  _sample_timestamp_ns = _sample_timestamp_ns == 0U
                           ? ray.timestamp_ns
                           : std::min(_sample_timestamp_ns, ray.timestamp_ns);

  const float wrapped_azimuth_rad = wrapZeroToTwoPi(ray.azimuth_rad);

  const bool has_obstacle_evidence = ray.return_kind == RayReturnKind::kReturn &&
                                     horizontalProjectedRange(ray) <= _config.max_distance_m;

  if (
    has_obstacle_evidence &&
    liesOnSectorBoundary(wrapped_azimuth_rad, _config.sector_boundary_epsilon_rad)) {
    const auto lower_index =
      static_cast<std::size_t>(std::floor(wrapped_azimuth_rad / kSectorWidthRad)) % kSectorCount;
    updateSector(lower_index, ray);
    updateSector((lower_index + 1U) % kSectorCount, ray);
    return;
  }

  updateSector(nearestSectorIndex(wrapped_azimuth_rad), ray);
}

void SectorMapBuilder::updateSector(std::size_t sector_index, const RaySample & ray)
{
  auto & accumulator = _accumulators.at(sector_index);
  ++accumulator.contributing_ray_count;

  if (ray.return_kind != RayReturnKind::kReturn) {
    return;
  }

  const float horizontal_range_m = horizontalProjectedRange(ray);

  if (horizontal_range_m < _config.min_distance_m) {
    accumulator.too_close = true;
    return;
  }

  if (horizontal_range_m <= _config.max_distance_m) {
    accumulator.minimum_obstacle_distance_m =
      std::min(accumulator.minimum_obstacle_distance_m, horizontal_range_m);
  }
}

SectorMap SectorMapBuilder::finalize() const
{
  SectorMap map{};
  map.sample_timestamp_ns = _sample_timestamp_ns;
  map.packet_continuity_valid = _packet_continuity_valid;
  map.accepted_ray_count = _accepted_ray_count;
  map.rejected_ray_count = _rejected_ray_count;

  for (std::size_t index = 0U; index < kSectorCount; ++index) {
    const auto & accumulator = _accumulators[index];
    auto & sector = map.sectors[index];
    sector.contributing_ray_count = accumulator.contributing_ray_count;

    if (accumulator.too_close) {
      sector.state = SectorState::kTooClose;
      sector.distance_m = 0.0F;
    }
    else if (std::isfinite(accumulator.minimum_obstacle_distance_m)) {
      sector.state = SectorState::kOccupied;
      sector.distance_m = accumulator.minimum_obstacle_distance_m;
    }
    else if (
      _packet_continuity_valid &&
      accumulator.contributing_ray_count >= _config.coverage_min_rays_per_sector) {
      sector.state = SectorState::kObservedClear;
    }
  }

  return map;
}

}  // namespace obstacle_sector_map
