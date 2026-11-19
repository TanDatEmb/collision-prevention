#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "obstacle_sector_map/types.hpp"

namespace obstacle_sector_map
{

class SectorMapBuilder
{
public:
  explicit SectorMapBuilder(const SectorMapConfig & config);

  void reset(bool packet_continuity_valid);
  void addRay(const RaySample & ray);
  [[nodiscard]] SectorMap finalize() const;

private:
  struct SectorAccumulator
  {
    uint32_t contributing_ray_count{0U};
    bool too_close{false};
    float minimum_obstacle_distance_m{std::numeric_limits<float>::infinity()};
  };

  void rejectRay();
  void updateSector(std::size_t sector_index, const RaySample & ray);

  SectorMapConfig _config;
  bool _packet_continuity_valid{false};
  uint64_t _sample_timestamp_ns{0U};
  uint32_t _accepted_ray_count{0U};
  uint32_t _rejected_ray_count{0U};
  std::array<SectorAccumulator, kSectorCount> _accumulators{};
};

}  // namespace obstacle_sector_map
