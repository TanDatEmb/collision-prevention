#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

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
    uint32_t clear_ray_count{0U};
    bool too_close{false};
    std::vector<float> obstacle_ranges_m;  // horizontal range of each in-band return within the range
    std::vector<float> free_up_m;          // free distance certified by each ray, upper half of the band
    std::vector<float> free_down_m;        // and lower half; a horizon ray counts in both
    uint32_t up_ray_count{0U};    // rays seen in the upper half of the band: free and obstacle (SYS-CP-029)
    uint32_t down_ray_count{0U};  // and in the lower half; a horizon ray counts in both
  };

  void rejectRay();
  void addObstacleReturn(std::size_t sector_index, float horizontal_range_m, float height_m);
  void addFreeEvidence(std::size_t sector_index, const RaySample & ray);

  SectorMapConfig _config;
  bool _packet_continuity_valid{false};
  uint64_t _sample_timestamp_ns{0U};
  uint32_t _accepted_ray_count{0U};
  uint32_t _rejected_ray_count{0U};
  std::array<SectorAccumulator, kSectorCount> _accumulators{};
};

}  // namespace obstacle_sector_map
