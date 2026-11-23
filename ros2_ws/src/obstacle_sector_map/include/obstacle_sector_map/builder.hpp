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

  /// Starts a frame. `origin_rise_bound_m` and `origin_fall_bound_m` bound, over the rays of the frame, how far the
  /// ray origin was above and below its position at the reference time (ADR 0022 decision 1 item e, ICD-CP-06). The
  /// free distance of a ray is then certified only inside the part of the band that is still above (below) the
  /// origin: `h_up - rise` and `h_down - fall`. Both are metres. NaN means "not provided" (a source that does
  /// not level): no correction, the band keeps its full height, as a NaN setpoint is not controlled in PX4. 0 is a
  /// real bound and is applied. A bound that is negative or infinite is wrong, not absent: it leaves no room in
  /// that half, so that half certifies no free distance.
  void reset(bool packet_continuity_valid, float origin_rise_bound_m, float origin_fall_bound_m);
  /// True when addRay() takes the ray (a time, not masked, a finite direction below the zenith, and a valid
  /// return). The map timestamp is the earliest time among these rays, so a caller that must use the same
  /// reference time (the leveling step, ICD-CP-06) selects it with this function.
  [[nodiscard]] static bool accepts(const RaySample & ray);
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
  float _room_up_m{0.0F};    // h_up - rise bound of the frame: band room above the lowest-high origin
  float _room_down_m{0.0F};  // h_down - fall bound of the frame
  uint64_t _sample_timestamp_ns{0U};
  uint32_t _accepted_ray_count{0U};
  uint32_t _rejected_ray_count{0U};
  std::array<SectorAccumulator, kSectorCount> _accumulators{};
};

}  // namespace obstacle_sector_map
