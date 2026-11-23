#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "obstacle_sector_map/types.hpp"

namespace livox_ray_source
{

/// One frame for the sector map (ICD-CP-06): the rays of one coverage window and one
/// packet-continuity flag for the whole frame.
struct RayFrame
{
  std::vector<obstacle_sector_map::RaySample> rays;
  bool packet_continuity_valid{false};
  /// Bounds, metres, not negative, of how far the sensor origin was above and below its position at the
  /// reference time over the rays of the frame (ADR 0022 decision 1 item e, ICD-CP-06). The sector map builder
  /// takes them as the loss of band room. As in PX4 setpoints, NaN means "not provided": the source does not
  /// level, so there is no reference position to move from and no correction is made. 0 is a real value: the
  /// origin did not move vertically and the full band room is used. LevelingRaySource sets both to finite values.
  double origin_rise_bound_m{std::numeric_limits<double>::quiet_NaN()};
  double origin_fall_bound_m{std::numeric_limits<double>::quiet_NaN()};
};

/// A ray as it arrives from a stream. `continuity_break` marks that packets were lost right
/// before this ray.
struct StreamSample
{
  obstacle_sector_map::RaySample ray;
  bool continuity_break{false};
};

/// Source of ray frames. Implementations preserve the direction of returns and no-returns.
class RaySource
{
public:
  virtual ~RaySource() = default;

  /// Next frame, or std::nullopt when the source has no more frames.
  [[nodiscard]] virtual std::optional<RayFrame> nextFrame() = 0;
};

}  // namespace livox_ray_source
