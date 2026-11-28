#pragma once

#include <cstdint>
#include <optional>

#include "livox_ray_source/ray_source.hpp"

namespace livox_ray_source
{

/// Groups a stream of rays into frames by coverage window, using the ray timestamps.
/// A window starts at the first ray with a non-zero timestamp and covers `window_ns`.
/// A frame has packet continuity only if no break was flagged and its timestamps never go backwards.
/// A backward timestamp closes the old frame as invalid and starts a new epoch. Equal timestamps are
/// retained: a packet or replay source may legitimately give several rays the same reference time.
class FrameSegmenter
{
public:
  /// Throws std::invalid_argument when `window_ns` is zero.
  explicit FrameSegmenter(uint64_t window_ns);

  /// Adds a ray; returns the finished frame when this ray starts a new window.
  [[nodiscard]] std::optional<RayFrame> push(const StreamSample & sample);

  /// Returns the frame still open, if any, and starts over.
  [[nodiscard]] std::optional<RayFrame> flush();

private:
  void startFrame();

  uint64_t _window_ns;
  RayFrame _frame;
  bool _frame_open{false};
  bool _origin_set{false};
  uint64_t _origin_ns{0U};
  uint64_t _latest_ns{0U};
};

}  // namespace livox_ray_source
