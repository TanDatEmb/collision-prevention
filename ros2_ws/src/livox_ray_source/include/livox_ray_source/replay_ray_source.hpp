#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "livox_ray_source/frame_segmenter.hpp"
#include "livox_ray_source/ray_source.hpp"

namespace livox_ray_source
{

/// RaySource over a recorded or synthetic stream, for tests and replay.
class ReplayRaySource : public RaySource
{
public:
  /// Throws std::invalid_argument when `window_ns` is zero.
  ReplayRaySource(std::vector<StreamSample> samples, uint64_t window_ns);

  [[nodiscard]] std::optional<RayFrame> nextFrame() override;

private:
  std::vector<StreamSample> _samples;
  FrameSegmenter _segmenter;
  std::size_t _next_index{0U};
  bool _flushed{false};
};

}  // namespace livox_ray_source
