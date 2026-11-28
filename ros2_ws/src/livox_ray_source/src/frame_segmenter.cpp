#include "livox_ray_source/frame_segmenter.hpp"

#include <stdexcept>
#include <utility>

namespace livox_ray_source
{

FrameSegmenter::FrameSegmenter(uint64_t window_ns) : _window_ns(window_ns)
{
  if (window_ns == 0U) {
    throw std::invalid_argument("coverage window must be non-zero");
  }
}

void FrameSegmenter::startFrame()
{
  _frame = RayFrame{};
  _frame.packet_continuity_valid = true;
  _frame_open = true;
  _origin_set = false;
  _origin_ns = 0U;
  _latest_ns = 0U;
}

std::optional<RayFrame> FrameSegmenter::push(const StreamSample & sample)
{
  std::optional<RayFrame> finished;
  const uint64_t timestamp_ns = sample.ray.timestamp_ns;

  if (!_frame_open) {
    startFrame();
  }

  const bool timestamp_goes_back =
    _origin_set && timestamp_ns != 0U && _latest_ns != 0U && timestamp_ns < _latest_ns;
  const bool starts_next_window =
    _origin_set && timestamp_ns != 0U && timestamp_ns >= _origin_ns &&
    timestamp_ns - _origin_ns >= _window_ns;
  const bool crosses_discontinuity = timestamp_goes_back || starts_next_window;

  // A continuity flag describes the gap before this ray. If that ray also starts a new epoch or
  // coverage window, the gap invalidates the frame that is about to be returned, not the new frame.
  if (crosses_discontinuity) {
    if (timestamp_goes_back || sample.continuity_break) {
      _frame.packet_continuity_valid = false;
    }
    finished = std::move(_frame);
    _frame_open = false;
    startFrame();
  }

  if (timestamp_ns != 0U) {
    if (!_origin_set) {
      _origin_set = true;
      _origin_ns = timestamp_ns;
    }
    _latest_ns = timestamp_ns;
  }

  if (sample.continuity_break && !crosses_discontinuity) {
    _frame.packet_continuity_valid = false;
  }

  _frame.rays.push_back(sample.ray);
  return finished;
}

std::optional<RayFrame> FrameSegmenter::flush()
{
  if (!_frame_open) {
    return std::nullopt;
  }

  _frame_open = false;
  return std::move(_frame);
}

}  // namespace livox_ray_source
