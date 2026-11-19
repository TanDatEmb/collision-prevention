#include "livox_ray_source/replay_ray_source.hpp"

#include <utility>

namespace livox_ray_source
{

ReplayRaySource::ReplayRaySource(std::vector<StreamSample> samples, uint64_t window_ns)
: _samples(std::move(samples)), _segmenter(window_ns)
{
}

std::optional<RayFrame> ReplayRaySource::nextFrame()
{
  while (_next_index < _samples.size()) {
    auto finished = _segmenter.push(_samples[_next_index++]);

    if (finished.has_value()) {
      return finished;
    }
  }

  if (!_flushed) {
    _flushed = true;
    return _segmenter.flush();
  }

  return std::nullopt;
}

}  // namespace livox_ray_source
