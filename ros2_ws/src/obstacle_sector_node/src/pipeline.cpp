#include "obstacle_sector_node/pipeline.hpp"

#include <stdexcept>
#include <utility>

#include "px4_obstacle_bridge/encoder.hpp"

namespace obstacle_sector_node
{

SectorMapPipeline::SectorMapPipeline(
  std::unique_ptr<livox_ray_source::RaySource> source,
  const obstacle_sector_map::SectorMapConfig & config)
: _source(std::move(source)), _config(config), _builder(config)
{
  if (!_source) {
    throw std::invalid_argument("pipeline needs a ray source");
  }
}

StepResult SectorMapPipeline::step()
{
  const auto frame = _source->nextFrame();

  if (!frame.has_value()) {
    return StepResult{StepStatus::kSourceEnded, std::nullopt};
  }

  ++_stats.frames;

  _builder.reset(
    frame->packet_continuity_valid, static_cast<float>(frame->origin_rise_bound_m),
    static_cast<float>(frame->origin_fall_bound_m));
  for (const auto & ray : frame->rays) {
    _builder.addRay(ray);
  }

  auto message = px4_obstacle_bridge::encodeObstacleDistance(_builder.finalize(), _config);

  if (!message.has_value()) {
    ++_stats.encode_failures;
    return StepResult{StepStatus::kEncodeFailed, std::nullopt};
  }

  ++_stats.messages;
  return StepResult{StepStatus::kMessage, std::move(message)};
}

}  // namespace obstacle_sector_node
