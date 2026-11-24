#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "livox_ray_source/ray_source.hpp"
#include "obstacle_sector_map/builder.hpp"
#include "px4_msgs/msg/obstacle_distance.hpp"

namespace obstacle_sector_node
{

enum class StepStatus : uint8_t
{
  kSourceEnded,
  kEncodeFailed,
  kMessage,
};

struct StepResult
{
  StepStatus status{StepStatus::kSourceEnded};
  std::optional<px4_msgs::msg::ObstacleDistance> message;
};

struct PipelineStats
{
  uint64_t frames{0U};
  uint64_t messages{0U};
  uint64_t encode_failures{0U};
  uint64_t rejected_rays{0U};  // rays the map did not use, self-masked rays included (summed over frames)
};

/// Frame to ObstacleDistance, without ROS: one frame of a RaySource goes through the sector map
/// builder and the encoder (ICD-CP-06 then ICD-CP-03). An encode failure produces no message and
/// is counted; nothing is inferred to replace it.
class SectorMapPipeline
{
public:
  /// Throws std::invalid_argument for a null source or an invalid config.
  SectorMapPipeline(
    std::unique_ptr<livox_ray_source::RaySource> source,
    const obstacle_sector_map::SectorMapConfig & config);

  [[nodiscard]] StepResult step();

  [[nodiscard]] const PipelineStats & stats() const { return _stats; }

private:
  std::unique_ptr<livox_ray_source::RaySource> _source;
  obstacle_sector_map::SectorMapConfig _config;
  obstacle_sector_map::SectorMapBuilder _builder;
  PipelineStats _stats;
};

}  // namespace obstacle_sector_node
