# obstacle_sector_node

Orin side of the sector pipeline: a ROS-free pipeline, a platform config loader and a ROS 2 node
class. There is no executable yet: it needs a sensor source (issue #58).

## Input contract
- A `livox_ray_source::RaySource` (ICD-CP-06) and a `SectorMapConfig`.
- `loadSectorMapConfig(yaml)` builds the config from the platform file (elevation band and coverage
  count) and refuses null, missing or invalid values. The node takes the parameter
  `sector_map_rate_hz` (default 10, the approved baseline).

## Output contract
- `SectorMapPipeline::step()` returns `kMessage` (an `ObstacleDistance`, ICD-CP-03), `kEncodeFailed`
  (no message, counted, nothing replaces it) or `kSourceEnded`. The message timestamp is the
  earliest accepted ray time (SYS-CP-016).
- `ObstacleSectorNode` publishes it on `/fmu/in/obstacle_distance` (best effort, volatile, keep
  last 2) at `sector_map_rate_hz`, and `DiagnosticArray` (frames, messages, encode_failures) on
  `status` at 1 Hz.

## Usage
```cpp
auto config = obstacle_sector_node::loadSectorMapConfig("config/platforms/x500.yaml");
rclcpp::spin(std::make_shared<obstacle_sector_node::ObstacleSectorNode>(std::move(source), config));
```
