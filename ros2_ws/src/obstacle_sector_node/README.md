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
  last 2) at `sector_map_rate_hz`, and `DiagnosticArray` (latest map age, frames, messages,
  encode_failures, negative_age_drops; ERROR when the source ended, the latest frame failed or the
  latest map was held back; WARN after earlier encode failures or drops) on `status` at 1 Hz.
- A map whose sample time is ahead of the node clock (negative age) is held back, not published:
  PX4 clamps such a timestamp to the receive time and cannot see it (ICD-CP-04). It is counted in
  `negative_age_drops`; `messages` counts the maps the pipeline built, held back or not. A sample
  time that does not fit in int64 nanoseconds counts as ahead of the clock. No limit on a large
  age is applied (OPEN-033).

## Usage
```cpp
auto config = obstacle_sector_node::loadSectorMapConfig("config/platforms/x500.yaml");
rclcpp::spin(std::make_shared<obstacle_sector_node::ObstacleSectorNode>(std::move(source), config));
```
