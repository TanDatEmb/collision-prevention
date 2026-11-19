# obstacle_sector_node

Orin side of the sector pipeline. At present it holds the ROS-free pipeline; the ROS 2 node
(publisher, QoS, diagnostics, platform values) is a later step (issue #59).

## Input contract
- A `livox_ray_source::RaySource` (ICD-CP-06) and a `SectorMapConfig` (elevation band, coverage
  threshold). No default is supplied here.

## Output contract
`SectorMapPipeline::step()` returns a `StepResult`:
- `kMessage` with an `ObstacleDistance` (ICD-CP-03) built from one frame: reset with the frame's
  continuity, `addRay` per ray, `finalize`, `encodeObstacleDistance`;
- `kEncodeFailed`: no message, counted in `stats().encode_failures`; nothing replaces it;
- `kSourceEnded`: the source has no more frames.
The message timestamp is the earliest accepted ray time (SYS-CP-016).

## Usage
```cpp
obstacle_sector_node::SectorMapPipeline pipeline(std::move(source), config);
auto result = pipeline.step();
if (result.status == obstacle_sector_node::StepStatus::kMessage) { /* publish *result.message */ }
```
