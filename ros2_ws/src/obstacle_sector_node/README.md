# obstacle_sector_node

Orin side of the sector pipeline: a ROS-free pipeline, a platform config loader, a ROS 2 node
class and an executable for a planar scan (the Gazebo 2D lidar). The Livox source is issue #58.

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

## Run in Gazebo SITL
The executable `obstacle_sector_node` reads `sensor_msgs/LaserScan` from `scan` (parameter
`scan_topic`), turns each scan into one ray frame and runs the node on it. Parameters: the required
`platform_config` (a null or missing value in the YAML is an error, so the committed `x500.yaml`,
whose values are still TBD, is refused), `scan_topic` (default `scan`), `source_wait_s` (default 1,
how long it waits for a scan before the diagnostics say "source ended"), `sector_map_rate_hz`.

In a scan, a range inside `[range_min, range_max)` is a return, a range at or beyond `range_max`
or `+inf` is a qualified no-return (ideal simulation, a fidelity gap of `docs/architecture.md`
section 7), and anything else is invalid, never free. Azimuth is the scan angle, elevation is 0.

```bash
# terminal 1: PX4 SITL with the x500 and a 2D lidar. SIM_GZ_EN_LIDAR=0 stops PX4's own Gazebo
# bridge from publishing a second obstacle_distance next to this node's.
cd firmware/PX4-Autopilot
PX4_PARAM_SIM_GZ_EN_LIDAR=0 HEADLESS=1 make px4_sitl gz_x500_lidar_2d
# terminal 2
MicroXRCEAgent udp4 -p 8888
# terminal 3
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 launch obstacle_sector_node sitl_gz.launch.py platform_config:=<platform yaml with approved values>
```

The launch file bridges `/clock` and the lidar scan from Gazebo and runs the node on simulated time
(ADR 0003: `use_sim_time`, `UXRCE_DDS_SYNCT=0`). Change `gz_scan_topic` if the world or the model
instance has another name.
