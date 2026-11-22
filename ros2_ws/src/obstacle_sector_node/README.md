# obstacle_sector_node

Orin side of the sector pipeline: a ROS-free pipeline, a platform config loader, a ROS 2 node
class and an executable for a planar scan (the Gazebo 2D lidar) or a Livox Mid-360(S) (SDK2).

## Input contract
- A `livox_ray_source::RaySource` (ICD-CP-06) and a `SectorMapConfig`.
- `loadSectorMapConfig(yaml)` builds the config from the platform file (height band, window and
  ray counts) and refuses null, missing or invalid values. The node takes the parameter
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

## Run with a Livox Mid-360(S)
`ros2 run obstacle_sector_node obstacle_sector_node --ros-args -p platform_config:=<yaml> -p
source:=livox -p livox_config:=<Livox SDK2 JSON>` (the SDK's `mid360s_config.json` sample, with
the host IP of this machine, works). `coverage_window_ms` is the frame window (default 100, the
ICD value); `configure_point_type` (default true) lets the node ask the lidar for spherical points
once, which is the only parameter it writes (see `livox_ray_source`). Every 5 s the log shows the
packet counters: `unsynchronized` counts packets whose time is not PTP; their rays carry time 0, no
frame forms and the diagnostics say "source ended". The lidar must be synchronized by PTP
(`time_type` 1, ADR 0003), for example with `ptp4l` as master on the lidar's port.

## Run in Gazebo SITL
The executable `obstacle_sector_node` reads `sensor_msgs/LaserScan` from `scan` (parameter
`scan_topic`), turns each scan into one ray frame and runs the node on it. Parameters: the required
`platform_config` (a null or missing value in the YAML is an error, so the committed `x500.yaml`,
whose values are still TBD, is refused), `scan_topic` (default `scan`), `source_wait_s` (default 1,
how long it waits for a scan before the diagnostics say "source ended"), `sector_map_rate_hz`.

In a scan, a range inside `[range_min, range_max)` is a return. A range at or beyond `range_max`
or `+inf` is a qualified no-return (ideal simulation, a fidelity gap of `docs/architecture.md`
section 7) only when `range_max` is at least the map's 40 m; a lidar that reaches less far says
nothing about the space beyond its limit, so those rays are invalid and the sector stays unknown
(the Gazebo 2D lidar has `range_max` 30 m, so a sector with no return is 65535, not 4001). Anything
else is invalid, never free. Azimuth is the scan angle, elevation is 0.

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

### Multi-ring bench lidar (`source:=ring_cloud`)
The planar scan has no height information. The bench model `x500_lidar_multi_ring`
(`tools/scenarios/models`, issue #138, ADR 0022 decision 4) carries a multi-ring lidar over the
Mid-360S field (-7 to +52 degrees from the boresight, 720 azimuth steps by 60 rings, 10 Hz, 50 m)
at mount R1 (sensor 0.12 m ahead of and 0.14 m above `base_link`, 10 degrees nose down). It is a
simulation asset: the scan pattern and density of the real sensor are not claimed.

`ros_gz_bridge` converts a `LaserScan` of this sensor to one ring only (720 ranges of 43200), so the
source reads the organised `PointCloud2` (720 x 60). Azimuth and elevation of every ray come from the
index (`index = ring * 720 + azimuth step`) and the required parameters `ring_layout.*`
(`tools/scenarios/multi_ring_lidar_layout.yaml`, no defaults), never from x, y, z: a no-hit point has
infinite coordinates and the `ring` field of the Gazebo cloud is always 0. The cloud gives the range
and the no-hit state. A finite point that is not on its index direction is invalid and counted
("index mismatches" in the 5 s log line); a cloud whose size or fields do not match the layout gives
a frame with no rays. A range at or beyond `ring_layout.range_max_m` or `+inf` is a qualified
no-return only when `range_max_m` is at least 40 m, as for the planar scan; `-inf` (nearer than the
minimum) and NaN are invalid.

The node has no mount and no leveling yet (#137): the rays are in the sensor frame, so a sector map
from this source is not leveled and is not valid evidence on the vehicle's pitch.

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 launch obstacle_sector_node sitl_gz_ring_cloud.launch.py \
  platform_config:=<platform yaml with approved values> \
  layout_config:=tools/scenarios/multi_ring_lidar_layout.yaml
python3 tools/scenarios/check_multi_ring_lidar.py   # static vehicle, known wall and ground
```
