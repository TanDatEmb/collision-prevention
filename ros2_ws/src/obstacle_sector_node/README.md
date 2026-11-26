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
- The wire map uses 72 clockwise 5-degree bins in BODY_FRD, starting at the nose. A finite
  distance in the normal range is the smaller of the confirmed obstacle distance and the
  certified free range when both exist (ADR 0024). The message carries a single distance per bin,
  so consumers need separate evidence to locate confirmed obstacle returns. The pinned PX4 fork
  exports the monitor-only `cp_brake_constraints` and `cp_brake_hook` uORB topics over uXRCE-DDS;
  the ROS mirror is `collision_prevention_msgs`.

## Live CP Monitor
`make dashboard` serves the read-only browser dashboard at `http://127.0.0.1:8765` and waits for
SITL telemetry. `make sim-dashboard` starts the dashboard with the manual Gazebo SITL flow and
opens the page. The browser reads the 72-bin map, vehicle pose/status, PX4 speed caps and limiter
hook. Selecting a radar bin only changes the inspection readout; it does not alter leveling. The
leveling panel shows pitch and roll and, per bin, the rays in the upper and lower half of the collision
band, from `/collision_prevention/leveling_coverage` (published for the `ring_cloud` source only). It does
not show ray pairs before and after leveling.

## Livox telemetry and diagnostics

For `source:=livox`, the node keeps the safety path on one primary lidar and publishes raw
telemetry per detected serial:

- `/livox/<serial>/imu` (`sensor_msgs/Imu`): PTP-stamped gyro and accelerometer. Acceleration is
  converted from Livox g to m/s²; it is not fused into leveling.
- `/livox/<serial>/points` (`sensor_msgs/PointCloud2`): finite return points in the Livox FLU frame,
  with `reflectivity`, `tag` and per-point `time_offset_ns` fields.
- `/livox/status` (`diagnostic_msgs/DiagnosticArray`): device identity, packet/IMU counters,
  timestamp types, sample ages, continuity and primary-lidar faults.

Set `primary_lidar_sn` when more than one lidar is reachable. Without it, the first detected lidar
is accepted until a second appears; then the safety stream is rejected fail-closed while raw topics
remain available. A run without an FMU pose is acquisition-only: it cannot qualify leveling or PX4
`/fmu/in/obstacle_distance`.

## Usage
```cpp
auto config = obstacle_sector_node::loadSectorMapConfig("config/platforms/x500.yaml");
rclcpp::spin(std::make_shared<obstacle_sector_node::ObstacleSectorNode>(std::move(source), config));
```

## Run with a Livox Mid-360(S)
`ros2 run obstacle_sector_node obstacle_sector_node --ros-args -p platform_config:=<yaml> -p
source:=livox -p livox_config:=<Livox SDK2 JSON> -p primary_lidar_sn:=<serial>` (the tracked
`config/livox/mid360s_config.json` uses host `192.168.1.5`; change it for another NIC). Use the
bench profile `config/platforms/x500-livox-bench.yaml` for the owner-selected gyro history
(`depth: 128`, `max_bracket_gap_ms: 10`). `coverage_window_ms` is the frame window (default 100,
the ICD value); `configure_point_type` (default true) asks the lidar for spherical points once,
which is the only setting this path writes. Every 5 s the log shows packet/IMU counters:
`unsynchronized` counts packets whose time is not PTP; their rays carry time 0 and cannot form a
frame. The lidar must report PTP (`time_type` 1), for example with `make livox-up`. Software-only
PTP is not an absolute clock-offset qualification.

For a repeatable source check without an FC, use `make livox-check`. It verifies the Ethernet
address, tracked config and PTP process, then runs `livox_source_probe`; this is source-only
evidence. To run the real ROS 2 pipeline, keep `/fmu/out/vehicle_odometry` active and start:

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 run obstacle_sector_node obstacle_sector_node --ros-args \
  -p platform_config:=config/platforms/x500-livox-bench.yaml \
  -p source:=livox \
  -p livox_config:=config/livox/mid360s_config.json \
  -p primary_lidar_sn:=ARMCP550031900
```

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

The incoming cloud is in the sensor frame. The node applies the configured mount and uses the FMU
pose from `/fmu/out/vehicle_odometry` to level the rays at the map reference time (#179, ADR 0023).
The sector builder receives rays in the gravity-aligned, yaw-only frame. If the reference pose is
unavailable, the leveler drops the rays and marks the frame discontinuous. The live ROS topics
expose the input cloud and the resulting map. The individual leveled rays stay in the node.

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 launch obstacle_sector_node sitl_gz_ring_cloud.launch.py \
  platform_config:=<platform yaml with approved values> \
  layout_config:=tools/scenarios/multi_ring_lidar_layout.yaml
python3 tools/scenarios/check_multi_ring_lidar.py   # static vehicle, known wall and ground
```
