# livox_ray_source

Ray source interface for `obstacle_sector_map` (ICD-CP-06). It holds the interface, the frame
segmentation, a replay source, the self-mask, the decoder of Livox point packets and the adapter
that reads them through the Livox SDK2 (ADR 0016).

## Input contract
- `StreamSample`: a `RaySample` (non-zero `timestamp_ns`, `range_m`, `azimuth_rad`,
  `elevation_rad`, `masked`, `return_kind`) and a `continuity_break` flag set when packets were
  lost just before it.

## Output contract
- `RayFrame`: the rays of one coverage window plus one `packet_continuity_valid` flag.
  The flag is false if a break was flagged or the timestamps went backwards inside the frame.
- A window starts at the first ray with a non-zero timestamp and lasts `window_ns`
  (`coverage_window_ms` in the ICD, no default here).
- Rays are passed on unchanged: the direction of returns and no-returns is preserved.
- `SelfMask` (ADR 0006) sets `masked` for rays in the sensor frame: a return whose end point is in
  an upright cylinder, a no-return whose segment up to `max_distance_m` crosses one.
  `loadSelfMask(yaml)` refuses null, missing or empty values. `MaskingRaySource` wraps any source
  and applies it to every frame, before any frame change or deskew.

## Livox Mid-360S point stream (issue #58)
- `decodeSphericalPacket(bytes, size)` (pure, no SDK types) turns one point packet of data type 3
  (36 byte header, `dot_num` points of depth u32 mm, theta u16, phi u16, reflectivity u8, tag u8)
  into rays. Direction as the vendor driver `livox_ros_driver2` computes it: theta and phi are in
  0.01 degree, theta is the polar angle from +z and phi increases from +x towards +y, so azimuth is
  phi (counter-clockwise, FLU) and elevation is 90 degrees minus theta. Values outside theta
  [0, 18000] and phi [0, 36000] are invalid.
- A depth of 0 with a valid direction is a qualified no-return (issue #172): the vendor convention
  is `(0, theta, phi)` for no object in range, the vendor driver forwards such points unfiltered,
  and a real Mid-360S sends them. No vendor document defines it beyond that, so a blocked or dirty
  window may read the same (open, bench test). Reflectivity and tag are retained as telemetry
  metadata; they do not change the safety-map classification.
- Cartesian-high (32-bit millimetres) and Cartesian-low (16-bit centimetres) packets are dispatched
  to the same ray contract. Their direction is derived from x/y/z; an all-zero vector is invalid,
  never a qualified no-return.
- Time: only PTP time (`time_type` 1) is used; the packet time is the time of the first point and
  the points are equally spaced over `time_interval` (0.1 us, first to last point: a real packet
  has 96 points, 475 us first to last, 480 us between packets). Any other time type gives
  timestamp 0, which the sector map rejects (SYS-CP-016, ADR 0003). The vendor driver instead
  falls back to the host clock; this project does not.
- `UdpContinuityTracker` flags a gap, a duplicate or a step back of the 16 bit `udp_cnt`.
- `LivoxSdkSampleSource` (library `livox_ray_source_sdk`) starts the SDK with a JSON config,
  decodes supported spherical, Cartesian-high and Cartesian-low packets from the SDK thread and
  gives the safety rays to a sink. It also exposes complete PTP IMU samples (gyro plus acceleration
  in g), reflectivity/tag/line metadata, per-device counters and a diagnostic snapshot.
  `GyroSink` remains the only input to leveling; acceleration is telemetry only. With
  `configure_point_type` it writes the configured point layout during the device handshake.
  Optional Mid-360S settings are written only when present; a failed command leaves the safety sink
  closed. Each detection also requests `kLivoxLidarNormal` and enables the IMU stream, matching the
  official ROS2 driver lifecycle. A configured primary serial is required when more than one lidar
  is present; raw telemetry still identifies every device. SDK2 status push text is logged, while
  temperature/FUSA fields are not promoted because the SDK exposes them as opaque text.
- `collision_prevention.install_attitude` is an optional device setting with `roll_deg`, `pitch_deg`,
  `yaw_deg`, `x_mm`, `y_mm` and `z_mm`. It updates the Mid-360S installation record only; it does
  not transform `StreamSample` or PointCloud2 coordinates. The host pipeline still applies
  `sensor.mount_pose_in_base_link` during leveling, so raw FLU telemetry remains compatible with
  other LIO consumers. Example:

  ```json
  "collision_prevention": {
    "point_data_type": "spherical",
    "install_attitude": {
      "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
      "x_mm": 0, "y_mm": 0, "z_mm": 0
    }
  }
  ```
- Linking: the SDK archive carries its own spdlog 1.3.1, which crashes `rclcpp::init` next to
  ROS's 1.12 unless its symbols are hidden: link with `-Wl,--exclude-libs,liblivox_lidar_sdk_static.a`
  (the node executable does).

## Live probe

Build the workspace first, then run the project-linked probe. It exits non-zero when no stream,
IMU, PTP timestamp, continuity or primary-lidar contract is available:

```bash
make livox-up
LIVOX_PRIMARY_SN=ARMCP550031900 LIVOX_DURATION_S=10 make livox-check
make livox-down
```

`make livox-up` assigns `192.168.1.5/24` to `enp52s0` and starts the tracked software PTP
profile. Override `LIVOX_IFACE`, `LIVOX_HOST_IP`, `LIVOX_SENSOR_IP` and `LIVOX_CONFIG` for a
different bench. Software-only PTP proves packet timestamp continuity, not an absolute host/sensor
clock offset. `LIVOX_ALLOW_UNSYNC=1 make livox-check` is source-only debug evidence and is never a
qualified pipeline run.

## Usage
```cpp
livox_ray_source::ReplayRaySource source(samples, window_ns);
while (auto frame = source.nextFrame()) {
  builder.reset(frame->packet_continuity_valid);
  for (const auto & ray : frame->rays) { builder.addRay(ray); }
  const auto map = builder.finalize();
}
```
