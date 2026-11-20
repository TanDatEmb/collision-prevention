# livox_ray_source

Ray source interface for `obstacle_sector_map` (ICD-CP-06). It holds the interface, the frame
segmentation, a replay source, the self-mask, the decoder of Livox spherical packets and the adapter
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

## Livox Mid-360(S), spherical samples (issue #58)
- `decodeSphericalPacket(bytes, size)` (pure, no SDK types) turns one point packet of data type 3
  (36 byte header, `dot_num` points of depth u32 mm, theta u16, phi u16, reflectivity u8, tag u8)
  into rays. Direction as the vendor driver `livox_ros_driver2` computes it: theta and phi are in
  0.01 degree, theta is the polar angle from +z and phi increases from +x towards +y, so azimuth is
  phi (counter-clockwise, FLU) and elevation is 90 degrees minus theta. Values outside theta
  [0, 18000] and phi [0, 36000] are invalid.
- A depth of 0 is invalid, never a return and never free: the protocol does not say what it means
  and the qualified no-return rule is open (OPEN-022, ADR 0004). On a real Mid-360S about half of
  the points had depth 0. Tags are not used.
- Time: only PTP time (`time_type` 1) is used; the packet time is the time of the first point and
  the points are equally spaced over `time_interval` (0.1 us, first to last point: a real packet
  has 96 points, 475 us first to last, 480 us between packets). Any other time type gives
  timestamp 0, which the sector map rejects (SYS-CP-016, ADR 0003). The vendor driver instead
  falls back to the host clock; this project does not.
- `UdpContinuityTracker` flags a gap, a duplicate or a step back of the 16 bit `udp_cnt`.
- `LivoxSdkSampleSource` (library `livox_ray_source_sdk`) starts the SDK with a JSON config,
  decodes every point packet from the SDK thread and gives the rays to a sink. With
  `configure_point_type` it reads the lidar's point type once per detection and writes the point
  type (spherical) only if it differs. The SDK itself also sends the host IP and ports of the
  config file at every start. Only the point cloud callback is set: the IMU is not read.
- Linking: the SDK archive carries its own spdlog 1.3.1, which crashes `rclcpp::init` next to
  ROS's 1.12 unless its symbols are hidden: link with `-Wl,--exclude-libs,liblivox_lidar_sdk_static.a`
  (the node executable does).

## Usage
```cpp
livox_ray_source::ReplayRaySource source(samples, window_ns);
while (auto frame = source.nextFrame()) {
  builder.reset(frame->packet_continuity_valid);
  for (const auto & ray : frame->rays) { builder.addRay(ray); }
  const auto map = builder.finalize();
}
```
