# livox_ray_source

Ray source interface for `obstacle_sector_map` (ICD-CP-06). At present it holds the interface, the
frame segmentation and a replay source; there is no Livox SDK code yet (ADR 0016).

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

## Usage
```cpp
livox_ray_source::ReplayRaySource source(samples, window_ns);
while (auto frame = source.nextFrame()) {
  builder.reset(frame->packet_continuity_valid);
  for (const auto & ray : frame->rays) { builder.addRay(ray); }
  const auto map = builder.finalize();
}
```
