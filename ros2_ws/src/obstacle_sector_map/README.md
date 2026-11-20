# obstacle_sector_map

Platform-neutral C++17 library. Builds a 72-sector horizontal obstacle map from qualified lidar
rays. No ROS or `px4_msgs` dependency. Design rationale: `docs/architecture.md`.

## Input contract
`SectorMapBuilder::addRay(RaySample)` accepts rays that the acquisition layer already:
- stamped with a non-zero acquisition time (`timestamp_ns`);
- transformed to a FLU frame (azimuth counter-clockwise from forward);
- classified as `kReturn`, `kQualifiedNoReturn` or `kInvalid`, with self-mask and any sensor
  confidence tags applied (platform-specific filtering stays in acquisition).

`SectorMapConfig` carries the elevation band and coverage threshold (none defaulted). The range
limits (0.10 m and 40 m, ICD-CP-03) and the boundary tolerance are fixed constants.

## Output contract
`finalize()` returns a `SectorMap` with one `SectorObservation` per sector:
`kUnknown`, `kObservedClear`, `kOccupied` (minimum projected range) or `kTooClose`.
- A `kQualifiedNoReturn` ray and a `kReturn` beyond the qualified range (40 m horizontally) count as
  clear evidence (`clear_ray_count`): both prove the ray path up to 40 m empty. A far return is never
  dropped and never an obstacle. Clear also needs packet continuity and the minimum count; a sector
  with no evidence stays `kUnknown`.
- Obstacle evidence on a sector boundary is copied into both adjacent sectors.
- The map timestamp is the earliest contributing ray time.

## Usage
```cpp
obstacle_sector_map::SectorMapBuilder builder(config);
builder.reset(packet_continuity_valid);
for (const auto & ray : rays) { builder.addRay(ray); }
const obstacle_sector_map::SectorMap map = builder.finalize();
```
