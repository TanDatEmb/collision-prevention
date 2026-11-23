# obstacle_sector_map

Platform-neutral C++17 library. Builds a 72-sector horizontal obstacle map from qualified lidar
rays. No ROS or `px4_msgs` dependency. Design rationale: `docs/architecture.md`.

## Input contract
`SectorMapBuilder::addRay(RaySample)` accepts rays that the acquisition layer already:
- stamped with a non-zero acquisition time (`timestamp_ns`);
- transformed to a FLU frame (azimuth counter-clockwise from forward);
- classified as `kReturn`, `kQualifiedNoReturn` or `kInvalid`, with self-mask and any sensor
  confidence tags applied (platform-specific filtering stays in acquisition).

`SectorMapConfig` carries the height band (`height_up_m`, `height_down_m`, in the level frame), the window `obstacle_min_returns` within `obstacle_window_m` and `free_min_rays` (none defaulted, ADR 0020 D2). The range
limits (0.10 m and 40 m, ICD-CP-03) and the boundary tolerance are fixed constants.

## Output contract
`finalize()` returns a `SectorMap` with one `SectorObservation` per sector:
`kUnknown`, `kObservedClear`, `kOccupied` (`distance_m`: the smaller of the confirmed obstacle distance and the certified free distance) or `kTooClose`.
- A return inside the height band within the qualified range is obstacle evidence. The obstacle
  distance is the near edge of the nearest range window of width `obstacle_window_m` that holds at
  least `obstacle_min_returns` returns, so a few isolated returns never brake the vehicle. Any
  in-band return below 0.10 m is `kTooClose`.
- Every other ray (a no-return, a return beyond the qualified range, a return above or below the
  band) proves the band empty along its path: it certifies `min(end * cos(elevation), h / tan|elevation|)`
  horizontally, at most 40 m, with `h_up` above and `h_down` below the horizon (a horizon ray counts in
  both halves). Each half certifies its `free_min_rays`-th largest value; the sector's free distance is
  the smaller half. A half with fewer rays certifies nothing. Free distance needs packet continuity.
- `distance_m` is the smaller of the obstacle distance and a certified free distance. Free distance at
  the qualified range with no obstacle is `kObservedClear`. No obstacle and nothing certified is
  `kUnknown`. A sector that no free ray reached has no free distance, and that never shortens a
  confirmed obstacle distance (a wall that fills the sector has no free ray).
- Rear class (ADR 0020 D3 option B, issue #141): a sector whose centre is more than `rear_start_rad` from the front
  needs and certifies from the upper half only (the lidar never sees below its own height behind the vehicle);
  obstacle evidence is unchanged and a confirmed rear obstacle still limits. Default pi: no rear class, the
  conservative case. X500: 134.6 degrees. Obstacles below the lidar behind the vehicle are a stated non-claim.
- Coverage (SYS-CP-029, issue #148): a half of the band that holds fewer than `free_min_rays` rays, free
  or obstacle (an in-band return counts in the half it lies in, a horizon return in both), is outside the
  field of view, not filled by a wall. With packet continuity the sector is then `kUnknown` (cap 0), even
  when an obstacle is confirmed farther away, so the cap cannot rise on the data that is left. The
  confirmed distance stays in `obstacle_distance_m` as a diagnostic. Without packet continuity credible
  obstacle returns stay occupied as before.

## Usage
```cpp
obstacle_sector_map::SectorMapBuilder builder(config);
builder.reset(packet_continuity_valid);
for (const auto & ray : rays) { builder.addRay(ray); }
const obstacle_sector_map::SectorMap map = builder.finalize();
```
