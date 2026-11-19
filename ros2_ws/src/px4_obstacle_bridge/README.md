# px4_obstacle_bridge

Encodes an `obstacle_sector_map::SectorMap` as a PX4 `px4_msgs/msg/ObstacleDistance`.
It is the only place that changes frame, direction and unit conventions.

## Input contract
- `SectorMap` in FLU order (sector 0 forward, counter-clockwise) and the `SectorMapConfig` used to
  build it.

## Output contract
`encodeObstacleDistance()` returns `std::optional<ObstacleDistance>`:
- `MAV_FRAME_BODY_FRD`, laser sensor, 72 sectors of 5 degrees, clockwise from forward;
- distances in centimetres (floored); `0` = too close, `max_distance + 1` = observed clear,
  `UINT16_MAX` = unknown;
- `timestamp` is the map acquisition time in microseconds.

It returns `std::nullopt` (fail closed) for an invalid config, zero or sub-microsecond timestamp,
occupied distance outside the configured range, clear sectors without continuity or coverage, and
ranges that would alias reserved wire values.

## Usage
```cpp
const auto message = px4_obstacle_bridge::encodeObstacleDistance(map, config);
if (message) { /* publish to /fmu/in/obstacle_distance */ }
```
