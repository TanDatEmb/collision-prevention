# 0015. Deskew rays in the acquisition layer

## Context
A Mid-360S frame at 10 Hz spans about 100 ms. At 12 m/s the vehicle moves about 1.2 m during one
frame, and at a yaw rate of 90 deg/s the heading changes about 9 deg, more than one 5 deg sector.
Rays from one frame therefore do not share one vehicle pose.

## Decision
Deskew in `livox_ray_source`: each ray is corrected with its own acquisition time and a vehicle
pose history, so the rays reach `obstacle_sector_map` already expressed at one reference time.
The alternative, widening the influence zone with velocity and yaw rate, is not used. The sector
map and the bridge stay free of motion compensation.

## Consequences
- Needs per-point timestamps from the sensor path (OPEN-022) and a pose source with known rate,
  latency and frame (OPEN-030).
- A ray without a valid pose at its time must not become clear evidence (ADR 0004); the exact
  handling is decided at G2.
- Simulation stamps a scan once, so deskew is not exercised there; see the SITL fidelity gaps.
- Closes OPEN-027.
