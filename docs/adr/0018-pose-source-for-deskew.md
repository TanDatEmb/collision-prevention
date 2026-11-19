# 0018. Deskew pose comes from VehicleOdometry

## Context
ADR 0015 deskews rays in `livox_ray_source` and needs a vehicle pose at each ray time. In the pinned
PX4 (`uxrce_dds_client/dds_topics.yaml`), `/fmu/out/vehicle_local_position` is rate limited to 50 Hz
and has no roll or pitch, while `/fmu/out/vehicle_odometry` has no rate limit and carries position,
attitude `q`, `timestamp_sample` and `reset_counter` (`msg/versioned/VehicleOdometry.msg`).

## Decision
- The pose for deskew comes from `VehicleOdometry`, from the same estimator output that feeds the
  heading and position history of `cp_brake` (ICD-CP-04), so both use one pose chain.
- A pose reset (`reset_counter` change) invalidates interpolation across it; a ray whose time
  needs such an interpolation has no valid pose, which ICD-CP-06 treats as a missing pose.
- Origin motion during a scan (OPEN-031) is handled by a conservative filter in acquisition. The
  threshold value is not set here.

## Consequences
- The actual publication rate and latency of `VehicleOdometry` on the target network, and the
  interpolation scheme, are measured and stay open (OPEN-030, now limited to those).
- The filter threshold is a new value that needs an owner decision (OPEN-031) before the deskew
  code is implemented.
