# 0004. Only a qualified no-return counts as clear evidence

## Context
An empty Cartesian sector cannot prove free space. `livox_ros_driver2` (Livox, MIT, v1.2.6)
supports Mid-360S and ROS 2 Jazzy, but converts every sample to x, y, z, tag, line and time without
filtering. A zero-depth sample becomes (0, 0, 0), so its direction is lost and it cannot be assigned
to a sector. Livox SDK2 (v1.3.1) exposes spherical samples (`kLivoxLidarSphericalCoordinateData`:
depth, theta, phi, reflectivity, tag), which keep the direction.

## Decision
A sector is observed clear only from positive, time-valid evidence: rays with preserved direction
classified as qualified no-return, valid packet continuity and the configured minimum ray count.
Everything else is unknown. Raw rays come from `livox_ray_source`, built on SDK2 spherical samples.
`livox_ros_driver2` is used for bring-up, debug and rosbag comparison only.

## Consequences
- Missing points, lost packets and discarded samples encode `UINT16_MAX`.
- The coverage threshold is configuration, not a default; it stays open as OPEN-023.
- Whether Mid-360S sends zero-depth samples at all is unverified (OPEN-022).
- Embedding SDK2 is decided in ADR 0016 (closes OPEN-025).
