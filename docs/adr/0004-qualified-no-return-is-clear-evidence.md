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
- Whether Mid-360S sends zero-depth samples at all was unverified (OPEN-022). Decided 2026-11-23 (PR #173,
  issue #172), from the vendor code and a capture of the real lidar: the SDK2 header and the vendor ROS driver give
  depth 0 no meaning (the driver forwards every point unfiltered and never decodes the tag); the lidar sends a fixed
  200,000 points per second, 48.7 % of the points of a room scene at depth 0 with their direction kept, so a shot
  with no return is a sample of depth 0 in spherical data. A depth of 0 with a valid direction is a qualified
  no-return (the Horizon-family page defines (0, theta, phi) for no object in range); the frame gates above still
  apply. Open: a blocked or dirty window may read the same; the vendor health status (`lidar_diag_status`,
  `hms_code`) is the known signal, to be used only if a bench capture shows it is needed.
- Embedding SDK2 is decided in ADR 0016 (closes OPEN-025).
