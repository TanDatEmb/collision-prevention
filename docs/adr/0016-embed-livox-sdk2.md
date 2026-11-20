# 0016. Embed Livox SDK2 for raw rays

## Context
ADR 0004 needs rays with preserved direction. `livox_ros_driver2` converts every sample to
Cartesian x, y, z, so a zero-depth sample becomes (0, 0, 0) and loses its direction. The offline
probe of the Livox sample data found 35% to 70% zero-range points per lidar in Cartesian form
(issue #33). OPEN-025 left two options: embed Livox SDK2 or patch the driver to publish spherical
samples.

## Decision
`livox_ray_source` uses Livox SDK2 directly, with spherical samples (depth, theta, phi,
reflectivity, tag). SDK2 is added as a submodule. ADR 0004 examined v1.3.1, the last release tag,
but v1.3.1 does not build with GCC 13 (`<cstdint>` is missing in two headers) and has no call to
set the IMU output rate and ranges (key 0x002B, Mid-360S only). The pin is therefore commit
`68ae1e1d`, the first vendor commit after v1.3.1: it adds the two includes and
`SetLivoxLidarImuRange`, and the vendor's own CHANGELOG and version macro call it 1.3.3. It has no
release tag. The pin moves to a release tag when the vendor tags one that contains this commit
(issue #88). `livox_ros_driver2` stays a bring-up, debug and rosbag-comparison tool only.

## Consequences
- ADR 0012 gains a third pinned submodule: PX4, `px4_msgs` and SDK2. CI must initialise it; that
  change is made in the PR that adds the submodule.
- No ROS hop between the sensor and the sector map, and no patched third-party driver to maintain.
- The pinned commit is on the vendor's `master` branch, so `git submodule update --init` fetches
  it. The vendor has not released it, so a later vendor tag is a pin change made in its own issue.
- The 16 files the pin changes do not include the point and IMU receive path. The same commit also
  rewrites the SDK's lidar-log and debug point-cloud writers (file names, `fsync` on every write);
  they act only when those options are enabled, so `livox_ray_source` leaves them disabled until
  they are reviewed.
- The third-party license notice follows OPEN-014.
- How a sample becomes a qualified no-return is still ADR 0004 and OPEN-022, decided with
  Mid-360S spherical data.
- Closes OPEN-025.
