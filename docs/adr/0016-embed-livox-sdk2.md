# 0016. Embed Livox SDK2 for raw rays

## Context
ADR 0004 needs rays with preserved direction. `livox_ros_driver2` converts every sample to
Cartesian x, y, z, so a zero-depth sample becomes (0, 0, 0) and loses its direction. The offline
probe of the Livox sample data found 35% to 70% zero-range points per lidar in Cartesian form
(issue #33). OPEN-025 left two options: embed Livox SDK2 or patch the driver to publish spherical
samples.

## Decision
`livox_ray_source` uses Livox SDK2 directly, with spherical samples (depth, theta, phi,
reflectivity, tag). SDK2 is added as a submodule pinned to a release tag; the tag is chosen in the
issue that adds it (ADR 0004 examined v1.3.1). `livox_ros_driver2` stays a bring-up, debug and
rosbag-comparison tool only.

## Consequences
- ADR 0012 gains a third pinned submodule: PX4, `px4_msgs` and SDK2. CI must initialise it; that
  change is made in the PR that adds the submodule.
- No ROS hop between the sensor and the sector map, and no patched third-party driver to maintain.
- The third-party license notice follows OPEN-014.
- How a sample becomes a qualified no-return is still ADR 0004 and OPEN-022, decided with
  Mid-360S spherical data.
- Closes OPEN-025.
