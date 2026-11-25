# collision-prevention

LiDAR sector-based collision prevention for PX4 multicopters (pre-alpha).

## Status

Pre-alpha. G1 (requirements) and G2 (architecture) are approved by the owner, who is the sole
safety approver; G1 has one open exception (HZ-CP-009). Built and unit-tested: the sector map with the
certified range (ADR 0024), leveling with the FMU pose and the Livox gyro (ADR 0023), the `ObstacleDistance`
encoder, the ROS 2 node with a Gazebo multi-ring source and the Livox SDK2 source, and the PX4 brake module
`cp_brake` in the fork. Evidence so far is Gazebo SITL with bench values only; the platform values are TBD,
and there is no bench, HIL or flight evidence. No safety claim is made.

## How it works

A lidar produces rays. `obstacle_sector_map` classifies them into 72 horizontal sectors
(occupied, observed clear, too close or unknown; only a qualified no-return counts as clear).
`px4_obstacle_bridge` encodes the map as a PX4 `ObstacleDistance` message, which reaches PX4 over
uXRCE-DDS. PX4 owns the final velocity constraint. Details: [docs/architecture.md](docs/architecture.md).

```
Livox -> obstacle_sector_map -> px4_obstacle_bridge -> uXRCE-DDS -> PX4
```

## Repository layout

```
AGENTS.md                      rules for AI agents and contributors
config/platforms/x500.yaml     platform values (X500 SITL bench values and self-mask; gyro TBD)
docs/architecture.md           scope, requirements, ICDs, hazards, V&V, open items
docs/adr/                      accepted architecture decisions
firmware/PX4-Autopilot         PX4 fork (submodule, pinned)
ros2_ws/src/livox_ray_source     ray source interface, frame segmentation, replay source
ros2_ws/src/obstacle_sector_map   platform-neutral sector map library
ros2_ws/src/obstacle_sector_node pipeline, config loader and ROS node class
ros2_ws/src/px4_obstacle_bridge   ObstacleDistance encoder
ros2_ws/src/px4_msgs              PX4 ROS 2 messages (submodule, pinned)
ros2_ws/third_party/Livox-SDK2    Livox SDK2 (submodule, pinned; used by `livox_ray_source_sdk`)
tools/check_msg_parity.sh      checks PX4 and px4_msgs message definitions match
.agents/skills/                commit and pr skills (symlinked from .claude/skills/)
.github/workflows/ci.yml       PR title check, format check, build and test
```

## Requirements

Ubuntu 24.04 and ROS 2 Jazzy.

| Dependency | Pin |
| --- | --- |
| PX4 | `v1.17.0-25-ga814a95391`, fork branch `collision-prevention/release-1.17` |
| px4_msgs | `86d8239`, branch `release/1.17` |
| Livox SDK2 | `68ae1e1d` (vendor version 1.3.3, no release tag; last tag `v1.3.1`), license review open (OPEN-014) |
| PX4 targets | `px4_fmu-v6x_multicopter`, `px4_sitl_default` |

## Build and test

```bash
git clone --recurse-submodules https://github.com/TanDatEmb/collision-prevention.git
cd collision-prevention
source /opt/ros/jazzy/setup.bash
./tools/check_msg_parity.sh
cd ros2_ws
colcon build --parallel-workers 2
colcon test --parallel-workers 2 --packages-select obstacle_sector_map px4_obstacle_bridge \
  livox_ray_source obstacle_sector_node
colcon test-result --verbose
```

PX4 firmware (from `firmware/PX4-Autopilot`; needs the PX4 toolchain):

```bash
MAKEFLAGS=-j2 make px4_fmu-v6x_multicopter
MAKEFLAGS=-j2 make px4_sitl_default
```

A green build and unit test is not SITL, HIL, flight testing or qualification.

## Platforms

X500 (Gazebo SITL) is the first platform class: `config/platforms/x500.yaml`. The M350-class
multicopter is the target hardware platform and has no configuration yet. Unapproved values are
`null` with a `TBD: OPEN-xxx` note.

## Contributing

See [AGENTS.md](AGENTS.md).

## License

TBD (OPEN-014). Package manifests carry `TODO` until it is decided.
