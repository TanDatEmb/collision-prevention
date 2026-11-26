# Repository Guidelines

This repository contains the PX4 collision-prevention stack: a ROS 2 companion pipeline builds a
72-sector obstacle map, PX4 applies the final velocity constraint, and the CP Monitor exposes
read-only SITL telemetry.

## Project Structure & Module Organization

- `ros2_ws/src/` contains the ROS 2 packages: ray sources, sector mapping, the node, the PX4 bridge,
  and `collision_prevention_msgs` (DDS monitor-message mirrors).
- `firmware/PX4-Autopilot/` is the pinned PX4 submodule; changes to DDS exports belong in its fork.
- `tools/dashboard/` contains the HTTP/SSE bridge, standalone UI, and bridge tests.
- `tools/manual_sim/` runs the Gazebo X500 bench scenario; `config/`, `docs/`, and `docs/adr/` hold
  platform values, architecture, and decisions.

## Build, Test, and Development Commands

Use ROS 2 Jazzy and at most two parallel jobs:

```bash
make px4-sitl       # build PX4 SITL
make ros            # build the ROS 2 workspace
make test           # run all selected ROS tests
make dashboard      # serve CP Monitor on 127.0.0.1:8765
make sim-dashboard  # start SITL and CP Monitor together
make sim-stop       # stop simulator processes
```

Run `./tools/check_msg_parity.sh` after changing mirrored PX4 messages. A green build is not flight
qualification.

## Coding Style & Naming Conventions

Format C/C++ with the repository CI `clang-format` check; use four-space Python indentation and
small, descriptive functions. Use `snake_case` for Python, lower-case ROS packages, PascalCase
message types, and unit-bearing interface fields such as `_m`, `_rad`, `_us`, and `_m_s`.

## Testing Guidelines

Add focused `ament_cmake_gtest` tests for C++ behavior and direct `unittest` coverage for dashboard
Python. Name C++ tests `test_<component>.cpp`; run `make test` and the relevant dashboard test before
opening a PR. Do not claim SITL, HIL, bench, or flight evidence unless that run was actually made.

## Commit & Pull Request Guidelines

Use imperative Conventional Commit subjects, for example `feat(dashboard): add live telemetry bridge`.
Each change has one issue, one branch (`<type>/<issue>-<slug>`), and one PR. Describe scope, topic
contracts, test commands, and evidence in the PR; link the issue. Keep PRs draft until review is
complete, never push to `main` or force-push after review, and merge only with owner approval.

## Safety & Agent-Specific Instructions

Read `docs/architecture.md` before behavior changes. Never invent safety thresholds: retain `TBD`
values and record open items. Keep dashboard code monitor-only; selecting a radar bin must not alter
control or leveling. Preserve unrelated work, avoid committing plans/progress reports, and verify
the final tree and tests before reporting completion.
