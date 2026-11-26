# Leveling Coverage Dashboard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the dashboard's empty leveling ray-pair plots with a live sensor-FOV and per-bin vertical-coverage view that explains why a distance bin is covered or unknown.

**Architecture:** The sector-map builder exposes the existing up/down ray counts and coverage decision in its in-memory `SectorMap`. The ROS node publishes those counts with the actual mount and ring-layout geometry in a ROS-only `CpLevelingCoverage` message; PX4 DDS/uORB is unchanged. The Python bridge combines that message with odometry attitude and serves a compact snapshot consumed by the vanilla SVG dashboard.

**Tech Stack:** C++17, ROS 2/rosidl, Python `unittest`, vanilla HTML/CSS/SVG/JavaScript.

**Spec:** `docs/architecture.md` (SYS-CP-017, SYS-CP-018, SYS-CP-029), `ros2_ws/src/obstacle_sector_map/README.md`, and the approved leveling/FOV design in the task conversation.

## Global Constraints

- Use the repository's actual X500 values: mount pitch `+10°` down, ring FOV `−7°…+52°`, height band `−0.60…+0.25 m`, attitude allowance `0.0157 rad`, and `free_min_rays=5`.
- Do not invent or hardcode a general `±3°` sensor limit; no such limit exists in the current platform configuration.
- Keep PX4 DDS/uORB interfaces unchanged; coverage is a ROS-only monitor/debug topic.
- Keep the radar selector as a display selection only; it must not alter leveling or map processing.
- Preserve `AGENTS.md` and the root README exactly as they are on the feature branch.

## Review Focus

- Mount sign: a level vehicle must show the forward FOV as `−17°…+42°`, not the opposite sign; test the zero-pose case.
- Attitude update: pitch and roll must move the displayed view without changing configured sensor limits; test a non-zero pitch quaternion.
- Rear coverage: rear sectors require only the upper half; test `rear_only` with a missing lower half.
- Continuity: an incomplete packet must be visible as degraded coverage, not silently labelled clear.
- Missing telemetry: the UI must show an unavailable state and never draw fabricated ray pairs.

---

### Task 1: Preserve coverage evidence in the sector-map pipeline

**Files:**
- Modify: `ros2_ws/src/obstacle_sector_map/include/obstacle_sector_map/types.hpp`
- Modify: `ros2_ws/src/obstacle_sector_map/src/builder.cpp`
- Modify: `ros2_ws/src/obstacle_sector_node/include/obstacle_sector_node/pipeline.hpp`
- Modify: `ros2_ws/src/obstacle_sector_node/src/pipeline.cpp`
- Test: `ros2_ws/src/obstacle_sector_map/test/test_builder.cpp`
- Test: `ros2_ws/src/obstacle_sector_node/test/test_pipeline.cpp`

**Interfaces:** `SectorObservation` gains `up_ray_count`, `down_ray_count`, `rear_only`, and `coverage_ok`. `SectorMapPipeline::latestMap()` returns the most recently finalized map, including when encoding fails.

- [x] **Step 1: Write failing tests** for five up/five down coverage, a missing half, rear-only lower-half omission, and `latestMap()` retaining the finalized map.
- [x] **Step 2: Run the focused C++ tests** and confirm they fail because the fields/accessor do not exist.
- [x] **Step 3: Implement the fields, populate them before the builder's coverage `continue`, and retain the finalized map in the pipeline.** Do not change PX4 encoding semantics.
- [x] **Step 4: Run the focused tests** and confirm they pass.
- [x] **Step 5: Commit** as `feat(map): expose leveling coverage counts`.

### Task 2: Publish ROS-only leveling coverage telemetry

**Files:**
- Create: `ros2_ws/src/collision_prevention_msgs/msg/CpLevelingCoverage.msg`
- Modify: `ros2_ws/src/collision_prevention_msgs/CMakeLists.txt`
- Modify: `ros2_ws/src/obstacle_sector_node/CMakeLists.txt`
- Modify: `ros2_ws/src/obstacle_sector_node/package.xml`
- Modify: `ros2_ws/src/obstacle_sector_node/include/obstacle_sector_node/node.hpp`
- Modify: `ros2_ws/src/obstacle_sector_node/src/node.cpp`
- Modify: `ros2_ws/src/obstacle_sector_node/src/main.cpp`
- Test: `ros2_ws/src/obstacle_sector_node/test/test_node.cpp`

**Interfaces:** `CpLevelingCoverage` contains `timestamp`, `sample_timestamp`, mount RPY, sensor elevation min/max, `required_rays`, packet continuity, and fixed 72-element `up_ray_count`, `down_ray_count`, `rear_only`, and `coverage_ok` arrays. `LevelingCoverageConfig` carries the geometry into `ObstacleSectorNode`.

- [x] **Step 1: Add a failing node test** asserting an enabled node publishes one message with the geometry, required count, and 72 coverage entries after a frame.
- [x] **Step 2: Run the focused node test** and confirm the message/publisher/config are missing.
- [x] **Step 3: Add the message and publisher.** In `main.cpp`, retain the validated ring layout and loaded mount settings before constructing the node; calculate `sensor_elevation_max = min + step*(ring_count-1)`. Publish coverage only after a map with a non-negative age is accepted for PX4. Keep the feature disabled for planar scans.
- [x] **Step 4: Run the focused node test and ROS build tests** and confirm they pass.
- [x] **Step 5: Commit** as `feat(node): publish leveling coverage telemetry`.

### Task 3: Build the bridge snapshot contract

**Files:**
- Modify: `tools/dashboard/bridge.py`
- Test: `tools/dashboard/test_bridge.py`

**Interfaces:** `SnapshotStore.update_leveling_coverage(payload)` stores geometry and per-bin counts. The snapshot adds `leveling.geometry`, `leveling.view`, `leveling.band`, and `leveling.coverage`; view edges are computed for the selected bearing from mount rotation plus current pitch/roll.

- [x] **Step 1: Write failing Python tests** for the hover range `−17°…+42°`, a pitch change moving both edges, and `up/down` state `5/5` versus `2/5`; test that packet discontinuity is not labelled clear.
- [x] **Step 2: Run `python3 -m unittest tools/dashboard/test_bridge.py`** and confirm the new tests fail.
- [x] **Step 3: Implement geometry helpers and `update_leveling_coverage`.** Use the existing quaternion attitude convention, the validated mount/FOV values from the message, and the selected bin azimuth. Do not add a `±3°` default. Preserve old snapshots when telemetry is absent.
- [x] **Step 4: Add the ROS subscription** to `/collision_prevention/leveling_coverage`, guarded with the same optional-message fallback used for the existing custom messages.
- [x] **Step 5: Run the bridge tests** and confirm all pass.
- [x] **Step 6: Commit** as `feat(dashboard): expose live leveling coverage`.

### Task 4: Replace the dead leveling plots with the FOV/coverage chart

**Files:**
- Modify: `tools/dashboard/index.html`
- Test: `tools/dashboard/test_contract.py`

**Interfaces:** The leveling panel contains one SVG with short English labels: `Sensor FOV`, `Current view`, `Horizon`, `Band`, `Up`, `Down`, and selected-bin coverage. The radar selector only changes the highlighted bearing/readout.

- [x] **Step 1: Update the contract test** to require the new labels and to reject the `No ray pairs` empty-state text.
- [x] **Step 2: Run the contract test** and confirm it fails against the old pair plots.
- [x] **Step 3: Replace both `x-z/y-z` SVGs** with one responsive `Vertical coverage` SVG. Draw the configured FOV, dynamic current view, horizon, selected-bin marker, band/allowance labels, and `Up n/5 · Down n/5` state. Use a clear `Coverage unavailable` state when the ROS telemetry is missing.
- [x] **Step 4: Re-run contract and browser syntax checks**, then visually inspect a live snapshot at port 8765 after restarting only the dashboard process.
- [x] **Step 5: Commit** as `feat(dashboard): render live lidar coverage`.

### Task 5: Full verification and delivery record

**Files:**
- Modify: `tools/dashboard/test_contract.py` only if the final topic/name contract requires a locked assertion.
- Modify: `docs/superpowers/plans/2026-11-26-leveling-coverage-dashboard.md` to mark completed steps.

- [x] **Step 1: Run `python3 -m unittest discover -s tools/dashboard -p 'test_*.py'`.**
- [x] **Step 2: Run `ROS_DOMAIN_ID=99 make test`** so the user's live SITL cannot contaminate the integration suite.
- [x] **Step 3: Run the dashboard snapshot/SSE smoke test and inspect the selected-bin coverage plus dynamic FOV fields.**
- [x] **Step 4: Review `git diff --check`, `git status`, and the changed-file list; verify `AGENTS.md` and the root README are unchanged from the branch base.**
- [x] **Step 5: Commit any test-only adjustments** as `test(dashboard): lock leveling coverage contract`.

