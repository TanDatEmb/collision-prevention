# 0012. PX4 and px4_msgs are pinned submodules

## Context
Wire contracts and controller behaviour must be reproducible; floating branches would change them.

## Decision
`firmware/PX4-Autopilot` (fork branch `collision-prevention/release-1.17`, `f65ddce`) and
`ros2_ws/src/px4_msgs` (`release/1.17`, `86d8239`) are submodules at fixed commits. Updates are
deliberate, reviewed changes. `tools/check_msg_parity.sh` checks that the seven required message
definitions are identical in both.

## Consequences
- CI and local builds initialize only these two submodules, not PX4's nested ones. ADR 0016 adds
  Livox SDK2 as a third pinned submodule.
