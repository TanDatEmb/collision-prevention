#!/usr/bin/env bash
# Fail if a PX4 message used by this project differs between the PX4 firmware and px4_msgs.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
declare -A paths=(
  [ObstacleDistance]=msg/ObstacleDistance.msg
  [VehicleLocalPosition]=msg/versioned/VehicleLocalPosition.msg
  [VehicleLocalPositionSetpoint]=msg/VehicleLocalPositionSetpoint.msg
  [VehicleCommand]=msg/versioned/VehicleCommand.msg
  [VehicleCommandAck]=msg/versioned/VehicleCommandAck.msg
  [VehicleStatus]=msg/versioned/VehicleStatus.msg
  [TrajectorySetpoint]=msg/versioned/TrajectorySetpoint.msg
)
status=0
for name in "${!paths[@]}"; do
  fw="$root/firmware/PX4-Autopilot/${paths[$name]}"
  ros="$root/ros2_ws/src/px4_msgs/msg/$name.msg"
  if [[ "$(git hash-object "$fw")" == "$(git hash-object "$ros")" ]]; then
    echo "OK    $name"
  else
    echo "DIFF  $name" >&2
    status=1
  fi
done
exit "$status"
