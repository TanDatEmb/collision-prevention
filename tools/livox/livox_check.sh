#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)
iface=${LIVOX_IFACE:-enp52s0}
host_ip=${LIVOX_HOST_IP:-192.168.1.5}
config=${LIVOX_CONFIG:-${repo_dir}/config/livox/mid360s_config.json}
duration_s=${LIVOX_DURATION_S:-10}
sensor_ip=${LIVOX_SENSOR_IP:-192.168.1.100}
primary_sn=${LIVOX_PRIMARY_SN:-}
allow_unsync=${LIVOX_ALLOW_UNSYNC:-0}

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # ROS setup scripts use optional variables that are not nounset-safe.
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
command -v ip >/dev/null || { echo "livox-check: ip command is required" >&2; exit 2; }
command -v ros2 >/dev/null || { echo "livox-check: ROS 2 Jazzy is not installed" >&2; exit 2; }
command -v python3 >/dev/null || { echo "livox-check: python3 is required to validate JSON" >&2; exit 2; }
[[ -f "$config" ]] || { echo "livox-check: SDK2 config not found: $config" >&2; exit 2; }
if ! python3 -m json.tool "$config" >/dev/null 2>&1; then
  echo "livox-check: SDK2 config is not valid JSON: $config" >&2
  exit 2
fi
ip link show "$iface" >/dev/null 2>&1 || { echo "livox-check: interface not found: $iface" >&2; exit 2; }
if ! ip -4 -o addr show dev "$iface" | grep -Fq " ${host_ip}/"; then
  echo "livox-check: ${host_ip} is not assigned to ${iface}; run 'make livox-up'" >&2
  exit 2
fi
if ! grep -Eq '"host_ip"[[:space:]]*:[[:space:]]*"'"${host_ip}"'"' "$config"; then
  echo "livox-check: $config host_ip does not match ${host_ip}" >&2
  exit 2
fi

if ! ip route get "$sensor_ip" dev "$iface" >/dev/null 2>&1; then
  echo "livox-check: no route to ${sensor_ip} through ${iface}" >&2
  exit 2
fi

probe_args=(--config "$config" --duration-s "$duration_s")
if [[ -n "$primary_sn" ]]; then
  probe_args+=(--primary-sn "$primary_sn")
fi
if [[ "$allow_unsync" == "1" ]]; then
  probe_args+=(--allow-unsynchronized)
  echo "livox-check: WARNING: unsynchronized mode is source-only and not clock-qualified" >&2
else
  if ! pgrep -af "ptp4l.*-i[[:space:]]*${iface}" >/dev/null; then
    echo "livox-check: no ptp4l master found on ${iface}; run 'make livox-up'" >&2
    exit 2
  fi
fi

if [[ ! -f "$repo_dir/ros2_ws/install/setup.bash" ]]; then
  echo "livox-check: build ros2_ws before probing (make ros)" >&2
  exit 2
fi
# Colcon-generated setup files read optional variables such as COLCON_TRACE. Keep nounset for
# this script's own inputs, but let the setup chain handle an unset environment safely.
set +u
source "$repo_dir/ros2_ws/install/setup.bash"
set -u
echo "livox-check: probing ${duration_s}s on ${iface}; sensor ${sensor_ip}"
exec ros2 run livox_ray_source livox_source_probe "${probe_args[@]}"
