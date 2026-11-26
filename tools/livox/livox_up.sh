#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)
iface=${LIVOX_IFACE:-enp52s0}
host_ip=${LIVOX_HOST_IP:-192.168.1.5}
sensor_ip=${LIVOX_SENSOR_IP:-192.168.1.100}
runtime_dir=${LIVOX_RUNTIME_DIR:-${repo_dir}/build/livox}
ptp_config=${repo_dir}/config/livox/ptp4l-software.conf
ptp_pid_file=${runtime_dir}/ptp4l.pid
ptp_log=${runtime_dir}/ptp4l.log

command -v ip >/dev/null || { echo "livox-up: ip command is required" >&2; exit 2; }
command -v ptp4l >/dev/null || { echo "livox-up: ptp4l is required" >&2; exit 2; }
ip link show "$iface" >/dev/null 2>&1 || { echo "livox-up: interface not found: $iface" >&2; exit 2; }
mkdir -p "$runtime_dir"

sudo ip addr replace "${host_ip}/24" dev "$iface"

if pgrep -af "ptp4l.*-i[[:space:]]*${iface}" >/dev/null; then
  echo "livox-up: ptp4l already runs on ${iface}"
else
  sudo ptp4l -S -i "$iface" -m -f "$ptp_config" >"$ptp_log" 2>&1 &
  echo $! >"$ptp_pid_file"
  sleep 1
  if ! kill -0 "$(cat "$ptp_pid_file")" 2>/dev/null; then
    echo "livox-up: ptp4l exited; see ${ptp_log}" >&2
    exit 3
  fi
  echo "livox-up: ptp4l started (pid $(cat "$ptp_pid_file"))"
fi

echo "livox-up: host ${host_ip}/24 on ${iface}; sensor ${sensor_ip}"
echo "livox-up: run 'make livox-check' to run the project SDK2 probe"
echo "livox-up: software PTP is not absolute clock-offset qualification"
