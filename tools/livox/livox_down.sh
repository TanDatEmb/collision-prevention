#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)
runtime_dir=${LIVOX_RUNTIME_DIR:-${repo_dir}/build/livox}
pid_file=${runtime_dir}/ptp4l.pid

if [[ ! -f "$pid_file" ]]; then
  echo "livox-down: no PTP process owned by this workspace"
  exit 0
fi

pid=$(<"$pid_file")
if [[ ! "$pid" =~ ^[0-9]+$ ]]; then
  echo "livox-down: invalid PID file: $pid_file" >&2
  exit 2
fi

if sudo kill -0 "$pid" 2>/dev/null; then
  sudo kill -TERM "$pid"
  for _ in {1..10}; do
    if ! sudo kill -0 "$pid" 2>/dev/null; then
      break
    fi
    sleep 0.2
  done
  if sudo kill -0 "$pid" 2>/dev/null; then
    echo "livox-down: ptp4l did not stop (pid $pid)" >&2
    exit 3
  fi
  echo "livox-down: stopped ptp4l (pid $pid)"
else
  echo "livox-down: ptp4l is already stopped (pid $pid)"
fi

rm -f "$pid_file"
