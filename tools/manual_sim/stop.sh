#!/usr/bin/env bash
# Stop every simulator process of the bench: the manual sim or a leftover scripted run. QGroundControl is not touched.
# usage: make sim-stop   (or bash tools/manual_sim/stop.sh)
pids() { ps -eo pid,args | awk -v me=$$ '$1 != me' | grep -E "$1" | grep -v grep | awk '{print $1}'; }

# the clean way first: Ctrl-C the status screen, so that fly.sh restores its terminal and stops its processes
k=$(pids 'python3 [^ ]*manual_sim/keys\.py')
if [ -n "$k" ]; then
  kill -INT $k 2>/dev/null
  sleep 8
fi

SIM='^ *[0-9]+ +gz sim|bin/px4 |MicroXRCEAgent udp4|ros_gz_bridge|parameter_bridge|lib/obstacle_sector_node/obstacle_sector_node|ros2 run obstacle_sector_node|relay_one_frame\.py|make px4_sitl|manual_sim/fly\.sh'
p=$(pids "$SIM")
[ -n "$p" ] && kill $p 2>/dev/null && sleep 3
p=$(pids "$SIM")
[ -n "$p" ] && kill -9 $p 2>/dev/null && sleep 1

n=$(pids "$SIM" | wc -l)
if [ "$n" -eq 0 ]; then
  echo "SITL stopped, no simulator process left."
else
  echo "$n simulator processes are still running:"
  ps -eo pid,args | grep -E "$SIM" | grep -v grep | cut -c1-120
  exit 1
fi
