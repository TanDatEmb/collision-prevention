#!/usr/bin/env bash
# Add a wall that leans away from the vehicle to the running Gazebo SITL world (bench tool).
# usage: spawn_lean_wall.sh <distance_north_m> <sensor_height_m> [lean_deg]
# The wall face passes through (0, <distance_north_m>, <sensor_height_m>) and leans away from the vehicle by
# lean_deg (default 30) from the vertical, about the east axis. With the ground and a vertical wall it gives
# the plane fit of analyze_attitude_time.py a plane whose normal has a north and a vertical part, so roll,
# pitch and yaw errors of a pose all change the residuals. Face normal (world ENU): (0, -cos a, sin a).
set -euo pipefail
dist="${1:?distance north in metres}"
height="${2:?sensor height in metres}"
lean="${3:-30}"
read -r cy cz roll < <(python3 - "$dist" "$height" "$lean" <<'PY'
import math, sys
d, h, a = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
thick = 0.3
# face centre (local y = -thick/2) must lie at (0, d, h); the box rolls by -a about the east axis
print(d + 0.5 * thick * math.cos(a), h - 0.5 * thick * math.sin(a), -a)
PY
)
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req "sdf: \"<sdf version='1.9'><model name='lean_wall'><static>true</static>\
<pose>0 ${cy} ${cz} ${roll} 0 0</pose><link name='link'>\
<collision name='collision'><geometry><box><size>60 0.3 40</size></box></geometry></collision>\
<visual name='visual'><geometry><box><size>60 0.3 40</size></box></geometry></visual>\
</link></model></sdf>\""
