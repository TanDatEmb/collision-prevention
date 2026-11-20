#!/usr/bin/env bash
# Add a tall flat wall to the running Gazebo SITL world (bench tool).
# usage: spawn_wall.sh <distance_east_m> [width_m] [height_m]
# The wall face is <distance_east_m> east of the world origin (Gazebo +x is East, PX4 local east). It
# is tall so that the vehicle cannot fly over it and the lidar plane always meets it; a 2 m wall lets
# the vehicle climb above it and the input stops. A frame with no valid ray cannot be encoded, so the
# world must always hold something the lidar can hit.
set -euo pipefail
dist="${1:?distance east in metres}"
width="${2:-80}"
height="${3:-30}"
thick="0.3"
x=$(python3 -c "print(${dist} + ${thick} / 2)")
z=$(python3 -c "print(${height} / 2)")
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req "sdf: \"<sdf version='1.9'><model name='test_wall'><static>true</static>\
<pose>${x} 0 ${z} 0 0 0</pose><link name='link'>\
<collision name='collision'><geometry><box><size>${thick} ${width} ${height}</size></box></geometry></collision>\
<visual name='visual'><geometry><box><size>${thick} ${width} ${height}</size></box></geometry></visual>\
</link></model></sdf>\""
