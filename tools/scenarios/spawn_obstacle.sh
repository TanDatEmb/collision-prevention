#!/usr/bin/env bash
# Add a static obstacle to the running Gazebo SITL world (bench tool).
# usage: spawn_obstacle.sh <name> <x> <y> <z> <size_x|radius> <size_y> <size_z|length> <yaw_rad> [box|cyl]
# Gazebo +x is East and +y North (PX4 local east and north). <x> <y> <z> is the centre. A box takes its three
# sizes; a cylinder takes <radius> in the first size, the second is ignored, <length> in the third. Make the
# obstacle tall enough that the vehicle cannot fly over it, or the case is about height on purpose. A frame
# with no valid ray cannot be encoded, so the world must always hold something the lidar can hit.
set -euo pipefail
name="${1:?name}"; x="${2:?x}"; y="${3:?y}"; z="${4:?z}"
sx="${5:?size x or radius}"; sy="${6:?size y}"; sz="${7:?size z or length}"; yaw="${8:?yaw in rad}"
shape="${9:-box}"
if [ "$shape" = cyl ]; then
  geom="<cylinder><radius>${sx}</radius><length>${sz}</length></cylinder>"
else
  geom="<box><size>${sx} ${sy} ${sz}</size></box>"
fi
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req "sdf: \"<sdf version='1.9'><model name='${name}'><static>true</static>\
<pose>${x} ${y} ${z} 0 0 ${yaw}</pose><link name='link'>\
<collision name='collision'><geometry>${geom}</geometry></collision>\
<visual name='visual'><geometry>${geom}</geometry></visual></link></model></sdf>\""
