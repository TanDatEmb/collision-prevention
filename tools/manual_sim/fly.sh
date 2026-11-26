#!/usr/bin/env bash
# Manual-flight SITL: Gazebo (window optional), the obstacle arena, PX4 in Position mode with cp_brake on, the
# lidar bridge and obstacle_sector_node, then the status screen. Bench values only, never platform values.
#
# usage (from the repository root): make sim | make sim-keys | make sim-headless   (or bash tools/manual_sim/fly.sh)
#   JOY=qgc   stick from the QGroundControl virtual joystick (default)
#   JOY=keys  stick from the keyboard in this terminal
#   GUI=0     no Gazebo window
#   KEYS="3:t,35:w,..."  replay key presses at those seconds (scripted runs, implies no keyboard)
#   PX4_BUILD=<dir>      PX4 SITL build to fly (default: firmware/PX4-Autopilot/build/px4_sitl_default)
# x or Ctrl-C quits and stops everything; make sim-stop stops a session from any other terminal.
# PX4 parameters and logs live in build/manual_sim, apart from every other SITL run.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
FORK=$REPO/firmware/PX4-Autopilot
PX4=${PX4_BUILD:-$FORK/build/px4_sitl_default}
T=$REPO/tools/scenarios
RUN=$REPO/build/manual_sim
LOG=$RUN/log-launch

if ps -eo args | grep -qE "^gz sim|[b]in/px4( |$)|^MicroXRCEAgent|dashboard/bridge\.py"; then
  echo "Another simulation is running. Stop it with: make sim-stop"; exit 1
fi
for f in "$PX4/bin/px4" "$REPO/ros2_ws/install/setup.bash" "$T/spawn_obstacle.sh" "$REPO/tools/dashboard/bridge.py" "$REPO/tools/dashboard/index.html"; do
  [ -e "$f" ] || { echo "missing: $f (build it with: make px4-sitl / make ros)"; exit 1; }
done
mkdir -p "$LOG"
rm -f "$RUN/parameters.bson" "$RUN/parameters_backup.bson"

# COM_RC_IN_MODE 1: MAVLink manual control (QGroundControl). 8: the highest MAVLink instance wins, so the keyboard
# beats the zero stick QGroundControl sends.
export JOY=${JOY:-qgc}
RC_MODE=1; [ "$JOY" = keys ] && RC_MODE=8

PIDS=()
cleanup() {
  echo "stopping the simulation..."
  for p in "${PIDS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
# each program in its own process group, so that cleanup also ends its children
start() { local log=$1; shift; setsid "$@" > "$LOG/$log" 2>&1 < /dev/null & PIDS+=($!); }

if [ "${DASHBOARD:-0}" = 1 ]; then
  echo "0/4 dashboard..."
  start dashboard.log bash -c "source /opt/ros/jazzy/setup.bash; source '$REPO/ros2_ws/install/setup.bash'; exec python3 '$REPO/tools/dashboard/bridge.py' --host '${DASHBOARD_HOST:-127.0.0.1}' --port '${DASHBOARD_PORT:-8765}'"
  for _ in $(seq 1 20); do
    sleep 0.2
    grep -q "CP Monitor:" "$LOG/dashboard.log" 2>/dev/null && break
  done
  grep -q "CP Monitor:" "$LOG/dashboard.log" 2>/dev/null || { echo "dashboard did not start, see $LOG/dashboard.log"; exit 1; }
  if [ "${DASHBOARD_OPEN:-0}" = 1 ]; then
    python3 -m webbrowser "http://${DASHBOARD_HOST:-127.0.0.1}:${DASHBOARD_PORT:-8765}/" >/dev/null 2>&1 || true
  fi
fi

# the Mid-360S-density lidar model comes first; the rest of the bench models from tools/scenarios
export GZ_SIM_RESOURCE_PATH="$HERE/models:$T/models:$FORK/Tools/simulation/gz/models:$FORK/Tools/simulation/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4/src/modules/simulation/gz_plugins
export GZ_SIM_SERVER_CONFIG_PATH="$FORK/src/modules/simulation/gz_bridge/server.config"

echo "1/4 Gazebo..."
start server.log gz sim -s -r --headless-rendering --verbose=1 "$FORK/Tools/simulation/gz/worlds/default.sdf"
sleep 10
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 20000 \
  --req "sdf: \"<sdf version='1.9'><include><uri>model://x500_lidar_multi_ring</uri><name>x500_lidar_multi_ring_0</name><pose>0 0 0.0 0 0 0</pose></include></sdf>\"" \
  > /dev/null || { echo "could not spawn the vehicle, see $LOG/server.log"; exit 1; }
echo "2/4 obstacles and window..."
bash "$HERE/arena.sh"
if [ "${GUI:-1}" = 1 ]; then
  start gui.log env QT_QPA_PLATFORM=xcb DISPLAY="${DISPLAY:-:0}" gz sim -g --verbose=1
  sleep 10
  gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 --req 'x: -12, y: 0, z: 10' > /dev/null
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "x500_lidar_multi_ring_0"' > /dev/null
fi

echo "3/4 PX4 (Position mode, cp_brake on)..."
start agent.log MicroXRCEAgent udp4 -p 8888
start px4.log env PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 HEADLESS=1 PX4_GZ_STANDALONE=1 \
  PX4_GZ_MODEL_NAME=x500_lidar_multi_ring_0 \
  PX4_PARAM_SIM_GZ_EN_LIDAR=0 PX4_PARAM_CPB_EN=1 PX4_PARAM_UXRCE_DDS_SYNCT=0 PX4_PARAM_CPB_T_LAT=0.95 \
  PX4_PARAM_MPC_XY_VEL_P_ACC=3.0 PX4_PARAM_MPC_XY_VEL_D_ACC=0.35 PX4_PARAM_MPC_XY_VEL_I_ACC=0.4 \
  PX4_PARAM_CPB_CSTR_AGE=0.25 PX4_PARAM_CPB_DECEL_MAX=3.0 PX4_PARAM_CPB_HDG_DEPTH=64 PX4_PARAM_CPB_HDG_GAP=0.05 \
  PX4_PARAM_CPB_SYNC_RTT=0.01 PX4_PARAM_CPB_SYNC_OFF=0.005 PX4_PARAM_CPB_SYNC_AGE=2.0 \
  PX4_PARAM_MC_ROLL_P=4.0 PX4_PARAM_MC_PITCH_P=4.0 PX4_PARAM_MPC_XY_P=0.95 \
  PX4_PARAM_CPB_A_SIDE=1.2 PX4_PARAM_CPB_V_SIDE=5.0 PX4_PARAM_CPB_REAR_ANG=134.6 \
  PX4_PARAM_COM_RC_IN_MODE=$RC_MODE PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_NAV_RCL_ACT=1 PX4_PARAM_MIS_TAKEOFF_ALT=4.0 \
  "$PX4/bin/px4" -d -w "$RUN" "$PX4/etc"
for _ in $(seq 1 60); do
  sleep 1
  timeout 3 "$PX4/bin/px4-cp_brake" status 2>/dev/null | grep -q "enabled: 1" && break
done
timeout 3 "$PX4/bin/px4-cp_brake" status 2>/dev/null | grep -q "config faults: 0x0" \
  || { echo "cp_brake is not running correctly, see $LOG/px4.log"; exit 1; }

echo "4/4 lidar bridge and obstacle map..."
start node.log bash -c "source /opt/ros/jazzy/setup.bash; source '$REPO/ros2_ws/install/setup.bash';
  G=/world/default/model/x500_lidar_multi_ring_0/link/lidar_link/sensor/lidar_multi_ring/scan/points
  ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \"\$G@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked\" --ros-args -r \$G:=points_raw &
  python3 '$HERE/relay_one_frame.py' &
  exec ros2 run obstacle_sector_node obstacle_sector_node --ros-args --params-file '$HERE/layout.yaml' \
    -p use_sim_time:=true -p platform_config:='$REPO/config/platforms/x500.yaml' -p cloud_topic:=points"
# ready only once the node has published obstacle messages: its statistics line is printed even when the bridge,
# the relay or the clouds are missing, with every counter at 0
published() { grep -qE "ring cloud: .* messages [1-9][0-9]*," "$LOG/node.log" 2>/dev/null; }
for _ in $(seq 1 40); do
  sleep 1
  published && break
done
published || { echo "obstacle_sector_node publishes no obstacle message, see $LOG/node.log"; exit 1; }
echo "ready."

cat <<'MAP'

             NORTH (left with the nose east)
       [square pillar]      ======north wall======      | 6 m gate |
                                      ^ 16 m
   WEST  \ oblique wall               (0,0) ---- 20 m ----> | east wall   EAST (ahead)
                      ---- bar 4 m high     o     o  o  three thin poles
             SOUTH (right)

MAP
PX4_BIN="$PX4/bin" python3 "$HERE/keys.py"
