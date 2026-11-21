#!/usr/bin/env python3
"""Fly one approach to a wall in PX4 SITL over MAVLink and record the vehicle position (bench tool).

usage: run_approach.py --speed 5 [--distance 100] [--bearing-deg 90] [--yaw-deg H] [--altitude 12] [--toggle-cpb]
                       [--baseline] [--connect udpin:0.0.0.0:14550]

The vehicle takes off, turns to its heading and flies a straight leg at --speed toward an obstacle that the
caller has placed (see spawn_obstacle.sh). By default the leg is east (Gazebo +x) with the nose along the leg: body +x
forward, the priority direction; --yaw-deg holds another heading to fly sideways or backwards. The run ends
when the vehicle has been at rest for 5 s after moving, when the leg ends, or after --timeout seconds. It prints one JSON line with what it saw; the
verdict comes from analyze_stop.py on the ULog, not from this script. Exit code 0: the leg ran;
3 a parameter was not applied; 4 the vehicle did not take off; 5 cp_brake was not enforcing, so the leg
was not started; 6 the mission was refused; 7 the leg did not start (JSON field "error" says which).
Bench values only: nothing here
is an approved platform value.

The vehicle first takes off and hovers; with --px4-bin the runner then reads `px4-cp_brake status` and
waits until cp_brake is enforcing (ACTIVE_CLEAR or BRAKING) before the leg starts. If the state machine
has latched SAFE_STOP (issue #112, which happens only sometimes) it sets CPB_EN to 0 and then to 1,
the bench workaround; a report that used it must say so (the JSON says `toggled`). The flag
--toggle-cpb forces the toggle. Run with python3 without -I (pymavlink is a user package).
"""
import argparse
import json
import math
import re
import subprocess
import sys
import time

from pymavlink import mavutil

EARTH_M_PER_DEG = 111320.0
REST_SPEED = 0.1
REST_HOLD_S = 5.0


def set_param(link, name, value, ptype):
    link.mav.param_set_send(link.target_system, link.target_component, name.encode(), float(value), ptype)
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
    return None


def px4_param_set(px4_bin, name, value):
    """Set a parameter through the PX4 shell client (px4-param next to px4-cp_brake). A MAVLink PARAM_SET
    did not re-enable CPB_EN in the bench runs; the shell client does. None on a timeout."""
    client = px4_bin.rsplit("/", 1)[0] + "/px4-param"
    try:
        return subprocess.run([client, "set", name, str(value)], capture_output=True, text=True,
                              timeout=15).stdout.strip()
    except subprocess.TimeoutExpired:
        return None


def toggle_cpb(px4_bin):
    """CPB_EN 0 then 1 (bench workaround of issue #112); CPB_EN is set back to 1 even on an error."""
    try:
        px4_param_set(px4_bin, "CPB_EN", 0)
        time.sleep(2)
    finally:
        for _ in range(3):
            if px4_param_set(px4_bin, "CPB_EN", 1) is not None:
                break


def commander(px4_bin, *args):
    """Run px4-commander next to px4-cp_brake (arm and takeoff worked through it where a MAVLink
    takeoff command did not in the bench runs)."""
    client = px4_bin.rsplit("/", 1)[0] + "/px4-commander"
    try:
        return subprocess.run([client, *args], capture_output=True, text=True, timeout=20).stdout.strip()
    except subprocess.TimeoutExpired:
        return None


def cp_state(px4_bin):
    """(state, safe_stop_latched) from `px4-cp_brake status`, or None when it cannot be read."""
    try:
        out = subprocess.run([px4_bin, "status"], capture_output=True, text=True, timeout=15).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"state: (\d+).*", out)
    n = re.search(r"safe stop latched: (\d)", out)
    return (int(m.group(1)), int(n.group(1))) if m and n else None


def upload_mission(link, items):
    link.mav.mission_clear_all_send(link.target_system, link.target_component)
    link.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    link.mav.mission_count_send(link.target_system, link.target_component, len(items))
    sent = set()
    deadline = time.time() + 20
    while len(sent) < len(items) and time.time() < deadline:
        msg = link.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=2)
        if msg is None:
            continue
        seq = msg.seq
        cmd, p1, lat, lon, alt, yaw_deg = items[seq]  # param4 of a waypoint is its heading in degrees
        link.mav.mission_item_int_send(
            link.target_system, link.target_component, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            cmd, 1 if seq == 0 else 0, 1, p1, 0, 0, yaw_deg, int(lat * 1e7), int(lon * 1e7), alt)
        sent.add(seq)
    ack = link.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    return ack is not None and ack.type == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="udpin:0.0.0.0:14550")
    ap.add_argument("--speed", type=float, required=True, help="mission leg speed [m/s]")
    ap.add_argument("--distance", type=float, default=100.0, help="length of the leg [m]")
    ap.add_argument("--bearing-deg", type=float, default=90.0,
                    help="direction of the leg, degrees clockwise from north (default 90: east, Gazebo +x)")
    ap.add_argument("--yaw-deg", type=float, default=None,
                    help="heading held on the leg; default: the nose along the leg (body +x forward, the priority "
                         "direction). Give another heading to fly sideways or backwards")
    ap.add_argument("--altitude", type=float, default=12.0, help="takeoff altitude [m]")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--toggle-cpb", action="store_true", help="always toggle CPB_EN 0 then 1 (issue #112)")
    ap.add_argument("--px4-bin", default=None, help="path of px4-cp_brake; enables the wait for enforcing")
    ap.add_argument("--hover-s", type=float, default=20.0, help="hover time before the leg [s]")
    ap.add_argument("--baseline", action="store_true",
                    help="CPB_EN is 0 on purpose: do not wait for cp_brake to enforce (needs --px4-bin for arming)")
    a = ap.parse_args()

    link = mavutil.mavlink_connection(a.connect)
    link.wait_heartbeat(timeout=30)
    result = {"speed_m_s_requested": a.speed, "error": None}
    mav = mavutil.mavlink

    def fail(message, code):
        result["error"] = message
        print(json.dumps(result))
        return code

    # the headless simulator has no GCS link (without NAV_DLL_ACT 0 the GCS failsafe disarms the vehicle);
    # the leg speed is a bench value for this run; both must be confirmed or the run does not count
    applied = {"NAV_DLL_ACT": set_param(link, "NAV_DLL_ACT", 0, mav.MAV_PARAM_TYPE_INT32),
               "MPC_XY_CRUISE": set_param(link, "MPC_XY_CRUISE", a.speed, mav.MAV_PARAM_TYPE_REAL32)}
    result["params_applied"] = applied
    if applied["NAV_DLL_ACT"] != 0 or applied["MPC_XY_CRUISE"] is None or abs(applied["MPC_XY_CRUISE"] - a.speed) > 1e-3:
        return fail("a parameter was not applied (NAV_DLL_ACT, MPC_XY_CRUISE)", 3)

    gp = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=30)
    if gp is None:
        return fail("no GLOBAL_POSITION_INT", 3)
    lat0, lon0 = gp.lat / 1e7, gp.lon / 1e7
    bearing = math.radians(a.bearing_deg)
    lat1 = lat0 + a.distance * math.cos(bearing) / EARTH_M_PER_DEG
    lon1 = lon0 + a.distance * math.sin(bearing) / (EARTH_M_PER_DEG * math.cos(math.radians(lat0)))
    yaw_deg = a.bearing_deg if a.yaw_deg is None else a.yaw_deg
    # the first item turns the vehicle on the spot, so the leg starts with the heading it asks for
    items = [(mav.MAV_CMD_NAV_WAYPOINT, 0, lat0, lon0, a.altitude, yaw_deg),
             (mav.MAV_CMD_NAV_WAYPOINT, 0, lat1, lon1, a.altitude, yaw_deg)]
    result["bearing_deg"] = a.bearing_deg
    result["yaw_deg_commanded"] = yaw_deg

    if a.px4_bin:
        px4_param_set(a.px4_bin, "MIS_TAKEOFF_ALT", a.altitude)  # `commander takeoff` takes no altitude
        result["arm"] = commander(a.px4_bin, "arm", "-f")
        time.sleep(1)
        result["takeoff"] = commander(a.px4_bin, "takeoff")
    else:
        link.mav.command_long_send(link.target_system, link.target_component, mav.MAV_CMD_COMPONENT_ARM_DISARM,
                                   0, 1, 21196, 0, 0, 0, 0, 0)
        time.sleep(2)
        link.mav.command_long_send(link.target_system, link.target_component, mav.MAV_CMD_NAV_TAKEOFF,
                                   0, 0, 0, 0, float("nan"), float("nan"), float("nan"), gp.alt / 1000.0 + a.altitude)
    airborne = False
    deadline = time.time() + 60
    while time.time() < deadline:
        lp = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if lp is not None and -lp.z > a.altitude - 1.0:
            airborne = True
            break
    if not airborne:
        return fail("the vehicle did not reach the takeoff altitude (not armed, or the takeoff was refused)", 4)
    time.sleep(a.hover_s)  # the PX4 heading must be good for control before CP can enforce

    result["toggled"] = False
    if a.px4_bin and not a.baseline:
        deadline = time.time() + 60
        while time.time() < deadline:
            st = cp_state(a.px4_bin)
            result["cp_state_before_leg"] = st
            if st and st[0] in (3, 4) and not a.toggle_cpb:  # a forced toggle runs before the state is accepted
                break
            if (st and st[1] == 1) or a.toggle_cpb:
                toggle_cpb(a.px4_bin)
                result["toggled"] = True
                a.toggle_cpb = False
                time.sleep(3)
            time.sleep(2)
        st = result.get("cp_state_before_leg")
        if not (st and st[0] in (3, 4)):
            return fail("cp_brake is not enforcing before the leg (state %s); the leg was not started" % (st,), 5)

    result["mission_uploaded"] = upload_mission(link, items)
    if not result["mission_uploaded"]:
        return fail("the mission was not accepted", 6)
    link.mav.command_long_send(link.target_system, link.target_component, mav.MAV_CMD_MISSION_START,
                               0, 0, 0, 0, 0, 0, 0, 0)

    start = time.time()
    peak = 0.0
    moved = rested = False
    rest_since = None
    last = None
    yaw_deg_now = None
    yaw_at_start = None
    yaw_in_leg = []
    while time.time() - start < a.timeout:
        lp = link.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=True, timeout=2)
        if lp is None:
            continue
        if lp.get_type() == "ATTITUDE":
            yaw_deg_now = math.degrees(lp.yaw)
            if moved:
                yaw_in_leg.append(yaw_deg_now)
            continue
        speed = math.hypot(lp.vx, lp.vy)
        peak = max(peak, speed)
        last = (lp.x, lp.y, -lp.z)
        if speed > 0.5:
            if not moved:
                yaw_at_start = yaw_deg_now
            moved = True
        if moved and speed < REST_SPEED:
            rest_since = rest_since or time.time()
            if time.time() - rest_since >= REST_HOLD_S:
                rested = True
                break
        else:
            rest_since = None
    if yaw_at_start is not None:
        result["yaw_deg_at_leg_start"] = round(yaw_at_start, 1)
    if yaw_in_leg:
        result["yaw_deg_range_in_leg"] = [round(min(yaw_in_leg), 1), round(max(yaw_in_leg), 1)]
    result.update({"peak_speed_m_s": round(peak, 2), "last_xyz_m": [round(v, 2) for v in last] if last else None,
                   "ended_at_rest": rested, "duration_s": round(time.time() - start, 1)})
    if not moved:
        return fail("the leg did not start: the vehicle never exceeded 0.5 m/s", 7)
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
