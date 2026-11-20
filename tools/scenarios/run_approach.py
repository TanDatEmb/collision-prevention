#!/usr/bin/env python3
"""Fly one approach to a wall in PX4 SITL over MAVLink and record the vehicle position (bench tool).

usage: run_approach.py --speed 5 [--distance 100] [--altitude 12] [--toggle-cpb] [--connect udpin:0.0.0.0:14550]

The vehicle takes off in Mission mode and flies a straight leg east at --speed toward a wall that the
caller has placed (see spawn_wall.sh). The run ends when the vehicle has been at rest for 5 s after
moving, when the leg ends, or after --timeout seconds. It prints one JSON line with what it saw; the
verdict comes from analyze_stop.py on the ULog, not from this script. Bench values only: nothing here
is an approved platform value.

--toggle-cpb: set CPB_EN to 0 and then to 1 once in flight. This is the bench workaround of issue #112
(the state machine latches SAFE_STOP when the PX4 state becomes valid in flight); a report that uses it
must say so.
"""
import argparse
import json
import math
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
        cmd, p1, lat, lon, alt = items[seq]
        link.mav.mission_item_int_send(
            link.target_system, link.target_component, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            cmd, 1 if seq == 0 else 0, 1, p1, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), alt)
        sent.add(seq)
    ack = link.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    return ack is not None and ack.type == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="udpin:0.0.0.0:14550")
    ap.add_argument("--speed", type=float, required=True, help="mission leg speed [m/s]")
    ap.add_argument("--distance", type=float, default=100.0, help="length of the leg east [m]")
    ap.add_argument("--altitude", type=float, default=12.0, help="takeoff altitude [m]")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--toggle-cpb", action="store_true")
    a = ap.parse_args()

    link = mavutil.mavlink_connection(a.connect)
    link.wait_heartbeat(timeout=30)
    result = {"speed_m_s": a.speed, "toggle_cpb": a.toggle_cpb}

    # the headless simulator has no GCS link: without this the GCS failsafe disarms the vehicle
    set_param(link, "NAV_DLL_ACT", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    gp = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=30)
    lat0, lon0 = gp.lat / 1e7, gp.lon / 1e7
    lon1 = lon0 + a.distance / (EARTH_M_PER_DEG * math.cos(math.radians(lat0)))
    mav = mavutil.mavlink
    items = [(mav.MAV_CMD_NAV_TAKEOFF, 0, lat0, lon0, a.altitude),
             (mav.MAV_CMD_NAV_WAYPOINT, 0, lat0, lon1, a.altitude)]
    result["mission_uploaded"] = upload_mission(link, items)

    # leg speed: the mission cruise speed parameter, bench value for this run only
    set_param(link, "MPC_XY_CRUISE", a.speed, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    link.mav.command_long_send(link.target_system, link.target_component, mav.MAV_CMD_COMPONENT_ARM_DISARM,
                               0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(2)

    if a.toggle_cpb:
        while True:  # wait for flight before the toggle
            lp = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
            if lp is not None and -lp.z > 2.0:
                break
            link.mav.command_long_send(link.target_system, link.target_component, mav.MAV_CMD_MISSION_START,
                                       0, 0, 0, 0, 0, 0, 0, 0)
            time.sleep(1)
        time.sleep(8)  # the PX4 heading must be good for control
        result["cpb_en_off"] = set_param(link, "CPB_EN", 0, mav.MAV_PARAM_TYPE_INT32)
        time.sleep(1)
        result["cpb_en_on"] = set_param(link, "CPB_EN", 1, mav.MAV_PARAM_TYPE_INT32)
    else:
        link.mav.command_long_send(link.target_system, link.target_component, mav.MAV_CMD_MISSION_START,
                                   0, 0, 0, 0, 0, 0, 0, 0)

    start = time.time()
    peak = 0.0
    moved = False
    rest_since = None
    last = None
    while time.time() - start < a.timeout:
        lp = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if lp is None:
            continue
        speed = math.hypot(lp.vx, lp.vy)
        peak = max(peak, speed)
        last = (lp.x, lp.y, -lp.z)
        if speed > 0.5:
            moved = True
        if moved and speed < REST_SPEED:
            rest_since = rest_since or time.time()
            if time.time() - rest_since >= REST_HOLD_S:
                break
        else:
            rest_since = None
    result.update({"peak_speed_m_s": round(peak, 2), "last_xyz_m": [round(v, 2) for v in last] if last else None,
                   "ended_at_rest": rest_since is not None, "duration_s": round(time.time() - start, 1)})
    print(json.dumps(result))
    return 0 if result["mission_uploaded"] and last else 2


if __name__ == "__main__":
    sys.exit(main())
