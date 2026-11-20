#!/usr/bin/env python3
"""Measure the velocity step response of the simulated vehicle, the basis of CPB_T_LAT (bench tool).

usage:
  measure_velocity_step.py fly --px4-bin <path of px4-cp_brake> [--speeds 3,5,8] [--hold-s 8] [--altitude 12]
  measure_velocity_step.py analyze <csv-prefix> [--speeds 3,5,8]

`fly` takes off, then in Offboard mode commands east velocity steps 0 -> +V -> 0 -> -V -> 0 for each V
(the sign alternates so the vehicle stays near the origin) and prints one JSON line. Offboard is used
because the limiter hook is not active there: the response is the vehicle's own, not the limiter's.
`analyze` reads the ulog2csv output (`vehicle_local_position_setpoint_0.csv`, `vehicle_local_position_0.csv`)
and prints, for every step down to zero, the equivalent lag: the extra distance travelled after the step
divided by the speed before it. It is the lag that, put in front of an ideal instant stop, gives the same
stopping distance, so it is the quantity the stopping model needs. The same output lists the time to 63 %
of the response and the settling time. CPB_T_LAT is the largest equivalent lag rounded up to 0.05 s.
Bench tool: nothing here is an approved platform value. Run with python3 without -I (pymavlink).
"""
import argparse
import csv
import json
import math
import subprocess
import sys
import time

STEP_MIN = 1.0       # m/s, a setpoint change below this is not a step
SETTLE_BAND = 0.1    # m/s
ROUND_UP_S = 0.05    # CPB_T_LAT is rounded up to this resolution (owner, 2026-11-20: "rounded up")


def fly(a):
    from pymavlink import mavutil
    link = mavutil.mavlink_connection(a.connect)
    link.wait_heartbeat(timeout=30)
    mav = mavutil.mavlink
    client = a.px4_bin.rsplit("/", 1)[0]

    def run(tool, *args):
        try:
            return subprocess.run([client + "/" + tool, *args], capture_output=True, text=True,
                                  timeout=20).stdout.strip()
        except subprocess.TimeoutExpired:
            return None

    result = {"error": None, "steps": []}
    run("px4-param", "set", "NAV_DLL_ACT", "0")
    run("px4-param", "set", "MIS_TAKEOFF_ALT", str(a.altitude))
    run("px4-commander", "arm", "-f")
    time.sleep(1)
    run("px4-commander", "takeoff")
    deadline = time.time() + 60
    while time.time() < deadline:
        lp = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if lp is not None and -lp.z > a.altitude - 1.0:
            break
    else:
        result["error"] = "the vehicle did not take off"
        print(json.dumps(result))
        return 4
    time.sleep(a.hover_s)

    ignore = 0b110111000111  # use velocity only: position, acceleration, yaw and yaw rate ignored

    def send(vy):
        link.mav.set_position_target_local_ned_send(
            0, link.target_system, link.target_component, mav.MAV_FRAME_LOCAL_NED, ignore,
            0, 0, 0, 0, vy, 0, 0, 0, 0, 0, 0)

    def hold(vy, seconds):
        end = time.time() + seconds
        while time.time() < end:
            send(vy)
            time.sleep(0.05)

    hold(0.0, 2.0)  # PX4 needs a setpoint stream before it accepts Offboard
    result["offboard"] = run("px4-commander", "mode", "offboard")
    hold(0.0, 2.0)
    for v in a.speeds:
        for target in (v, 0.0, -v, 0.0):
            send(target)
            result["steps"].append({"t_unix": round(time.time(), 2), "to_m_s": target})
            hold(target, a.hold_s)
    run("px4-commander", "mode", "auto:loiter")
    print(json.dumps(result))
    return 0


def read(path, columns):
    """Rows with every column finite; a setpoint is NaN where PX4 controls position instead of velocity."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    kept = []
    for r in rows:
        try:
            vals = [float(r[c]) for c in columns]
        except (KeyError, ValueError, TypeError):
            continue
        if all(math.isfinite(v) for v in vals):
            kept.append(vals)
    if len(kept) < 10:
        raise ValueError("fewer than 10 finite rows in " + path)
    return {c: [row[i] for row in kept] for i, c in enumerate(columns)}


def analyze(a):
    sp = read(a.prefix + "vehicle_local_position_setpoint_0.csv", ["timestamp", "vy"])
    ps = read(a.prefix + "vehicle_local_position_0.csv", ["timestamp", "vy"])
    ts = [v * 1e-6 for v in sp["timestamp"]]
    tp = [v * 1e-6 for v in ps["timestamp"]]
    steps = []
    for i in range(1, len(ts)):
        # a step to zero: the setpoint was beyond STEP_MIN and is now within the settle band of zero
        if abs(sp["vy"][i - 1]) >= STEP_MIN and abs(sp["vy"][i]) < SETTLE_BAND:
            j = max(k for k in range(len(tp)) if tp[k] <= ts[i - 1] + 1e-9) if tp[0] <= ts[i - 1] else 0
            v0 = ps["vy"][j]
            if abs(v0) < STEP_MIN:
                continue
            sign = 1.0 if v0 > 0 else -1.0
            t0 = ts[i]
            area, t63, settled = 0.0, None, None
            for k in range(j + 1, len(tp)):
                if tp[k] < t0:
                    continue
                y = sign * ps["vy"][k]
                dt = tp[k] - max(tp[k - 1], t0)
                area += max(0.0, y) * dt
                if t63 is None and y <= 0.368 * abs(v0):
                    t63 = tp[k] - t0
                if abs(y) < SETTLE_BAND:
                    settled = tp[k] - t0
                    break
                if tp[k] - t0 > 15.0:
                    break
            steps.append({"t_s": round(t0, 2), "speed_before_m_s": round(abs(v0), 2),
                          "equivalent_lag_s": round(area / abs(v0), 3),
                          "t63_s": None if t63 is None else round(t63, 3),
                          "settle_s": None if settled is None else round(settled, 3)})
    wanted = {round(v, 1) for v in a.speeds}
    kept = [s for s in steps if any(abs(s["speed_before_m_s"] - w) < 0.5 for w in wanted)]
    out = {"steps_down_to_zero": kept, "all_steps_found": len(steps)}
    if kept:
        worst = max(s["equivalent_lag_s"] for s in kept)
        out["worst_equivalent_lag_s"] = worst
        out["t_lat_rounded_up_s"] = round(math.ceil(worst / ROUND_UP_S - 1e-9) * ROUND_UP_S, 2)
    print(json.dumps(out, indent=2))
    return 0 if kept else 2


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fly")
    f.add_argument("--connect", default="udpin:0.0.0.0:14550")
    f.add_argument("--px4-bin", required=True)
    f.add_argument("--speeds", type=lambda s: [float(v) for v in s.split(",")], default=[3.0, 5.0, 8.0])
    f.add_argument("--hold-s", type=float, default=8.0)
    f.add_argument("--altitude", type=float, default=12.0)
    f.add_argument("--hover-s", type=float, default=15.0)
    n = sub.add_parser("analyze")
    n.add_argument("prefix")
    n.add_argument("--speeds", type=lambda s: [float(v) for v in s.split(",")], default=[3.0, 5.0, 8.0])
    a = ap.parse_args()
    return fly(a) if a.cmd == "fly" else analyze(a)


if __name__ == "__main__":
    sys.exit(main())
