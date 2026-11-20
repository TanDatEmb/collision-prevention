#!/usr/bin/env python3
"""Metrics of one simulated approach to a wall, from `ulog2csv` output (scratch bench tool, not a product).

usage: analyze_stop.py <csv-prefix> --wall-dist D [--dir-ned N,E] [--lidar-offset 0.0] [--margin 2.0]

<csv-prefix> is the path printed by `ulog2csv -o <dir> <log>.ulg` without the topic suffix, for
example `/tmp/run1/log_0_` (files `<prefix>vehicle_local_position_0.csv`,
`<prefix>cp_brake_constraints_0.csv`). The wall face is D metres from the takeoff origin along
`--dir-ned` (a unit vector in local NED; the Gazebo +x axis is East, so `0,1`). Criteria (the owner,
2026-11-20): (a) the distance to the wall never goes below the margin CPB_MARGIN; (b) the vehicle is
at rest before the wall; (c) no oscillation: at most one sign change of the approach velocity after
the cap first reaches zero (here: after the peak speed); the stale-input criterion (d) is read from
the state timeline of a separate run (SAFE_STOP in flight).

The run verdict (ICD-CP-05) is PASS when criteria a to c hold, FAIL when one does not, and INVALID
when the evidence is incomplete (the approach never started, or `--require-constraints` was given and
the `cp_brake_constraints` log is missing); incomplete evidence is never PASS. Exit code 0, 1, 2.
Bench tool: the values it is given are not approved platform values.
"""
import argparse
import csv
import json
import math
import sys

REST_SPEED = 0.1      # m/s, "at rest"
REST_HOLD_S = 2.0     # s the speed must stay below REST_SPEED
REVERSAL_BAND = 0.1   # m/s, a sign change counts only beyond +-this
START_SPEED = 0.5     # m/s along the approach: the approach has started
DECEL_WINDOW_S = 0.2


def read(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return rows


def col(rows, name):
    return [float(r[name]) for r in rows]


def analyse(prefix, wall_dist, direction, lidar_offset, margin, require_constraints=False):
    pos = read(prefix + "vehicle_local_position_0.csv")
    t = [v * 1e-6 for v in col(pos, "timestamp")]
    x, y = col(pos, "x"), col(pos, "y")
    vx, vy = col(pos, "vx"), col(pos, "vy")
    dn, de = direction
    along = [px * dn + py * de for px, py in zip(x, y)]
    speed = [a * dn + b * de for a, b in zip(vx, vy)]
    gap = [wall_dist - s - lidar_offset for s in along]  # distance from the lidar to the wall face

    out = {"samples": len(t), "assumptions": {"lidar_offset_m": lidar_offset, "wall_dist_m": wall_dist}}
    start = next((i for i, v in enumerate(speed) if v > START_SPEED), None)
    if start is None:
        out["error"] = "the approach never started (speed along the wall direction stayed under 0.5 m/s)"
        return out
    peak = max(range(start, len(t)), key=lambda i: speed[i])
    out["t_start_s"] = round(t[start], 2)
    out["peak_speed_m_s"] = round(speed[peak], 2)
    out["t_peak_s"] = round(t[peak], 2)

    closest = min(gap[start:])
    out["closest_approach_m"] = round(closest, 2)
    out["criterion_a_margin_kept"] = closest >= margin

    rest = None
    for i in range(peak, len(t)):
        if abs(speed[i]) < REST_SPEED:
            j = i
            while j < len(t) and t[j] - t[i] < REST_HOLD_S and abs(speed[j]) < REST_SPEED:
                j += 1
            if j < len(t) and t[j] - t[i] >= REST_HOLD_S:
                rest = i
                break
    if rest is None:
        out["criterion_b_at_rest_before_wall"] = False
        out["note_b"] = "the speed never stayed under 0.1 m/s for 2 s after the peak"
    else:
        out["t_rest_s"] = round(t[rest], 2)
        out["time_peak_to_rest_s"] = round(t[rest] - t[peak], 2)
        out["rest_distance_to_wall_m"] = round(gap[rest], 2)
        out["criterion_b_at_rest_before_wall"] = gap[rest] > 0.0

    # oscillation: sign changes of the approach velocity beyond +-REVERSAL_BAND after the peak
    reversals, state = 0, 0
    for i in range(peak, len(t)):
        if speed[i] > REVERSAL_BAND:
            new = 1
        elif speed[i] < -REVERSAL_BAND:
            new = -1
        else:
            continue
        if state != 0 and new != state:
            reversals += 1
        state = new
    out["velocity_reversals_after_peak"] = reversals
    out["criterion_c_no_oscillation"] = reversals <= 1

    # peak deceleration along the approach after the peak (finite difference over a short window)
    decel = 0.0
    j = peak
    for i in range(peak, len(t)):
        while j < i and t[i] - t[j] > DECEL_WINDOW_S:
            j += 1
        if t[i] - t[j] >= DECEL_WINDOW_S * 0.5:
            decel = max(decel, (speed[j] - speed[i]) / (t[i] - t[j]))
    out["peak_decel_m_s2"] = round(decel, 2)

    try:
        con = read(prefix + "cp_brake_constraints_0.csv")
        ct = [v * 1e-6 for v in col(con, "timestamp")]
        st = [int(float(v)) for v in col(con, "state")]
        timeline, last = [], None
        for ti, si in zip(ct, st):
            if si != last:
                timeline.append([round(ti, 2), si])
                last = si
        out["state_timeline_[t_s,state]"] = timeline[:40]
        first_brake = next((ti for ti, si in zip(ct, st) if si == 4), None)
        out["t_first_braking_s"] = None if first_brake is None else round(first_brake, 2)
    except FileNotFoundError:
        out["state_timeline_[t_s,state]"] = "cp_brake_constraints was not logged"
        if require_constraints:
            out["error"] = "cp_brake_constraints was required and is missing"
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix")
    ap.add_argument("--wall-dist", type=float, required=True)
    ap.add_argument("--dir-ned", default="0,1")
    ap.add_argument("--lidar-offset", type=float, default=0.0)
    ap.add_argument("--margin", type=float, default=2.0)
    ap.add_argument("--require-constraints", action="store_true",
                    help="the run is INVALID without the cp_brake_constraints log (hook-on runs)")
    a = ap.parse_args()
    d = [float(v) for v in a.dir_ned.split(",")]
    norm = math.hypot(*d)
    result = analyse(a.prefix, a.wall_dist, (d[0] / norm, d[1] / norm), a.lidar_offset, a.margin,
                     a.require_constraints)
    if "error" in result:
        result["verdict"] = "INVALID"
    elif all(result.get(k) for k in ("criterion_a_margin_kept", "criterion_b_at_rest_before_wall",
                                     "criterion_c_no_oscillation")):
        result["verdict"] = "PASS"
    else:
        result["verdict"] = "FAIL"
    print(json.dumps(result, indent=2))
    return {"PASS": 0, "FAIL": 1, "INVALID": 2}[result["verdict"]]


if __name__ == "__main__":
    sys.exit(main())
