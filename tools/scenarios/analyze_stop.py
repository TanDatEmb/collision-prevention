#!/usr/bin/env python3
"""Metrics and verdict of one simulated approach to a wall, from `ulog2csv` output (bench tool).

usage: analyze_stop.py <csv-prefix> --wall-dist D [--dir-ned N,E] [--lidar-offset 0.0] [--margin 2.0]
                       [--require-constraints] [--min-peak-speed 2.0] [--max-gap-s 0.5] [--near-m 35] [--max-constraint-gap-s 1.0]

<csv-prefix> is the path printed by `ulog2csv -o <dir> <log>.ulg` without the topic suffix, for example
`/tmp/run1/log_0_` (files `<prefix>vehicle_local_position_0.csv`, `<prefix>cp_brake_constraints_0.csv`).
The wall face is D metres from the takeoff origin along `--dir-ned` (a unit vector in local NED; the
Gazebo +x axis is East, so `0,1`). Criteria (the owner, 2026-11-20): (a) the distance to the wall never
goes below the margin CPB_MARGIN; (b) the vehicle is at rest before the wall and at rest at the end of the
run; (c) no oscillation: at most one sign change of the approach velocity after the approach started.

Verdict (ICD-CP-05): PASS when a to c hold on valid evidence, FAIL when one does not, INVALID when the
evidence cannot support either answer. Evidence is invalid when: a value is missing or not a number, time
does not strictly increase or has a gap above --max-gap-s inside the approach, the approach never
started or never reached --min-peak-speed, the vehicle started inside the margin or beyond the wall,
or the vehicle never came within --near-m of the wall,
or the samples have a gap anywhere up to the end of the log (a stop far from the wall is not a stop at the
wall), or (with --require-constraints, for hook-on runs) the constraint log is missing or does not cover the
approach, CP was DISABLED, OBSERVER_ONLY or NOT_READY during it, SAFE_STOP occurred in it (a stop caused by
SAFE_STOP says nothing about braking), or CP never reached BRAKING. Exit code 0 PASS, 1 FAIL, 2 INVALID.
The stale-input criterion (d) is read from the state timeline of a separate run.
Bench tool: the values it is given are not approved platform values.
"""
import argparse
import csv
import json
import math
import sys

REST_SPEED = 0.1      # m/s, a rest starts below this
REST_STAY = 0.2       # m/s, a rest and a reversal band: hysteresis against noise around REST_SPEED
REST_HOLD_S = 2.0     # s the speed must stay inside the band
START_SPEED = 0.5     # m/s along the approach: the approach has started
DECEL_WINDOW_S = 0.2


class Invalid(Exception):
    pass


def read(path, columns):
    try:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
    except FileNotFoundError:
        raise Invalid("missing file %s" % path.rsplit("/", 1)[-1])
    out = {c: [] for c in columns}
    for n, r in enumerate(rows, 1):
        for c in columns:
            try:
                v = float(r[c])
            except (KeyError, ValueError, TypeError):
                raise Invalid("column %s missing or empty at row %d of %s" % (c, n, path.rsplit("/", 1)[-1]))
            if not math.isfinite(v):
                raise Invalid("column %s is not a number at row %d of %s" % (c, n, path.rsplit("/", 1)[-1]))
            out[c].append(v)
    return out


def analyse(prefix, wall_dist, direction, lidar_offset, margin, require_constraints, min_peak, max_gap, near_m,
            max_cgap):
    pos = read(prefix + "vehicle_local_position_0.csv", ["timestamp", "x", "y", "vx", "vy"])
    t = [v * 1e-6 for v in pos["timestamp"]]
    if len(t) < 3 or any(b <= a for a, b in zip(t, t[1:])):
        raise Invalid("time is missing or does not strictly increase")
    dn, de = direction
    along = [px * dn + py * de for px, py in zip(pos["x"], pos["y"])]
    speed = [a * dn + b * de for a, b in zip(pos["vx"], pos["vy"])]  # signed, along the approach
    hspeed = [math.hypot(a, b) for a, b in zip(pos["vx"], pos["vy"])]  # horizontal magnitude, for rest
    gap = [wall_dist - s - lidar_offset for s in along]  # distance from the lidar to the wall face

    out = {"samples": len(t), "assumptions": {"lidar_offset_m": lidar_offset, "wall_dist_m": wall_dist}}
    start = next((i for i, v in enumerate(speed) if v > START_SPEED), None)
    if start is None:
        raise Invalid("the approach never started: speed along the wall direction stayed under %.1f m/s "
                      "(is --dir-ned right?)" % START_SPEED)
    if gap[start] < margin:
        raise Invalid("the vehicle started inside the margin or beyond the wall (gap %.2f m)" % gap[start])
    peak = max(range(start, len(t)), key=lambda i: speed[i])
    out["t_start_s"] = round(t[start], 2)
    out["peak_speed_m_s"] = round(speed[peak], 2)
    out["t_peak_s"] = round(t[peak], 2)
    if speed[peak] < min_peak:
        raise Invalid("the peak approach speed %.2f m/s is below %.1f m/s: no real approach" % (speed[peak], min_peak))

    closest_i = min(range(start, len(t)), key=lambda i: gap[i])
    worst_dt = max(b - a for a, b in zip(t[start:], t[start + 1:]))  # through the end of the log
    if worst_dt > max_gap:
        raise Invalid("a gap of %.2f s in the samples from the approach to the end of the log (limit %.2f s)"
                      % (worst_dt, max_gap))
    closest = gap[closest_i]
    if closest > near_m:
        raise Invalid("the vehicle never came within %.0f m of the wall (closest %.1f m): a stop here is not a "
                      "stop at the wall" % (near_m, closest))
    out["closest_approach_m"] = round(closest, 2)
    out["criterion_a_margin_kept"] = closest >= margin

    # rest: the final rest of the run. The horizontal speed stays below REST_STAY from the rest onset to the
    # end of the log, the onset is a sample below REST_SPEED, and the rest lasts at least REST_HOLD_S. A rest
    # that is followed by movement beyond REST_STAY is not the final rest, so a vehicle that rests, moves
    # again and then slows into the band without ever dropping below REST_SPEED has no rest.
    k = len(t)
    while k > 0 and hspeed[k - 1] < REST_STAY:
        k -= 1
    rest = next((i for i in range(max(k, peak), len(t)) if hspeed[i] < REST_SPEED), None)
    if rest is not None and t[-1] - t[rest] < REST_HOLD_S:
        rest = None
    at_rest_at_end = rest is not None
    if rest is None:
        out["criterion_b_at_rest_before_wall"] = False
        out["note_b"] = "the speed never stayed under %.1f m/s for %.0f s after the peak" % (REST_STAY, REST_HOLD_S)
    else:
        out["t_rest_s"] = round(t[rest], 2)
        out["time_peak_to_rest_s"] = round(t[rest] - t[peak], 2)
        out["rest_distance_to_wall_m"] = round(gap[rest], 2)
        out["at_rest_at_end_of_run"] = at_rest_at_end
        out["criterion_b_at_rest_before_wall"] = bool(gap[rest] > 0.0 and at_rest_at_end)

    # oscillation: sign changes of the approach velocity beyond +-REST_STAY after the approach started
    reversals, state = 0, 0
    for i in range(start, len(t)):
        if speed[i] > REST_STAY:
            new = 1
        elif speed[i] < -REST_STAY:
            new = -1
        else:
            continue
        if state != 0 and new != state:
            reversals += 1
        state = new
    out["velocity_reversals_after_start"] = reversals
    out["criterion_c_no_oscillation"] = reversals <= 1

    decel, j = 0.0, peak
    for i in range(peak, len(t)):
        while j < i and t[i] - t[j] > DECEL_WINDOW_S:
            j += 1
        if t[i] - t[j] >= DECEL_WINDOW_S * 0.5:
            decel = max(decel, (speed[j] - speed[i]) / (t[i] - t[j]))
    out["peak_decel_m_s2"] = round(decel, 2)

    try:
        con = read(prefix + "cp_brake_constraints_0.csv", ["timestamp", "state"])
        ct = [v * 1e-6 for v in con["timestamp"]]
        if any(b < a for a, b in zip(ct, ct[1:])):  # equal stamps occur in real logs; going back is malformed
            raise Invalid("constraint time goes backwards")
        st = [int(v) for v in con["state"]]
        timeline, last = [], None
        for ti, si in zip(ct, st):
            if si != last:
                timeline.append([round(ti, 2), si])
                last = si
        out["state_timeline_[t_s,state]"] = timeline[:40]
        out["t_first_braking_s"] = next((round(ti, 2) for ti, si in zip(ct, st) if si == 4), None)
        if require_constraints:
            if not ct or ct[0] > t[start]:
                raise Invalid("the constraint log is empty or starts after the approach")
            # a row holds its state until the next row but at most max_cgap: a stopped or truncated
            # constraint stream must not look like continuous enforcement
            ends = [min(nxt, ti + max_cgap) for ti, nxt in zip(ct, ct[1:] + [float("inf")])]
            # the evidence reaches the end of the rest hold: a SAFE_STOP that supplies the rest is no braking
            t_end = max(t[closest_i], t[rest] + REST_HOLD_S if rest is not None else t[-1])
            covered_to = ct[-1] + max_cgap
            if covered_to < t_end or any(b - a > max_cgap for a, b in zip(ct, ct[1:]) if b >= t[start] and a <= t_end):
                raise Invalid("the constraint log has a gap above %.1f s or stops before the vehicle came to rest"
                              % max_cgap)
            # a state holds the half-open interval [ti, te): one that ends when the approach starts, or has no length
            # (equal stamps), was never in force during the approach
            window = [(round(ti, 2), si) for ti, te, si in zip(ct, ends, st) if ti <= t_end and te > max(ti, t[start])]
            bad = [w for w in window if w[1] in (0, 1, 2)]
            if bad:
                raise Invalid("CP was not enforcing during the approach (first [t_s, state]: %s)" % (bad[0],))
            if any(w[1] == 5 for w in window):
                raise Invalid("SAFE_STOP occurred during the approach: that stop says nothing about braking")
            if not any(w[1] == 4 for w in window):
                raise Invalid("CP never reached BRAKING during the approach")
    except Invalid as e:
        if require_constraints:
            raise
        out["state_timeline_[t_s,state]"] = "cp_brake_constraints not usable: %s" % e
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix")
    ap.add_argument("--wall-dist", type=float, required=True)
    ap.add_argument("--dir-ned", default="0,1")
    ap.add_argument("--lidar-offset", type=float, default=0.0)
    ap.add_argument("--margin", type=float, default=2.0)
    ap.add_argument("--require-constraints", action="store_true",
                    help="hook-on runs: INVALID unless the constraint log shows CP enforcing and braking")
    ap.add_argument("--min-peak-speed", type=float, default=2.0, help="bench value [m/s]")
    ap.add_argument("--max-gap-s", type=float, default=0.5, help="bench value [s]")
    ap.add_argument("--max-constraint-gap-s", type=float, default=1.0, help="bench value [s]")
    ap.add_argument("--near-m", type=float, default=35.0, help="bench value [m]: the lidar range (40 m) minus a slack")
    a = ap.parse_args()
    try:
        d = [float(v) for v in a.dir_ned.split(",")]
        norm = math.hypot(*d)
        if len(d) != 2 or norm == 0.0 or not math.isfinite(a.wall_dist) or a.wall_dist <= 0.0:
            raise Invalid("bad --dir-ned or --wall-dist")
        result = analyse(a.prefix, a.wall_dist, (d[0] / norm, d[1] / norm), a.lidar_offset, a.margin,
                         a.require_constraints, a.min_peak_speed, a.max_gap_s, a.near_m, a.max_constraint_gap_s)
        crit = ("criterion_a_margin_kept", "criterion_b_at_rest_before_wall", "criterion_c_no_oscillation")
        result["verdict"] = "PASS" if all(result.get(k) for k in crit) else "FAIL"
    except (Invalid, ValueError, OSError) as e:
        result = {"verdict": "INVALID", "error": str(e)}
    print(json.dumps(result, indent=2))
    return {"PASS": 0, "FAIL": 1, "INVALID": 2}[result["verdict"]]


if __name__ == "__main__":
    sys.exit(main())
