#!/usr/bin/env python3
"""Metrics and verdict of one simulated approach to a wall, from `ulog2csv` output (bench tool).

usage: analyze_stop.py <csv-prefix> --wall-dist D [--dir-ned N,E] [--lidar-offset 0.0] [--lidar-offset-z 0.0] [--margin 2.0]
                       [--require-constraints] [--min-peak-speed 2.0] [--max-gap-s 0.5] [--near-m 35] [--max-constraint-gap-s 1.0]
                       [--obstacle pole:N,E,R|wall:N0,E0,nN,nE] [--require-nose-along-leg] [--max-nose-error-deg 15]

<csv-prefix> is the path printed by `ulog2csv -o <dir> <log>.ulg` without the topic suffix, for example
`/tmp/run1/log_0_` (files `<prefix>vehicle_local_position_0.csv`, `<prefix>cp_brake_constraints_0.csv`).
Optional inputs, used when the file is there: `<prefix>vehicle_attitude_0.csv` (the pitch, for the lidar offset) and
`<prefix>vehicle_local_position_groundtruth_0.csv` (same local frame as the EKF, columns timestamp, x, y).
Lidar offset: the lidar sits `--lidar-offset` ahead of the vehicle origin along the BODY forward axis and
`--lidar-offset-z` above it along the body up axis. Its forward reach along the nose is
x*cos(pitch) - z*sin(pitch), pitch nose-up positive, from the logged attitude (the pitch term); without the attitude
file the pitch term is skipped (z is then not applied) and the output says so. That reach lies along the logged
heading, so in the 1-D mode its share along the leg is the cosine of the angle between the nose and the leg (1 with
the nose along the leg, or when the log has no heading); in the 2-D mode the lidar point is placed with it. The
offset applied is reported in `lidar_offset_applied`. With the groundtruth file the same distances are also
reported from it (`ground_truth_*` fields); the EKF-based fields and the verdict never use it.
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
import bisect
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


def read_optional(path, column):
    """One column of a log: a list of floats, or None when the file or the column is absent (older logs have no
    heading). A column that is there but holds an empty or non-finite value is malformed evidence, not an absent
    one: it is rejected, never replaced by another orientation."""
    try:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None or column not in reader.fieldnames:
                return None
            rows = list(reader)
    except FileNotFoundError:
        return None
    values = []
    for n, r in enumerate(rows, 1):
        try:
            v = float(r[column])
        except (ValueError, TypeError):
            raise Invalid("column %s is empty or not a number at row %d of %s" % (column, n, path.rsplit("/", 1)[-1]))
        if not math.isfinite(v):
            raise Invalid("column %s is not finite at row %d of %s" % (column, n, path.rsplit("/", 1)[-1]))
        values.append(v)
    return values or None


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def resample(ts, vs, query, first, max_gap, what):
    """Linear interpolation of the samples (ts, vs) at the times `query`. From index `first` on, the samples must
    cover the query times and have no gap above max_gap between the two samples around a query time; before it
    the nearest sample is used (nothing is judged there). Raises Invalid when the coverage is not there."""
    if any(b < a for a, b in zip(ts, ts[1:])):
        raise Invalid("%s: time goes backwards" % what)
    out = []
    for n, q in enumerate(query):
        i = bisect.bisect_right(ts, q)
        lo, hi = (i - 1, i) if 0 < i < len(ts) else (min(i, len(ts) - 1), min(i, len(ts) - 1))
        if n >= first:
            if q < ts[0] - max_gap or q > ts[-1] + max_gap:
                raise Invalid("%s does not cover the approach (time %.2f s, samples %.2f to %.2f s)"
                              % (what, q, ts[0], ts[-1]))
            if lo != hi and ts[hi] - ts[lo] > max_gap:
                raise Invalid("%s has a gap of %.2f s in the approach (limit %.2f s)" % (what, ts[hi] - ts[lo], max_gap))
        if lo == hi or ts[hi] == ts[lo]:
            out.append(vs[lo])
        else:
            w = (q - ts[lo]) / (ts[hi] - ts[lo])
            out.append(vs[lo] + w * (vs[hi] - vs[lo]))
    return out


def read_pitch(prefix):
    """(time [s], pitch [rad], nose up positive) from vehicle_attitude_0.csv (quaternion q[0..3] = w, x, y, z, body
    FRD to NED), or None when the file or the quaternion columns are absent. Malformed values raise Invalid."""
    path = prefix + "vehicle_attitude_0.csv"
    cols = [read_optional(path, c) for c in ("timestamp", "q[0]", "q[1]", "q[2]", "q[3]")]
    if any(c is None for c in cols):
        return None
    ts, pitch = [v * 1e-6 for v in cols[0]], []
    for w, x, y, z in zip(*cols[1:]):
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm == 0.0:
            raise Invalid("zero quaternion in vehicle_attitude_0.csv")
        pitch.append(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x) / (norm * norm)))))
    return ts, pitch


def read_ground_truth(prefix):
    """(time [s], x, y) of vehicle_local_position_groundtruth_0.csv, or None when the file is absent; a file that is
    there with missing, empty or non-finite columns raises Invalid (the optional input is not silently dropped)."""
    path = prefix + "vehicle_local_position_groundtruth_0.csv"
    try:
        open(path).close()
    except FileNotFoundError:
        return None
    gt = read(path, ["timestamp", "x", "y"])
    return [v * 1e-6 for v in gt["timestamp"]], gt["x"], gt["y"]


def distance_to_obstacle(spec, n, e):
    """Distance [m] from the point (n, e), local NED, to the obstacle surface; negative behind it.
    spec is ("pole", N, E, radius) or ("wall", N0, E0, nN, nE) with (nN, nE) the unit normal pointing at the vehicle."""
    if spec[0] == "pole":
        return math.hypot(n - spec[1], e - spec[2]) - spec[3]
    return (n - spec[1]) * spec[3] + (e - spec[2]) * spec[4]


def parse_obstacle(text):
    """pole:N,E,radius or wall:N0,E0,nN,nE (the normal is normalised); None for no obstacle."""
    if not text:
        return None
    kind, _, rest = text.partition(":")
    try:
        v = [float(x) for x in rest.split(",")]
    except ValueError:
        raise Invalid("bad --obstacle")
    if kind == "pole" and len(v) == 3 and all(map(math.isfinite, v)) and v[2] >= 0.0:
        return ("pole", v[0], v[1], v[2])
    if kind == "wall" and len(v) == 4 and all(map(math.isfinite, v)) and math.hypot(v[2], v[3]) > 0.0:
        norm = math.hypot(v[2], v[3])
        return ("wall", v[0], v[1], v[2] / norm, v[3] / norm)
    raise Invalid("bad --obstacle (pole:N,E,radius or wall:N0,E0,nN,nE)")


def analyse(prefix, wall_dist, direction, lidar_offset, margin, require_constraints, min_peak, max_gap, near_m,
            max_cgap, obstacle=None, require_nose=False, max_nose_error_deg=15.0, lidar_offset_z=0.0):
    pos = read(prefix + "vehicle_local_position_0.csv", ["timestamp", "x", "y", "vx", "vy"])
    t = [v * 1e-6 for v in pos["timestamp"]]
    # equal stamps occur in real logs (one at start-up); going back is malformed
    if len(t) < 3 or any(b < a for a, b in zip(t, t[1:])):
        raise Invalid("time is missing or goes backwards")
    dn, de = direction
    along = [px * dn + py * de for px, py in zip(pos["x"], pos["y"])]
    speed = [a * dn + b * de for a, b in zip(pos["vx"], pos["vy"])]  # signed, along the approach
    hspeed = [math.hypot(a, b) for a, b in zip(pos["vx"], pos["vy"])]  # horizontal magnitude, for rest
    bearing = math.atan2(de, dn)
    heading = read_optional(prefix + "vehicle_local_position_0.csv", "heading")
    start = next((i for i, v in enumerate(speed) if v > START_SPEED), None)
    if start is None:
        raise Invalid("the approach never started: speed along the wall direction stayed under %.1f m/s "
                      "(is --dir-ned right?)" % START_SPEED)

    # the lidar offset: `lidar_offset` ahead along the body forward axis and `lidar_offset_z` above it, so its reach
    # along the nose is x*cos(pitch) - z*sin(pitch) (pitch nose up positive); it lies along the heading, so its share
    # along the leg is cos(heading - bearing). Without the attitude the pitch term is skipped; with both offsets
    # zero the attitude is not needed and is not read (a bad attitude file must not void a run that does not use it).
    hd = heading if heading is not None else [bearing] * len(t)
    pitch, pitch_note = None, None
    if lidar_offset == 0.0 and lidar_offset_z == 0.0:
        pitch_note = "not needed: both lidar offsets are 0"
    else:
        att = read_pitch(prefix)
        if att is None:
            pitch_note = "skipped: vehicle_attitude_0.csv has no usable quaternion in the inputs"
        else:
            pitch = resample(att[0], att[1], t, start, max_gap, "vehicle_attitude_0.csv")
    reach = [lidar_offset * math.cos(p) - lidar_offset_z * math.sin(p) for p in pitch] if pitch is not None \
        else [lidar_offset] * len(t)  # without the pitch only the forward offset is known
    heading_source = "log" if heading is not None else "assumed along the leg"

    def distance(xs, ys):
        if obstacle is not None:
            # 2-D: the lidar point is `reach` ahead of the vehicle origin along the nose; the distance is to the
            # obstacle surface, not along the leg
            return [distance_to_obstacle(obstacle, x + r * math.cos(h), y + r * math.sin(h))
                    for x, y, r, h in zip(xs, ys, reach, hd)]
        return [wall_dist - (x * dn + y * de) - r * math.cos(h - bearing) for x, y, r, h in zip(xs, ys, reach, hd)]

    gap = distance(pos["x"], pos["y"])
    share = [r * math.cos(h - bearing) for r, h in zip(reach, hd)]  # the offset along the leg, as projected

    out = {"samples": len(t), "assumptions": {"lidar_offset_m": lidar_offset, "lidar_offset_z_m": lidar_offset_z,
                                              "wall_dist_m": wall_dist,
                                              "distance_metric": "2-D to the obstacle" if obstacle else "1-D along the leg"}}
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
    if heading is not None:
        window = range(start, closest_i + 1)
        errors = [abs(wrap_pi(heading[i] - bearing)) for i in window]
        out["heading_deg_during_approach"] = [round(math.degrees(min(heading[i] for i in window)), 1),
                                              round(math.degrees(max(heading[i] for i in window)), 1)]
        out["nose_error_deg_max"] = round(math.degrees(max(errors)), 1)
        if require_nose and max(errors) > math.radians(max_nose_error_deg):
            raise Invalid("the nose was not along the leg during the approach (largest error %.0f deg, limit %.0f)"
                          % (math.degrees(max(errors)), max_nose_error_deg))
    elif require_nose:
        raise Invalid("the log has no heading, so the nose direction cannot be checked")
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

    # the offset actually applied, from the approach start to the end of the log
    win = range(start, len(t))
    out["lidar_offset_applied"] = {
        "lidar_offset_x_m": lidar_offset, "lidar_offset_z_m": lidar_offset_z,
        "heading_source": heading_source,
        "pitch_term_applied": pitch is not None,
        "pitch_term_note": pitch_note if pitch is None else "from vehicle_attitude_0.csv, nose up positive",
        "pitch_deg_range": [round(math.degrees(min(pitch[i] for i in win)), 1),
                            round(math.degrees(max(pitch[i] for i in win)), 1)] if pitch is not None else None,
        "forward_reach_along_nose_m_range": [round(min(reach[i] for i in win), 4), round(max(reach[i] for i in win), 4)],
        "offset_along_leg_m_range": [round(min(share[i] for i in win), 4), round(max(share[i] for i in win), 4)],
        "offset_along_leg_m_at_closest_approach": round(share[closest_i], 4),
        "offset_along_leg_m_at_rest": round(share[rest], 4) if rest is not None else None}

    # optional ground truth: the same distances from the simulator position, as separate fields; the EKF-based
    # fields above and the verdict never use them
    truth = read_ground_truth(prefix)
    if truth is not None:
        tx = resample(truth[0], truth[1], t, start, max_gap, "vehicle_local_position_groundtruth_0.csv")
        ty = resample(truth[0], truth[2], t, start, max_gap, "vehicle_local_position_groundtruth_0.csv")
        gap_truth = distance(tx, ty)
        out["ground_truth_used"] = True
        out["ground_truth_closest_approach_m"] = round(min(gap_truth[i] for i in win), 2)
        out["ground_truth_rest_distance_to_wall_m"] = round(gap_truth[rest], 2) if rest is not None else None
    else:
        out["ground_truth_used"] = False

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

    # tracking of the velocity cap: how far above the cap the vehicle ran while the hook was limiting
    try:
        with open(prefix + "cp_brake_hook_0.csv", newline="") as f:
            hook = list(csv.DictReader(f))
        excess = []
        for r in hook:
            if int(float(r["action"])) == 3:  # ACTION_LIMITED
                actual = math.hypot(float(r["vel[0]"]), float(r["vel[1]"]))
                capped = math.hypot(float(r["vel_sp_out[0]"]), float(r["vel_sp_out[1]"]))
                if math.isfinite(actual) and math.isfinite(capped):
                    excess.append(actual - capped)
        if excess:
            out["cap_excess_max_m_s"] = round(max(excess), 2)
            out["limited_samples"] = len(excess)
    except (FileNotFoundError, KeyError, ValueError):
        pass

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
    ap.add_argument("--lidar-offset", type=float, default=0.0,
                    help="[m] the lidar sits this far ahead of the vehicle origin along the BODY forward axis; its share "
                         "along the leg is projected from the logged heading (cos of the nose to leg angle)")
    ap.add_argument("--lidar-offset-z", type=float, default=0.0,
                    help="[m] optional: the lidar sits this far ABOVE the vehicle origin along the body up axis; with the "
                         "logged attitude the forward reach is x*cos(pitch) - z*sin(pitch), pitch nose-up positive "
                         "(from vehicle_attitude_0.csv; without that file the pitch term is skipped and the output says so)")
    ap.add_argument("--margin", type=float, default=2.0)
    ap.add_argument("--require-constraints", action="store_true",
                    help="hook-on runs: INVALID unless the constraint log shows CP enforcing and braking")
    ap.add_argument("--min-peak-speed", type=float, default=2.0, help="bench value [m/s]")
    ap.add_argument("--max-gap-s", type=float, default=0.5, help="bench value [s]")
    ap.add_argument("--max-constraint-gap-s", type=float, default=1.0, help="bench value [s]")
    ap.add_argument("--near-m", type=float, default=35.0, help="bench value [m]: the lidar range (40 m) minus a slack")
    ap.add_argument("--obstacle", default=None,
                    help="2-D distance to a pole (pole:N,E,radius) or an oblique wall (wall:N0,E0,nN,nE, the normal "
                         "points at the vehicle), local NED; default is the 1-D distance along --dir-ned")
    ap.add_argument("--require-nose-along-leg", action="store_true",
                    help="INVALID unless the heading stayed within --max-nose-error-deg of the leg direction")
    ap.add_argument("--max-nose-error-deg", type=float, default=15.0, help="bench value [deg]")
    a = ap.parse_args()
    try:
        d = [float(v) for v in a.dir_ned.split(",")]
        norm = math.hypot(*d)
        if len(d) != 2 or norm == 0.0 or not math.isfinite(a.wall_dist) or a.wall_dist <= 0.0 \
                or not math.isfinite(a.lidar_offset) or not math.isfinite(a.lidar_offset_z):
            raise Invalid("bad --dir-ned, --wall-dist or lidar offset")
        result = analyse(a.prefix, a.wall_dist, (d[0] / norm, d[1] / norm), a.lidar_offset, a.margin,
                         a.require_constraints, a.min_peak_speed, a.max_gap_s, a.near_m, a.max_constraint_gap_s,
                         parse_obstacle(a.obstacle), a.require_nose_along_leg, a.max_nose_error_deg, a.lidar_offset_z)
        crit = ("criterion_a_margin_kept", "criterion_b_at_rest_before_wall", "criterion_c_no_oscillation")
        result["verdict"] = "PASS" if all(result.get(k) for k in crit) else "FAIL"
    except (Invalid, ValueError, OSError) as e:
        result = {"verdict": "INVALID", "error": str(e)}
    print(json.dumps(result, indent=2))
    return {"PASS": 0, "FAIL": 1, "INVALID": 2}[result["verdict"]]


if __name__ == "__main__":
    sys.exit(main())
