#!/usr/bin/env python3
"""Self-test of analyze_stop.py on synthetic runs. It tests the metric and verdict code only; it says
nothing about PX4. Run: python3 -I tools/scenarios/selftest_analyze.py (exit 0 when every case passes).

Cases cover the owner's criteria (stops short, enters the margin, flies through, oscillates) and the
evidence rules (a run is INVALID rather than PASS when the evidence is weak): a spike that never
approaches, a rest declared across a time gap, SAFE_STOP the whole approach, CP disabled, CP that
never braked, a log with NaN, an empty cell, a missing log, time going backwards, a start beyond the
wall, speed noise around the rest threshold, a vehicle that drifts again after resting.
"""
import csv
import json
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DT = 0.02


def series(ys, dt=DT):
    """(t, y, vy) rows from positions along the wall direction."""
    rows, prev = [], ys[0]
    for i, y in enumerate(ys):
        rows.append((i * dt, y, (y - prev) / dt if i else 0.0))
        prev = y
    return rows


def stop_ys(v0=5.0, a=3.0, stop_at=17.0, total_s=30.0, a_up=2.0):
    t_acc, d_brk = v0 / a_up, v0 * v0 / (2 * a)
    d_cruise = stop_at - 0.5 * v0 * t_acc - d_brk
    assert d_cruise >= 0.0
    t_cruise, t_brk = d_cruise / v0, v0 / a
    out, s = [], 0.0
    for i in range(int(total_s / DT)):
        t = i * DT
        if t < t_acc:
            v = a_up * t
        elif t < t_acc + t_cruise:
            v = v0
        elif t < t_acc + t_cruise + t_brk:
            v = max(0.0, v0 - a * (t - t_acc - t_cruise))
        else:
            v = 0.0
        s += v * DT
        out.append(s)
    return out


def write(prefix, rows, states, nan_at=None, empty_at=None, with_constraints=True, with_position=True, lateral=0.0,
          constraints_until=None, swap_constraint_rows=None, heading_rad=None, north_m=0.0, hook_rows=None,
          heading_nan_at=None, pitch_rad=None, attitude_until=None, attitude_nan_at=None, truth_dy=None,
          truth_nan_at=None, truth_until=None, attitude_no_q=False):
    if pitch_rad is not None:  # vehicle_attitude: a pure pitch rotation as the quaternion (w, x, y, z), 100 Hz
        with open(prefix + "vehicle_attitude_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp"] if attitude_no_q else ["timestamp", "q[0]", "q[1]", "q[2]", "q[3]"])
            end = attitude_until if attitude_until is not None else rows[-1][0]
            for n in range(int(end / 0.01) + 1):
                w.writerow([int(n * 0.01 * 1e6)] if attitude_no_q else
                           [int(n * 0.01 * 1e6), "nan" if n == attitude_nan_at else math.cos(pitch_rad / 2), 0.0,
                            math.sin(pitch_rad / 2), 0.0])
    if truth_dy is not None:  # ground truth: the EKF track shifted by truth_dy along y, on every 2nd stamp
        with open(prefix + "vehicle_local_position_groundtruth_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "x", "y"])
            for n, (t, y, vy) in enumerate(rows):
                if n % 2 == 0 and (truth_until is None or t <= truth_until):
                    w.writerow([int(t * 1e6), north_m, "nan" if n == truth_nan_at else y + truth_dy])
    if with_position:
        with open(prefix + "vehicle_local_position_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "x", "y", "vx", "vy"] + (["heading"] if heading_rad is not None else []))
            for n, (t, y, vy) in enumerate(rows):
                y_cell = "nan" if n == nan_at else ("" if n == empty_at else y)
                w.writerow([int(t * 1e6), north_m, y_cell, lateral, vy] + ([("nan" if n == heading_nan_at else heading_rad)] if heading_rad is not None else []))
    if hook_rows is not None:
        with open(prefix + "cp_brake_hook_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "action", "vel[0]", "vel[1]", "vel_sp_out[0]", "vel_sp_out[1]"])
            w.writerows(hook_rows)
    if with_constraints:
        with open(prefix + "cp_brake_constraints_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "state"])
            # a dense log like the real one (cp_brake publishes at 20 Hz): the state in force every 0.1 s
            end = constraints_until if constraints_until is not None else rows[-1][0]
            t = states[0][0]
            lines = []
            while t <= end:
                state = [s for ts, s in states if ts <= t][-1]
                lines.append([int(t * 1e6), state])
                t += 0.1
            if swap_constraint_rows:  # reordered timestamps: the rows keep their content, their order changes
                i, j = swap_constraint_rows
                lines[i], lines[j] = lines[j], lines[i]
            w.writerows(lines)


def run(name, rows, expect, states=((0, 3), (7.5, 4)), wall=22.0, extra=(), **kw):
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        write(prefix, rows, states, **kw)
        p = subprocess.run([sys.executable, os.path.join(HERE, "analyze_stop.py"), prefix, "--wall-dist",
                            str(wall), "--margin", "2.0", *extra], capture_output=True, text=True)
    try:
        res = json.loads(p.stdout)
    except ValueError:
        res = {"verdict": "NO JSON: " + p.stderr.strip()[-60:]}
    # the process exit code is checked itself, not derived from the verdict that was printed
    ok = res["verdict"] == expect and p.returncode == {"PASS": 0, "FAIL": 1, "INVALID": 2}[expect]
    print(("PASS" if ok else "FAIL"), "%-58s" % name, "got", res["verdict"], "expected", expect,
          "|", (res.get("error") or "")[:70])
    return ok


REQ = ("--require-constraints",)
good_states = ((0, 3), (3.0, 4))
results = []

# the owner's criteria
results.append(run("stops short of the wall (gap 5 m)", series(stop_ys(stop_at=17.0)), "PASS"))
results.append(run("rests inside the margin (gap 1.5 m)", series(stop_ys(stop_at=20.5)), "FAIL"))
results.append(run("flies through the wall", series([5.0 * i * DT for i in range(500)]), "FAIL", wall=20.0))
results.append(run("oscillates around the stop point",
                   series([3.0 * i * DT if i * DT <= 5 else 15.0 + 2.0 * math.sin(0.8 * i * DT) for i in range(1500)]),
                   "FAIL", wall=30.0))
results.append(run("hook-on, CP brakes and stops short", series(stop_ys(stop_at=17.0)), "PASS", states=good_states, extra=REQ))

# evidence rules: INVALID, never PASS
results.append(run("never approaches the wall", series([0.0] * 500), "INVALID"))
results.append(run("one speed spike then zeros", [(0.0, 0.0, 0.0), (0.1, 0.0, 0.6)] + [(0.1 + 0.1 * i, 0.0, 0.0) for i in range(1, 80)],
                   "INVALID"))
results.append(run("time gap inside the approach (rest across a gap)",
                   [(0.1 * i, 0.25 * i, 5.0) for i in range(20)] + [(2.5, 10.0, 0.05), (7.5, 10.0, 3.0)], "INVALID"))
results.append(run("CP disabled during the approach", series(stop_ys()), "INVALID", states=((0, 0),), extra=REQ))
results.append(run("SAFE_STOP the whole approach, stops by itself", series(stop_ys()), "INVALID", states=((0, 5),), extra=REQ))
results.append(run("CP enforcing but never braked", series(stop_ys()), "INVALID", states=((0, 3),), extra=REQ))
results.append(run("constraint log starts after the approach", series(stop_ys()), "INVALID", states=((50, 4),), extra=REQ))
results.append(run("hook-on run without a constraint log", series(stop_ys()), "INVALID", extra=REQ, with_constraints=False))
results.append(run("NaN in a position", series(stop_ys()), "INVALID", nan_at=100))
results.append(run("empty cell in a position", series(stop_ys()), "INVALID", empty_at=100))
results.append(run("position file missing", series(stop_ys()), "INVALID", with_position=False))
rows = series(stop_ys())
rows[300] = (rows[300][0] - 5.0, rows[300][1], rows[300][2])
results.append(run("time goes backwards", rows, "INVALID"))
results.append(run("stops 68 m before the wall (never came near)", series(stop_ys(stop_at=12.0)), "INVALID", wall=80.0))
results.append(run("vehicle starts beyond the wall", series([30.0 + y for y in stop_ys()]), "INVALID"))

# noise and drift
noisy = series(stop_ys(stop_at=17.0))
noisy = [(t, y, (0.09 if (n // 5) % 2 == 0 else 0.11) if t > 14.0 else vy) for n, (t, y, vy) in enumerate(noisy)]
results.append(run("speed noise 0.09/0.11 at rest still rests", noisy, "PASS"))
drift = series(stop_ys(stop_at=17.0, total_s=12.0))
last_y = drift[-1][1]
drift += [(drift[-1][0] + DT * (k + 1), last_y + 0.3 * DT * (k + 1), 0.3) for k in range(1000)]
results.append(run("rests, then drifts toward the wall at the end", drift, "FAIL", wall=40.0))

# evidence must cover the whole run, and rest means the horizontal speed, not only the approach component
results.append(run("lateral drift of 2 m/s after the stop is not a rest", series(stop_ys(stop_at=17.0)), "FAIL", lateral=2.0))
gapy = series(stop_ys(stop_at=17.0, total_s=8.0))
gapy = gapy + [(10.0, gapy[-1][1], 0.0)]
results.append(run("samples vanish after the stop (5 s gap before the last)", gapy, "INVALID"))
results.append(run("constraint log truncated while the vehicle is still stopping", series(stop_ys(stop_at=17.0)), "INVALID",
                   states=((0, 3), (3.0, 4)), extra=REQ, constraints_until=3.5))
# the rest hold is part of the evidence, and only the final rest counts
results.append(run("SAFE_STOP supplies the rest hold", series(stop_ys(stop_at=17.0)), "INVALID",
                   states=((0, 3), (3.0, 4), (5.6, 5)), extra=REQ))
base = series(stop_ys(stop_at=17.0, total_s=8.0))
y, rows2 = base[-1][1], list(base)
tt = base[-1][0]
for v, dur in ((0.5, 1.0), (0.15, 2.5)):  # moves again, then slows into the band without reaching 0.1
    for _ in range(int(dur / DT)):
        tt += DT
        y += v * DT
        rows2.append((tt, y, v))
results.append(run("rests, moves again, then slows into the band", rows2, "FAIL", wall=40.0))
def lag_case(name, delay_s, tau_s, expect_s, rows=1000, expect_code=0):
    """measure_velocity_step.py analyze on a synthetic 5 m/s -> 0 step with a delay and a first-order lag:
    the equivalent lag must equal delay + tau (the extra distance after the step divided by the speed)."""
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        with open(prefix + "vehicle_local_position_setpoint_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "vy"])
            for n in range(rows):
                t = n * DT
                w.writerow([int(t * 1e6), "nan" if n < 20 else (5.0 if t < 10.0 else 0.0)])
        with open(prefix + "vehicle_local_position_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "vy"])
            for n in range(rows):
                t = n * DT
                after = t - 10.0 - delay_s
                v = 5.0 if after < 0.0 else 5.0 * math.exp(-after / tau_s)
                w.writerow([int(t * 1e6), v])
        p = subprocess.run([sys.executable, os.path.join(HERE, "measure_velocity_step.py"), "analyze", prefix,
                            "--speeds", "5"], capture_output=True, text=True)
    try:
        got = json.loads(p.stdout)["worst_equivalent_lag_s"]
    except (ValueError, KeyError):
        got = None
    if expect_code != 0:  # incomplete evidence: no figure may be reported and the exit code must say so
        ok = got is None and p.returncode == expect_code
    else:
        ok = got is not None and abs(got - expect_s) < 0.03 and p.returncode == 0
    print(("PASS" if ok else "FAIL"), "%-58s" % name, "got", got, "expected", expect_s)
    return ok


results.append(lag_case("step response: first-order lag 0.5 s", 0.0, 0.5, 0.5))
results.append(lag_case("step response: delay 0.2 s plus lag 0.4 s", 0.2, 0.4, 0.6))
results.append(lag_case("step response: log ends before the vehicle stops", 0.0, 0.5, None, rows=505, expect_code=2))

def ramp_case(name, ramp_s, expect_t_lat, expect_code=0, gap=False):
    """measure_velocity_step.py analyze on a 5 m/s -> 0 step answered by a linear ramp down over ramp_s: the
    equivalent lag is ramp_s / 2 (less one half sample). The upward rounding must work on the unrounded lag,
    and position samples missing during the response must be rejected, not integrated across."""
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        with open(prefix + "vehicle_local_position_setpoint_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "vy"])
            for n in range(1000):
                w.writerow([int(n * DT * 1e6), 5.0 if n * DT < 10.0 else 0.0])
        with open(prefix + "vehicle_local_position_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "vy"])
            for n in range(1000):
                t = n * DT
                if gap and 10.0 < t < 14.0:
                    continue
                after = t - 10.0
                w.writerow([int(t * 1e6), 5.0 if after < 0.0 else max(0.0, 5.0 * (1.0 - after / ramp_s))])
        p = subprocess.run([sys.executable, os.path.join(HERE, "measure_velocity_step.py"), "analyze", prefix,
                            "--speeds", "5"], capture_output=True, text=True)
    try:
        got = json.loads(p.stdout).get("t_lat_rounded_up_s")
    except ValueError:
        got = None
    ok = got == expect_t_lat and p.returncode == expect_code
    print(("PASS" if ok else "FAIL"), "%-58s" % name, "got", got, "expected", expect_t_lat)
    return ok


results.append(ramp_case("lag 0.6004 s rounds up to 0.65, not 0.6", 1.2008, 0.65))
results.append(ramp_case("linear ramp, lag 0.505 s: trapezoid, not right-endpoint", 1.01, 0.55))
results.append(ramp_case("position samples missing during the response", 1.2, None, expect_code=2, gap=True))

results.append(run("reordered constraint timestamps hide SAFE_STOP", series(stop_ys(stop_at=17.0)), "INVALID",
                   states=((0, 3), (0.3, 5), (0.4, 4)), extra=REQ, swap_constraint_rows=(2, 3)))

# a state that ends exactly when the approach starts (t = 0.30 s) was not in force during it
results.append(run("SAFE_STOP ends exactly when the approach starts", series([0.0, 0.0] + stop_ys(stop_at=17.0)), "PASS",
                   states=((0, 5), (0.3, 4)), extra=REQ))


def command_case(name, command, actual, expect_code, swap_position_rows=None, drop_position_before=None,
                 truncate_setpoint_after=None, drop_setpoint_between=None):
    """measure_velocity_step.py analyze with explicit setpoint and speed functions of time (50 Hz, 20 s)."""
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        for fname, fn in (("vehicle_local_position_setpoint_0.csv", command), ("vehicle_local_position_0.csv", actual)):
            with open(prefix + fname, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp", "vy"])
                lines = [[int(n * DT * 1e6), fn(n * DT)] for n in range(1000)]
                if drop_setpoint_between is not None and fname == "vehicle_local_position_setpoint_0.csv":
                    lo, hi = drop_setpoint_between
                    lines = [row for row in lines if not lo <= row[0] * 1e-6 < hi]
                if truncate_setpoint_after is not None and fname == "vehicle_local_position_setpoint_0.csv":
                    lines = [row for row in lines if row[0] * 1e-6 <= truncate_setpoint_after]
                if drop_position_before is not None and fname == "vehicle_local_position_0.csv":
                    lines = [row for row in lines if row[0] * 1e-6 >= drop_position_before]
                if swap_position_rows and fname == "vehicle_local_position_0.csv":
                    i, j = swap_position_rows
                    lines[i], lines[j] = lines[j], lines[i]
                w.writerows(lines)
        p = subprocess.run([sys.executable, os.path.join(HERE, "measure_velocity_step.py"), "analyze", prefix,
                            "--speeds", "5"], capture_output=True, text=True)
    ok = p.returncode == expect_code
    print(("PASS" if ok else "FAIL"), "%-58s" % name, "exit", p.returncode, "expected", expect_code)
    return ok


results.append(command_case("brief dip into the band, then moves again", lambda t: 5.0 if t < 10 else 0.0,
                            lambda t: 5.0 if t < 10 else (max(0.0, 5.0 - 10.0 * (t - 10)) if t < 10.5 else
                                                          (0.0 if t < 10.6 else 2.0)), 2))
results.append(command_case("stop during a commanded reversal is not the response",
                            lambda t: 5.0 if t < 10 else (0.0 if t < 12 else -5.0),
                            lambda t: 5.0 if t < 12 else (5.0 - 5.0 * (t - 12) if t < 14 else -5.0), 2))
results.append(command_case("clean stop is accepted", lambda t: 5.0 if t < 10 else 0.0,
                            lambda t: 5.0 if t < 10 else 5.0 * math.exp(-(t - 10) / 0.5), 0))

results.append(command_case("position rows reordered during the response", lambda t: 5.0 if t < 10 else 0.0,
                            lambda t: 5.0 if t < 10 else 5.0 * math.exp(-(t - 10) / 0.5), 2,
                            swap_position_rows=(525, 526)))
results.append(command_case("position control takes over before the stop", lambda t: 5.0 if t < 10 else
                            (0.0 if t < 12 else float("nan")),
                            lambda t: 5.0 if t < 12 else max(0.0, 5.0 - 5.0 * (t - 12)), 2))

results.append(command_case("position log starts after the step", lambda t: 5.0 if t < 10 else 0.0,
                            lambda t: 5.0 if t < 13 else 5.0 * math.exp(-(t - 13) / 0.5), 2,
                            drop_position_before=12.0))

results.append(command_case("setpoint log ends before the stop", lambda t: 5.0 if t < 10 else (0.0 if t < 12 else -5.0),
                            lambda t: 5.0 if t < 12 else max(0.0, 5.0 - 2.5 * (t - 12)), 2,
                            truncate_setpoint_after=10.5))

results.append(command_case("setpoint rows missing across the zero step", lambda t: 5.0 if t < 10 else 0.0,
                            lambda t: 5.0 if t < 10 else 5.0 * math.exp(-(t - 10) / 0.5), 2,
                            drop_setpoint_between=(10.0, 12.0)))


# 2-D obstacle distance (poles and oblique walls), the nose check, equal stamps, and the cap excess
pole_beside = series(stop_ys(stop_at=30.0))
results.append(run("1-D along the leg calls a flight beside a pole a flight through it", pole_beside, "FAIL", wall=20.0,
                   north_m=3.0))
results.append(run("2-D: passing a pole 3 m to the side keeps the margin", pole_beside, "PASS", wall=20.0, north_m=3.0,
                   extra=("--obstacle", "pole:0,20,0.15")))
results.append(run("2-D: stopping 0.85 m short of a pole in the path fails", series(stop_ys(stop_at=19.0)), "FAIL",
                   wall=20.0, extra=("--obstacle", "pole:0,20,0.15")))
results.append(run("2-D: stopping 2.85 m short of a pole in the path passes", series(stop_ys(stop_at=17.0)), "PASS",
                   wall=20.0, extra=("--obstacle", "pole:0,20,0.15")))
results.append(run("2-D: an oblique wall, 2.12 m from it at rest, passes", series(stop_ys(stop_at=17.0)), "PASS",
                   wall=20.0, extra=("--obstacle", "wall:0,20,-1,-1")))
results.append(run("2-D: an oblique wall, 1.41 m from it at rest, fails", series(stop_ys(stop_at=18.0)), "FAIL",
                   wall=20.0, extra=("--obstacle", "wall:0,20,-1,-1")))
NOSE = ("--require-nose-along-leg",)
results.append(run("nose along the leg (east) is accepted", series(stop_ys(stop_at=17.0)), "PASS", heading_rad=math.pi / 2,
                   extra=NOSE))
results.append(run("nose at north while the leg is east is INVALID", series(stop_ys(stop_at=17.0)), "INVALID",
                   heading_rad=0.0, extra=NOSE))
results.append(run("nose check without a heading in the log is INVALID", series(stop_ys(stop_at=17.0)), "INVALID",
                   extra=NOSE))
dup = series(stop_ys(stop_at=17.0))
dup[1] = (dup[0][0], dup[1][1], dup[1][2])  # two equal stamps at start-up
results.append(run("two equal position stamps are accepted", dup, "PASS"))


def field_case(name, field, expect, **kw):
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        write(prefix, series(stop_ys(stop_at=17.0)), ((0, 3), (3.0, 4)), **kw)
        p = subprocess.run([sys.executable, os.path.join(HERE, "analyze_stop.py"), prefix, "--wall-dist", "22.0"],
                           capture_output=True, text=True)
    try:
        got = json.loads(p.stdout).get(field)
    except ValueError:
        got = None
    ok = got == expect
    print(("PASS" if ok else "FAIL"), "%-58s" % name, "got", got, "expected", expect)
    return ok


# the vehicle ran 1.5 m/s above the cap in the limited rows (action 3); a transparent row is not counted
results.append(field_case("cap excess is the largest speed above the cap while limiting", "cap_excess_max_m_s", 1.5,
                          hook_rows=[(1000000, 3, 0.0, 4.0, 0.0, 3.0), (1100000, 3, 0.0, 3.5, 0.0, 2.0),
                                     (1200000, 0, 0.0, 9.0, 0.0, 0.0)]))


# a heading column that is there but holds a NaN is malformed evidence: no silent fallback to the leg bearing
nan_heading = series(stop_ys(stop_at=17.0))
results.append(run("2-D with a NaN heading sample is INVALID, not a fallback", nan_heading, "INVALID", wall=20.0,
                   heading_rad=math.pi / 2, heading_nan_at=0, extra=("--obstacle", "pole:0,20,0.15", "--lidar-offset", "1.0")))
results.append(run("2-D with no heading column keeps the leg bearing", series(stop_ys(stop_at=17.0)), "PASS", wall=20.0,
                   extra=("--obstacle", "pole:0,20,0.15", "--lidar-offset", "0.12")))


# lidar offset: projected on the leg from the heading, the pitch term, and the optional ground truth. The stop is
# at 17 m, the wall at 22 m, the leg points East (heading pi/2 is the nose along the leg)
def offset_case(name, expect_verdict, check, extra=(), stop_at=17.0, wall=22.0, **kw):
    """analyze_stop.py with a lidar offset; `check` is a predicate on the printed JSON (the numbers asserted)."""
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        write(prefix, series(stop_ys(stop_at=stop_at)), ((0, 3), (3.0, 4)), **kw)
        p = subprocess.run([sys.executable, os.path.join(HERE, "analyze_stop.py"), prefix, "--wall-dist", str(wall),
                            *extra], capture_output=True, text=True)
    try:
        res = json.loads(p.stdout)
    except ValueError:
        res = {"verdict": "NO JSON: " + p.stderr.strip()[-60:]}
    try:
        checked = bool(check(res))
    except (KeyError, TypeError):
        checked = False
    ok = res["verdict"] == expect_verdict and p.returncode == {"PASS": 0, "FAIL": 1, "INVALID": 2}[expect_verdict] and checked
    print(("PASS" if ok else "FAIL"), "%-58s" % name, "got", res["verdict"], "expected", expect_verdict,
          "| closest", res.get("closest_approach_m"), "|", (res.get("error") or "")[:60])
    return ok


def near(a, b, tol=0.03):
    return a is not None and abs(a - b) <= tol


def share(res):
    return res["lidar_offset_applied"]["offset_along_leg_m_at_closest_approach"]


OFF = ("--lidar-offset", "1.0")
for deg, cosine in ((0, 1.0), (45, math.cos(math.radians(45))), (90, 0.0)):
    results.append(offset_case("offset 1.0, nose %d deg off the leg: share %.3f" % (deg, cosine), "PASS",
                               lambda r, c=cosine: near(share(r), c, 0.001) and near(r["closest_approach_m"], 5.0 - c),
                               extra=OFF, heading_rad=math.pi / 2 - math.radians(deg)))
# the share is the projection of the nose-ward reach, so the verdict follows the heading: 1.7 m with the nose along the
# leg (FAIL under the 2.0 m margin), 2.7 m with the nose across it (PASS)
results.append(offset_case("verdict follows the projection: nose along the leg", "FAIL",
                           lambda r: near(r["closest_approach_m"], 1.7), extra=OFF, stop_at=19.3, heading_rad=math.pi / 2))
results.append(offset_case("verdict follows the projection: nose across the leg", "PASS",
                           lambda r: near(r["closest_approach_m"], 2.7), extra=OFF, stop_at=19.3, heading_rad=0.0))
results.append(offset_case("offset with nose across the leg still fails the nose check", "INVALID",
                           lambda r: True, extra=OFF + NOSE, heading_rad=0.0))
results.append(offset_case("no heading in the log: the nose is taken along the leg", "PASS",
                           lambda r: near(share(r), 1.0, 0.001) and r["lidar_offset_applied"]["heading_source"] != "log",
                           extra=OFF))
# the pitch term: nose up positive, reach = x cos(pitch) - z sin(pitch); x = 1.0, z = 0.4, pitch +-0.5 rad
reach_up = math.cos(0.5) - 0.4 * math.sin(0.5)
reach_dn = math.cos(0.5) + 0.4 * math.sin(0.5)
OFFZ = OFF + ("--lidar-offset-z", "0.4")
results.append(offset_case("pitch term, nose up 0.5 rad, z above: shorter reach", "PASS",
                           lambda r: near(share(r), reach_up, 0.001) and near(r["closest_approach_m"], 5.0 - reach_up)
                           and r["lidar_offset_applied"]["pitch_term_applied"] is True,
                           extra=OFFZ, heading_rad=math.pi / 2, pitch_rad=0.5))
results.append(offset_case("pitch term, nose down 0.5 rad, z above: longer reach", "PASS",
                           lambda r: near(share(r), reach_dn, 0.001) and near(r["closest_approach_m"], 5.0 - reach_dn),
                           extra=OFFZ, heading_rad=math.pi / 2, pitch_rad=-0.5))
results.append(offset_case("pitch term and heading together (45 deg, 0.5 rad up)", "PASS",
                           lambda r: near(share(r), reach_up * math.cos(math.pi / 4), 0.001),
                           extra=OFFZ, heading_rad=math.pi / 4, pitch_rad=0.5))
results.append(offset_case("no attitude file: pitch term skipped, z not applied, said so", "PASS",
                           lambda r: near(share(r), 1.0, 0.001) and r["lidar_offset_applied"]["pitch_term_applied"] is False
                           and "skipped" in r["lidar_offset_applied"]["pitch_term_note"],
                           extra=OFFZ, heading_rad=math.pi / 2))
results.append(offset_case("attitude file without quaternion columns is skipped", "PASS",
                           lambda r: r["lidar_offset_applied"]["pitch_term_applied"] is False, extra=OFFZ,
                           heading_rad=math.pi / 2, pitch_rad=0.5, attitude_no_q=True))
results.append(offset_case("NaN in the attitude is INVALID, no fallback", "INVALID", lambda r: "not finite" in r["error"],
                           extra=OFFZ, heading_rad=math.pi / 2, pitch_rad=0.5, attitude_nan_at=100))
results.append(offset_case("attitude log ends before the approach ends is INVALID", "INVALID",
                           lambda r: "does not cover" in r["error"], extra=OFFZ, heading_rad=math.pi / 2, pitch_rad=0.5,
                           attitude_until=5.0))
# the old default (both offsets zero): verdict and distances unchanged whatever the heading, and an attitude file is not read
for hd in (None, math.pi / 2, 0.0):
    results.append(offset_case("default offset 0: unchanged distance, heading %s" % ("none" if hd is None else "%.2f" % hd),
                               "PASS", lambda r: near(r["closest_approach_m"], 5.0, 0.02) and share(r) == 0.0,
                               heading_rad=hd))
results.append(offset_case("default offset 0 ignores a malformed attitude file", "PASS",
                           lambda r: near(r["closest_approach_m"], 5.0, 0.02) and r["lidar_offset_applied"]["pitch_term_applied"] is False,
                           heading_rad=math.pi / 2, pitch_rad=0.5, attitude_nan_at=100))
results.append(offset_case("2-D pole with the pitch term (0.5 rad up)", "PASS",
                           lambda r: near(r["closest_approach_m"], 20.0 - 0.15 - 17.0 - reach_up),
                           extra=OFFZ + ("--obstacle", "pole:0,20,0.15"), heading_rad=math.pi / 2, pitch_rad=0.5))
# ground truth: the vehicle is 0.5 m further along the leg than the EKF says; separate fields, EKF fields unchanged
results.append(offset_case("no ground truth file: no ground_truth figures", "PASS",
                           lambda r: r["ground_truth_used"] is False and "ground_truth_closest_approach_m" not in r,
                           extra=OFF, heading_rad=math.pi / 2))
results.append(offset_case("ground truth 0.5 m ahead: its own closest and rest", "PASS",
                           lambda r: r["ground_truth_used"] is True and near(r["closest_approach_m"], 4.0)
                           and near(r["ground_truth_closest_approach_m"], 3.5)
                           and near(r["ground_truth_rest_distance_to_wall_m"], r["rest_distance_to_wall_m"] - 0.5, 0.011),
                           extra=OFF, heading_rad=math.pi / 2, truth_dy=0.5))
results.append(offset_case("ground truth never changes the verdict (1.7 m EKF, 2.2 m truth)", "FAIL",
                           lambda r: near(r["closest_approach_m"], 1.7) and near(r["ground_truth_closest_approach_m"], 2.2),
                           extra=OFF, stop_at=19.3, heading_rad=math.pi / 2, truth_dy=-0.5))
results.append(offset_case("NaN in the ground truth is INVALID", "INVALID", lambda r: "not a number" in r["error"],
                           extra=OFF, heading_rad=math.pi / 2, truth_dy=0.5, truth_nan_at=100))
results.append(offset_case("ground truth ends before the approach ends is INVALID", "INVALID",
                           lambda r: "does not cover" in r["error"], extra=OFF, heading_rad=math.pi / 2, truth_dy=0.5,
                           truth_until=5.0))

print("%d of %d cases pass" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
