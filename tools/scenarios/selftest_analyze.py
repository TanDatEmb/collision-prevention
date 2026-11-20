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
          constraints_until=None, swap_constraint_rows=None):
    if with_position:
        with open(prefix + "vehicle_local_position_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "x", "y", "vx", "vy"])
            for n, (t, y, vy) in enumerate(rows):
                y_cell = "nan" if n == nan_at else ("" if n == empty_at else y)
                w.writerow([int(t * 1e6), 0.0, y_cell, lateral, vy])
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


results.append(ramp_case("lag 0.6004 s rounds up to 0.65, not 0.6", 1.2202, 0.65))
results.append(ramp_case("position samples missing during the response", 1.2, None, expect_code=2, gap=True))

results.append(run("reordered constraint timestamps hide SAFE_STOP", series(stop_ys(stop_at=17.0)), "INVALID",
                   states=((0, 3), (0.3, 5), (0.4, 4)), extra=REQ, swap_constraint_rows=(2, 3)))


def command_case(name, command, actual, expect_code, swap_position_rows=None):
    """measure_velocity_step.py analyze with explicit setpoint and speed functions of time (50 Hz, 20 s)."""
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        for fname, fn in (("vehicle_local_position_setpoint_0.csv", command), ("vehicle_local_position_0.csv", actual)):
            with open(prefix + fname, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp", "vy"])
                lines = [[int(n * DT * 1e6), fn(n * DT)] for n in range(1000)]
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

print("%d of %d cases pass" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
