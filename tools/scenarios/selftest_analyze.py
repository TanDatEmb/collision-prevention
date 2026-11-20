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


def write(prefix, rows, states, nan_at=None, empty_at=None, with_constraints=True, with_position=True):
    if with_position:
        with open(prefix + "vehicle_local_position_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "x", "y", "vx", "vy"])
            for n, (t, y, vy) in enumerate(rows):
                y_cell = "nan" if n == nan_at else ("" if n == empty_at else y)
                w.writerow([int(t * 1e6), 0.0, y_cell, 0.0, vy])
    if with_constraints:
        with open(prefix + "cp_brake_constraints_0.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "state"])
            for t, s in states:
                w.writerow([int(t * 1e6), s])


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
    code = {"PASS": 0, "FAIL": 1, "INVALID": 2}.get(res["verdict"], 99)
    ok = res["verdict"] == expect and code == {"PASS": 0, "FAIL": 1, "INVALID": 2}[expect]
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
results.append(run("rest declared across a 5 s time gap (still moving)",
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
results.append(run("vehicle starts beyond the wall", series([30.0 + y for y in stop_ys()]), "INVALID"))

# noise and drift
noisy = series(stop_ys(stop_at=17.0))
noisy = [(t, y, (0.09 if (n // 5) % 2 == 0 else 0.11) if t > 14.0 else vy) for n, (t, y, vy) in enumerate(noisy)]
results.append(run("speed noise 0.09/0.11 at rest still rests", noisy, "PASS"))
drift = series(stop_ys(stop_at=17.0, total_s=12.0))
last_y = drift[-1][1]
drift += [(drift[-1][0] + DT * (k + 1), last_y + 0.3 * DT * (k + 1), 0.3) for k in range(1000)]
results.append(run("rests, then drifts toward the wall at the end", drift, "FAIL", wall=40.0))

print("%d of %d cases pass" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
