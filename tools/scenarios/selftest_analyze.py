#!/usr/bin/env python3
"""Self-test of analyze_stop.py on synthetic runs: a mass that stops short of the wall, one that
crashes through the margin, and one that oscillates. It tests the metric code only; it says nothing
about PX4."""
import csv
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def write_run(prefix, positions_along, dt=0.02):
    with open(prefix + "vehicle_local_position_0.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "x", "y", "vx", "vy"])
        prev = positions_along[0]
        for i, s in enumerate(positions_along):
            v = (s - prev) / dt if i else 0.0
            prev = s
            w.writerow([int(i * dt * 1e6), 0.0, s, 0.0, v])
    with open(prefix + "cp_brake_constraints_0.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "state"])
        w.writerow([0, 3])
        w.writerow([int(len(positions_along) * 0.5 * 0.02 * 1e6), 4])


def stop_run(v0=5.0, a=3.0, stop_at=17.0, dt=0.02, total_s=30.0, a_up=2.0):
    """Accelerate at a_up to v0, cruise, brake at a so that the mass rests exactly at `stop_at` m."""
    t_acc = v0 / a_up
    d_acc = 0.5 * v0 * t_acc
    d_brk = v0 * v0 / (2 * a)
    d_cruise = stop_at - d_acc - d_brk
    assert d_cruise >= 0.0, "stop point too close for this profile"
    t_cruise = d_cruise / v0
    t_brk = v0 / a
    out, s, v = [], 0.0, 0.0
    for i in range(int(total_s / dt)):
        t = i * dt
        if t < t_acc:
            v = a_up * t
        elif t < t_acc + t_cruise:
            v = v0
        elif t < t_acc + t_cruise + t_brk:
            v = max(0.0, v0 - a * (t - t_acc - t_cruise))
        else:
            v = 0.0
        s += v * dt
        out.append(s)
    return out


def oscillating_run(dt=0.02, total_s=30.0):
    s, out = 0.0, []
    for i in range(int(total_s / dt)):
        t = i * dt
        s = 15.0 + 2.0 * math.sin(0.8 * t) if t > 5 else 3.0 * t
        out.append(s)
    return out


def crash_run(dt=0.02, total_s=10.0):
    """Keeps flying at 5 m/s straight through the wall face (wall at 20 m)."""
    return [5.0 * i * dt for i in range(int(total_s / dt))]


def run(name, positions, wall_dist, expect):
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        write_run(prefix, positions)
        p = subprocess.run([sys.executable, os.path.join(HERE, "analyze_stop.py"), prefix, "--wall-dist",
                            str(wall_dist), "--margin", "2.0"], capture_output=True, text=True)
        import json
        res = json.loads(p.stdout)
    ok = (res.get("criterion_a_margin_kept"), res.get("criterion_b_at_rest_before_wall"),
          res.get("criterion_c_no_oscillation")) == expect[:3] and res.get("verdict") == expect[3] \
        and p.returncode == {"PASS": 0, "FAIL": 1, "INVALID": 2}[expect[3]]
    print(("PASS" if ok else "FAIL"), name, {k: res.get(k) for k in (
        "closest_approach_m", "rest_distance_to_wall_m", "velocity_reversals_after_peak",
        "peak_decel_m_s2", "criterion_a_margin_kept", "criterion_b_at_rest_before_wall",
        "criterion_c_no_oscillation", "verdict")})
    return ok


results = [
    run("stops short of the wall (22 m wall, stops at 17 m, gap 5 m)", stop_run(stop_at=17.0), 22.0, (True, True, True, "PASS")),
    run("stops but inside the margin (rests at 20.5 m, gap 1.5 m)", stop_run(stop_at=20.5), 22.0, (False, True, True, "FAIL")),
    run("flies through the wall (never stops)", crash_run(), 20.0, (False, False, True, "FAIL")),
    run("oscillates around the stop point", oscillating_run(), 30.0, (True, False, False, "FAIL")),
    run("never approaches the wall (evidence incomplete)", [0.0] * 500, 22.0, (None, None, None, "INVALID")),
]
sys.exit(0 if all(results) else 1)
