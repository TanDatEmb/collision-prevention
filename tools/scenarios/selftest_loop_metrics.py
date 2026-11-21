#!/usr/bin/env python3
"""Self-test of measure_velocity_step.py (analyze) on synthetic responses of known shape. It tests the metric
and verdict code only; it says nothing about PX4. Run: python3 -I tools/scenarios/selftest_loop_metrics.py
(exit 0 when every case passes). Optional: --tool <path> runs the cases against another copy of the tool, for
example the previous version, to show which cases it fails.

The responses are exact: a linear system (first order, critically damped, underdamped second order) driven by
steps and ramps is the sum of its closed-form step response S(t) and ramp response R(t). The expected numbers
come from those closed forms (bisection on S, the analytic overshoot, the analytic ramp lag), not from the
code under test. Cases marked [old rule] test an evidence rule the tool already had; the others test new output
or new rules.
"""
import csv
import importlib.util
import json
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "measure_velocity_step.py")
if "--tool" in sys.argv:
    TOOL = sys.argv[sys.argv.index("--tool") + 1]
POS_DT = 0.02
SP_DT = 0.05


class First:
    """y' = (r - y) / tau"""
    def __init__(self, tau):
        self.tau = tau

    def S(self, t):
        return 1.0 - math.exp(-t / self.tau)

    def R(self, t):
        return t - self.tau * self.S(t)


class Critical:
    """wn^2 / (s + wn)^2"""
    def __init__(self, wn):
        self.wn = wn

    def S(self, t):
        x = self.wn * t
        return 1.0 - (1.0 + x) * math.exp(-x)

    def R(self, t):
        return t - 2.0 / self.wn + (2.0 / self.wn + t) * math.exp(-self.wn * t)


class Under:
    """wn^2 / (s^2 + 2 zeta wn s + wn^2), zeta < 1"""
    def __init__(self, wn, zeta):
        self.wn, self.zeta = wn, zeta
        self.sigma = zeta * wn
        self.wd = wn * math.sqrt(1.0 - zeta * zeta)

    def S(self, t):
        e = math.exp(-self.sigma * t)
        return 1.0 - e * (math.cos(self.wd * t) + self.sigma / self.wd * math.sin(self.wd * t))

    def R(self, t):
        e = math.exp(-self.sigma * t)
        return (t - 2.0 * self.zeta / self.wn
                + e * (2.0 * self.zeta / self.wn * math.cos(self.wd * t)
                       + (2.0 * self.zeta ** 2 - 1.0) / (self.wn * math.sqrt(1.0 - self.zeta ** 2)) * math.sin(self.wd * t)))


def response(system, inputs, t):
    """Output at t of the system driven by inputs: ("step", t0, change) and ("ramp", t0, slope change)."""
    y = 0.0
    for kind, t0, c in inputs:
        if t >= t0:
            y += c * (system.S(t - t0) if kind == "step" else system.R(t - t0))
    return y


def command(inputs, t):
    r = 0.0
    for kind, t0, c in inputs:
        if t >= t0:
            r += c if kind == "step" else c * (t - t0)
    return r


def ramp_block(t_up, sign, speed, decel, plateau_s):
    """A step to sign*speed at t_up, held, then a linear fall to zero at decel; returns (inputs, t_ramp, t_end)."""
    t_ramp = t_up + plateau_s
    t_end = t_ramp + speed / decel
    return [("step", t_up, sign * speed), ("ramp", t_ramp, -sign * decel), ("ramp", t_end, sign * decel)], t_ramp, t_end


def bisect_time(f, level):
    """First time f reaches level: a fine scan to the first sample at or above it, then bisection."""
    k = 0
    while f(k * 0.001) < level:
        k += 1
    lo, hi = (k - 1) * 0.001, k * 0.001
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) < level else (lo, mid)
    return 0.5 * (lo + hi)


def last_outside(system, band, hi=40.0):
    """Last time |S - 1| exceeds band, on a fine grid."""
    out, n = 0.0, int(hi / 0.001)
    for k in range(n):
        if abs(system.S(k * 0.001) - 1.0) > band:
            out = k * 0.001
    return out


def ts_us(t):
    return int(round(t * 1e6))


def write_logs(prefix, system, inputs, t_end, pos_keep=None, sp_until=None, swap_pos=None, pos_from=0.0, sp_from=0.0, sp_hold=None, sp_dt=SP_DT, pos_dt=POS_DT, sp_nan=None):
    with open(prefix + "vehicle_local_position_0.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "vx", "vy"])
        rows = []
        for k in range(int(round(t_end / pos_dt)) + 1):
            t = k * pos_dt
            if t >= pos_from and (pos_keep is None or pos_keep(t)):
                rows.append([ts_us(t), 0.0, response(system, inputs, t)])
        if swap_pos:
            i, j = swap_pos
            rows[i], rows[j] = rows[j], rows[i]
        w.writerows(rows)
    with open(prefix + "vehicle_local_position_setpoint_0.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "vx", "vy"])
        for k in range(int(round(t_end / sp_dt)) + 1):
            t = k * sp_dt
            if sp_from <= t and (sp_until is None or t <= sp_until):
                # sp_hold: the setpoint is updated every sp_hold s and the rows in between repeat it
                t_cmd = t if sp_hold is None else math.floor(t / sp_hold + 1e-9) * sp_hold
                nan = sp_nan is not None and sp_nan[0] <= t <= sp_nan[1]  # position control took over: no velocity setpoint
                w.writerow([ts_us(t), 0.0, "nan" if nan else command(inputs, t_cmd)])


def quat(roll_deg, pitch_deg):
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    return [cr * cp, sr * cp, cr * sp, -sr * sp]  # yaw zero


def write_attitude(prefix, t_end, rate_fn, angle_fn, cmd_fn, dt=0.004, keep=None, cmd_dt=0.01, back=False, att_keep=None,
                   cmd_keep=None):
    """rate_fn(t) -> (roll, pitch, yaw) rates [deg/s]; angle_fn / cmd_fn(t) -> (roll, pitch) [deg]."""
    n = int(round(t_end / dt)) + 1
    ts = [k * dt for k in range(n) if keep is None or keep(k * dt)]
    with open(prefix + "vehicle_angular_velocity_0.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "xyz[0]", "xyz[1]", "xyz[2]"])
        rows = [[ts_us(t)] + [math.radians(v) for v in rate_fn(t)] for t in ts]
        if back:
            rows[100], rows[101] = rows[101], rows[100]
        w.writerows(rows)
    with open(prefix + "vehicle_attitude_0.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "q[0]", "q[1]", "q[2]", "q[3]"])
        for t in ts:
            if att_keep is None or att_keep(t):
                w.writerow([ts_us(t)] + quat(*angle_fn(t)))
    with open(prefix + "vehicle_attitude_setpoint_0.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "q_d[0]", "q_d[1]", "q_d[2]", "q_d[3]"])
        for k in range(int(round(t_end / cmd_dt)) + 1):
            if cmd_keep is None or cmd_keep(k * cmd_dt):
                w.writerow([ts_us(k * cmd_dt)] + quat(*cmd_fn(k * cmd_dt)))


def run_tool(prefix, extra):
    p = subprocess.run([sys.executable, TOOL, "analyze", prefix, *extra], capture_output=True, text=True)
    try:
        res = json.loads(p.stdout)
    except ValueError:
        res = {"error": "NO JSON: " + (p.stderr.strip()[-80:] or p.stdout[:80])}
    return p.returncode, res


results = []


def report(name, ok, detail=""):
    print(("PASS" if ok else "FAIL"), "%-62s" % name, detail)
    results.append(ok)


def near(x, ref, tol):
    return x is not None and abs(x - ref) <= tol


def valid_case(name, build, extra, check):
    """The tool must exit 0 and check(res) must return (ok, detail)."""
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        build(prefix)
        rc, res = run_tool(prefix, extra)
    try:
        ok, detail = check(res)
    except (KeyError, IndexError, TypeError) as e:
        ok, detail = False, "missing output %s" % e
    report(name, ok and rc == 0, "rc=%s %s" % (rc, detail if detail else (res.get("error") or "")[:90]))


def invalid_case(name, build, extra, substring):
    """The tool must exit 2 with an INVALID error that names the reason (not a usage error, not a crash).
    substring may be a tuple of accepted wordings (an old rule had other words in the previous tool)."""
    wordings = substring if isinstance(substring, tuple) else (substring,)
    with tempfile.TemporaryDirectory() as d:
        prefix = os.path.join(d, "log_0_")
        build(prefix)
        rc, res = run_tool(prefix, extra)
    err = res.get("error") or ""
    report(name, rc == 2 and err.startswith("INVALID") and any(w in err for w in wordings), "rc=%s %s" % (rc, err[:110]))


def step_log(system, speed=5.0, t_up=2.0, t_down=12.0, t_end=24.0, **kw):
    inputs = [("step", t_up, speed), ("step", t_down, -speed)]
    return lambda prefix: write_logs(prefix, system, inputs, t_end, **kw)


def check_step_shape(system, overshoot, ringing, lag=None):
    """Both steps (up and down to zero) must show the closed-form rise, overshoot, settling and ringing."""
    t10, t90 = bisect_time(system.S, 0.1), bisect_time(system.S, 0.9)
    t_settle = last_outside(system, 0.02)

    def check(res):
        msgs, ok = [], True
        for rec in (res["steps_up"][0], res["steps_down_to_zero"][0]):
            good = (near(rec["rise_10_90_s"], t90 - t10, 0.01)
                    and near(rec["overshoot_percent"], overshoot, 1.0 if overshoot else 0.05)
                    and near(rec["settle_2pct_s"], t_settle, 0.03)
                    and rec["ringing_count"] == ringing)
            ok = ok and good
            msgs.append("rise %.3f/%.3f os %.1f/%.1f settle %.2f/%.2f ring %s/%s" % (
                rec["rise_10_90_s"] or -1, t90 - t10, rec["overshoot_percent"], overshoot,
                rec["settle_2pct_s"] or -1, t_settle, rec["ringing_count"], ringing))
        if lag is not None:
            down = res["steps_down_to_zero"][0]
            ok = ok and near(down["equivalent_lag_s"], lag, 0.02)
            msgs.append("lag %.3f/%.3f" % (down["equivalent_lag_s"], lag))
        return ok, "; ".join(msgs[:1] + msgs[1:])
    return check


# ---- steps of known shape
crit = Critical(3.0)
valid_case("critically damped step: rise, no overshoot, no ringing, lag 2/wn", step_log(crit), ["--speeds", "5"],
           check_step_shape(crit, 0.0, 0, lag=2.0 / 3.0))
zeta = 0.25
under = Under(4.0, zeta)
overshoot_exact = 100.0 * math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta * zeta))
valid_case("underdamped step: overshoot 44.4 %, ringing 3, rise", step_log(under), ["--speeds", "5"],
           check_step_shape(under, overshoot_exact, 3))
first = First(0.4)
valid_case("first-order step: rise 2.197 tau, no overshoot, lag tau", step_log(first), ["--speeds", "5"],
           check_step_shape(first, 0.0, 0, lag=0.4))
valid_case("[old output] first-order step: the 63 % time is tau, CPB_T_LAT 0.4", step_log(first), ["--speeds", "5"],
           lambda res: (near(res["steps_down_to_zero"][0]["t63_s"], 0.4, 0.03) and res["t_lat_rounded_up_s"] == 0.4,
                        "t63 %s t_lat %s" % (res["steps_down_to_zero"][0]["t63_s"], res["t_lat_rounded_up_s"])))

# ---- ramps
tau = 0.5


def ramps_log(system, blocks, rest_s=8.0, plateau_s=8.0, **kw):
    inputs, t_up, ends = [], 2.0, []
    for sign, decel in blocks:
        block, t_ramp, t_end = ramp_block(t_up, sign, 8.0, decel, plateau_s)
        inputs += block
        ends.append(t_end)
        t_up = t_end + rest_s
    return inputs, ends, (lambda prefix: write_logs(prefix, system, inputs, t_up, **kw))


inputs3, ends3, build3 = ramps_log(First(tau), [(1, 1.0), (-1, 2.0), (1, 3.0)])


def check_ramps(expected_lag, expected_overshoot_for=None):
    def check(res):
        recs = res["ramps_down_to_zero"]
        ok = len(recs) == 3
        msgs = []
        for rec, decel in zip(recs, (1.0, 2.0, 3.0)):
            good = near(rec["ramp_following_lag_s"], expected_lag, 0.02) and near(rec["decel_m_s2"], decel, 0.02)
            ok = ok and good
            msgs.append("a=%s lag %s" % (rec["decel_m_s2"], rec["ramp_following_lag_s"]))
        return ok and near(res["worst_ramp_following_lag_s"], expected_lag, 0.02), "; ".join(msgs)
    return check


valid_case("first-order ramps 1, 2, 3 m/s^2 (signs alternate): lag = tau", build3,
           ["--speeds", "", "--ramp-decels", "1,2,3"], check_ramps(tau))
valid_case("first-order ramps: final overshoot is 0 and no ringing", build3, ["--speeds", "", "--ramp-decels", "1,2,3"],
           lambda res: (all(near(r["final_overshoot_percent"], 0.0, 0.05) and r["ringing_count"] == 0
                            for r in res["ramps_down_to_zero"]), ""))
zeta_r, wn_r = 0.5, 4.0
second = Under(wn_r, zeta_r)
inputs_s, ends_s, build_s = ramps_log(second, [(1, 2.0)])


def check_second_order_ramp(res):
    rec = res["ramps_down_to_zero"][0]
    t_end = ends_s[0]
    grid = [t_end + k * 0.001 for k in range(8000)]
    under_exact = max(0.0, -min(response(second, inputs_s, t) for t in grid))
    ok = (near(rec["ramp_following_lag_s"], 2.0 * zeta_r / wn_r, 0.02)
          and under_exact > 0.1 and near(rec["final_overshoot_m_s"], under_exact, 0.01))
    return ok, "lag %s/%.3f undershoot %s/%.3f m/s ringing %s" % (
        rec["ramp_following_lag_s"], 2.0 * zeta_r / wn_r, rec["final_overshoot_m_s"], under_exact, rec["ringing_count"])


valid_case("underdamped ramp: lag 2 zeta/wn and the closed-form final overshoot", build_s,
           ["--speeds", "", "--ramp-decels", "2"], check_second_order_ramp)

# ---- attitude chatter, tilt, yaw rate
TAU_B, THETA, ROLL_PEAK, YAW_RATE = 0.4, 8.0, 3.0, 30.0
BUMPS = [(2.0, 1.0), (12.0, -1.0)]  # (command time, sign)


def bump(t, scale):
    """scale * x exp(1 - x) after each command, x in units of TAU_B: peak scale at x = 1."""
    v = 0.0
    for t0, s in BUMPS:
        if t >= t0:
            x = (t - t0) / TAU_B
            v += s * scale * x * math.exp(1.0 - x)
    return v


def bump_rate(t, scale):
    """The derivative of bump() [per s]."""
    v = 0.0
    for t0, s in BUMPS:
        if t >= t0:
            x = (t - t0) / TAU_B
            v += s * scale / TAU_B * (1.0 - x) * math.exp(1.0 - x)
    return v


def attitude_build(ripple_hz=0.0, ripple_deg_s=0.0, **kw):
    def build(prefix):
        step_log(first)(prefix)
        write_attitude(
            prefix, 24.0,
            lambda t: (bump_rate(t, ROLL_PEAK),
                       bump_rate(t, THETA) + ripple_deg_s * math.sin(2 * math.pi * ripple_hz * t), YAW_RATE),
            lambda t: (bump(t, ROLL_PEAK), bump(t, THETA)),
            lambda t: (1.5 * bump(t, ROLL_PEAK), 1.5 * bump(t, THETA)), **kw)
    return build


def rms_window(fn, t0, length):
    n = 4000
    return math.sqrt(sum(fn(t0 + length * k / n) ** 2 for k in range(n + 1)) / (n + 1))


def check_chatter(want_high):
    def check(res):
        msgs, ok = [], True
        for rec in (res["steps_up"][0], res["steps_down_to_zero"][0]):
            frac_p = rec["pitch_rate_power_above_2hz_fraction"]
            frac_r = rec["roll_rate_power_above_2hz_fraction"]
            length, t0 = rec["settle_s"], rec["t_s"]
            rms_expected = rms_window(lambda t: bump_rate(t, THETA), t0, length)
            good = (frac_r < 0.05 and (frac_p > 0.3 if want_high else frac_p < 0.05)
                    and near(rec["roll_rate_rms_deg_s"], rms_window(lambda t: bump_rate(t, ROLL_PEAK), t0, length), 0.1)
                    and (want_high or near(rec["pitch_rate_rms_deg_s"], rms_expected, 0.05 * rms_expected))
                    and near(rec["peak_pitch_deg"], THETA, 0.05) and near(rec["peak_roll_deg"], ROLL_PEAK, 0.05)
                    and near(rec["peak_pitch_cmd_deg"], 1.5 * THETA, 0.05) and near(rec["peak_roll_cmd_deg"], 1.5 * ROLL_PEAK, 0.05)
                    and near(rec["yaw_rate_mean_deg_s"], YAW_RATE, 0.01))
            ok = ok and good
            msgs.append("pitch hf %.3f roll hf %.3f rms %.2f peak %.2f/%.2f" % (
                frac_p, frac_r, rec["pitch_rate_rms_deg_s"], rec["peak_pitch_deg"], rec["peak_roll_deg"]))
        return ok and res["logs"] == {"angular_velocity": True, "attitude": True, "attitude_setpoint": True}, "; ".join(msgs)
    return check


valid_case("clean pitch rate: power above 2 Hz low, rms, peak tilt, yaw rate", attitude_build(), ["--speeds", "5"],
           check_chatter(False))
valid_case("12 Hz ripple on the pitch rate: power above 2 Hz high, roll clean", attitude_build(12.0, 20.0), ["--speeds", "5"],
           check_chatter(True))

# ---- a falling setpoint nobody asked for is listed, not rejected (no --ramp-decels)
stray = [("step", 2.0, 5.0), ("step", 12.0, -5.0), ("step", 26.0, 8.0), ("ramp", 36.0, -1.0), ("ramp", 38.0, -2.0), ("ramp", 40.0, 3.0)]
valid_case("unplanned invalid ramp is listed with its problems and does not reject the steps",
           lambda prefix: write_logs(prefix, First(tau), stray, 60.0), ["--speeds", "5"],
           lambda res: (res["all_ramps_found"] == 1 and len(res["ramps_down_to_zero"][0]["problems"]) > 0
                        and res["t_lat_rounded_up_s"] == 0.5 and "worst_ramp_following_lag_s" not in res,
                        "ramps %s" % res["all_ramps_found"]))

# ---- review findings (PR 145): each case below failed on the first version of this PR
# a sample-and-hold ramp: the setpoint updates every 0.1 s, the log has a row every 0.05 s, so values repeat
zoh_inputs, zoh_ends, _ = ramps_log(First(tau), [(1, 1.0)])
valid_case("ramp whose setpoint rows repeat between updates is still measured",
           lambda prefix: write_logs(prefix, First(tau), zoh_inputs, zoh_ends[0] + 8.0, sp_hold=0.1),
           ["--speeds", "", "--ramp-decels", "1"],
           lambda res: (res["all_ramps_found"] == 1 and near(res["ramps_down_to_zero"][0]["ramp_following_lag_s"], tau, 0.04)
                        and near(res["ramps_down_to_zero"][0]["decel_m_s2"], 1.0, 0.05),
                        "found %s" % res["all_ramps_found"]))
# a fast loop on a slow ramp: the speed is inside the settle band before the ramp ends
slow_inputs, slow_t, slow_end = ramp_block(2.0, 1, 3.0, 0.25, 8.0)
valid_case("settling of a ramp is searched only after the ramp end",
           lambda prefix: write_logs(prefix, First(0.2), slow_inputs, slow_end + 8.0), ["--speeds", "", "--ramp-decels", "0.25"],
           lambda res: (res["ramps_down_to_zero"][0]["settle_s_after_ramp"] >= 0.0,
                        "settle_s_after_ramp %s" % res["ramps_down_to_zero"][0]["settle_s_after_ramp"]))
# a maneuver with no roll at all: the zero roll-rate power is no reason to reject the response
def zero_roll_build(prefix):
    step_log(first)(prefix)
    write_attitude(prefix, 24.0, lambda t: (0.0, bump_rate(t, THETA), 0.0), lambda t: (0.0, bump(t, THETA)),
                   lambda t: (0.0, bump(t, THETA)))


valid_case("zero roll rate in a response is a zero fraction, not INVALID", zero_roll_build, ["--speeds", "5"],
           lambda res: (res["steps_up"][0]["roll_rate_power_above_2hz_fraction"] == 0.0
                        and res["steps_up"][0]["pitch_rate_power_above_2hz_fraction"] is not None
                        and res["steps_up"][0]["roll_rate_rms_deg_s"] == 0.0, ""))
invalid_case("two requested ramp decelerations are not both met by one recorded ramp",
             ramps_log(First(tau), [(1, 1.0)])[2], ["--speeds", "", "--ramp-decels", "1,1.2"], "('ramp', 1.2)")


# overlapping tolerance ranges: ramps at 0.7 and 1.05 m/s^2 do satisfy the requests 1 and 1.2 (0.7 -> 1, 1.05 -> 1.2)
valid_case("close ramp requests are matched to distinct ramps by a full matching, not greedily",
           ramps_log(First(tau), [(1, 0.7), (-1, 1.05)])[2], ["--speeds", "", "--ramp-decels", "1,1.2"],
           lambda res: (len(res["ramps_down_to_zero"]) == 2 and near(res["ramps_down_to_zero"][0]["decel_m_s2"], 0.7, 0.05)
                        and near(res["ramps_down_to_zero"][1]["decel_m_s2"], 1.05, 0.05), ""))


# the step-up requirement is opt-in: a log with the step down only (as the earlier rules accepted) stays valid
valid_case("log with a step down only is valid without --require-steps-up",
           step_log(first, pos_from=6.0, sp_from=6.0), ["--speeds", "5"],
           lambda res: (res["steps_up"] == [] and res["t_lat_rounded_up_s"] == 0.4, ""))


# ---- first real SITL run (parent session): two logging artefacts made every response INVALID
# 1. the setpoint log is 10 Hz and the ramp starts between two rows (8.07 s after a step that is on a row): the first row
#    after the plateau is a partial step; the steps themselves are on rows (rest 7.93 s)
ramps10_inputs, ramps10_ends, build10 = ramps_log(First(tau), [(1, 2.0), (-1, 3.0)], plateau_s=8.07, rest_s=7.93, sp_dt=0.1)
valid_case("perfect ramps logged at 10 Hz, starting between two rows, are valid", build10,
           ["--speeds", "", "--ramp-decels", "2,3"],
           lambda res: (len(res["ramps_down_to_zero"]) == 2
                        and all(near(r["ramp_following_lag_s"], tau, 0.03) for r in res["ramps_down_to_zero"])
                        and all(r["setpoint_nonlinearity_m_s"] < 0.02 for r in res["ramps_down_to_zero"]),
                        str([r["setpoint_nonlinearity_m_s"] for r in res["ramps_down_to_zero"]])))
invalid_case("a stepped ramp (setpoint changes every 0.5 s) is still INVALID",
             ramps_log(First(tau), [(1, 1.0)], sp_hold=0.5)[2], ["--speeds", "", "--ramp-decels", "1"],
             ("not linear", "no complete response"))
# 2. the attitude setpoint is logged at about 18 to 21 Hz, the attitude and the rates much faster
valid_case("attitude setpoint at 18 Hz does not invalidate a response that uses rates and attitude",
           attitude_build(cmd_dt=0.056), ["--speeds", "5"], check_chatter(False))


def cmd_hole_build(prefix):
    step_log(first)(prefix)
    write_attitude(prefix, 24.0, lambda t: (bump_rate(t, ROLL_PEAK), bump_rate(t, THETA), 0.0),
                   lambda t: (bump(t, ROLL_PEAK), bump(t, THETA)), lambda t: (1.5 * bump(t, ROLL_PEAK), 1.5 * bump(t, THETA)),
                   cmd_keep=lambda t: not (12.3 < t < 13.2))


valid_case("a hole in the attitude setpoint log: commanded tilt not available, response still valid", cmd_hole_build,
           ["--speeds", "5"],
           lambda res: (res["steps_down_to_zero"][0]["peak_pitch_cmd_deg"] is None
                        and any("vehicle_attitude_setpoint" in n for n in res["steps_down_to_zero"][0]["notes"])
                        and res["steps_down_to_zero"][0]["peak_pitch_deg"] is not None, ""))


def att_hole_build(prefix):
    step_log(first)(prefix)
    write_attitude(prefix, 24.0, lambda t: (bump_rate(t, ROLL_PEAK), bump_rate(t, THETA), 0.0),
                   lambda t: (bump(t, ROLL_PEAK), bump(t, THETA)), lambda t: (0.0, 0.0),
                   att_keep=lambda t: not (12.3 < t < 12.6))


invalid_case("a hole in the attitude log is INVALID and names that topic", att_hole_build, ["--speeds", "5"], "vehicle_attitude log")
# the problems name the topic that failed
invalid_case("a position gap names the position log", step_log(first, pos_keep=lambda t: not (12.2 < t < 13.0)),
             ["--speeds", "5"], "vehicle_local_position log")
invalid_case("a setpoint coverage gap names the setpoint log", step_log(first, sp_until=13.0), ["--speeds", "5"],
             "vehicle_local_position_setpoint log")


class Scaled:
    """The same system with a steady-state gain."""
    def __init__(self, system, gain):
        self.system, self.gain = system, gain

    def S(self, t):
        return self.gain * self.system.S(t)

    def R(self, t):
        return self.gain * self.system.R(t)


valid_case("settling to 2 percent that never happens says why (null with a note)",
           lambda prefix: write_logs(prefix, Scaled(first, 0.97), [("step", 2.0, 3.0), ("step", 12.0, -3.0)], 24.0),
           ["--speeds", "3"],
           lambda res: (res["steps_up"][0]["settle_2pct_s"] is None
                        and any("2 percent" in n for n in res["steps_up"][0]["notes"]), str(res["steps_up"][0].get("notes"))))


# ---- second review round on b727bfd
# slope changes in the first or last 10 percent of the fall are outside the fitted body and must still be flagged
t_r2 = 2.0 + 8.0
slow_start = [("step", 2.0, 8.0), ("ramp", t_r2, -0.1), ("ramp", t_r2 + 7.0, -0.9), ("ramp", t_r2 + 14.3, 1.0)]
slow_end = [("step", 2.0, 8.0), ("ramp", t_r2, -1.0), ("ramp", t_r2 + 7.3, 0.9), ("ramp", t_r2 + 14.3, 0.1)]
invalid_case("a slow first 10 percent of the fall is not a linear ramp",
             lambda prefix: write_logs(prefix, First(tau), slow_start, 50.0), ["--speeds", "", "--ramp-decels", "1"], "not linear")
invalid_case("a slow last 10 percent of the fall is not a linear ramp",
             lambda prefix: write_logs(prefix, First(tau), slow_end, 50.0), ["--speeds", "", "--ramp-decels", "1"], "not linear")


def sparse_cmd_build(rows):
    def build(prefix):
        step_log(first)(prefix)
        write_attitude(prefix, 24.0, lambda t: (bump_rate(t, ROLL_PEAK), bump_rate(t, THETA), 0.0),
                       lambda t: (bump(t, ROLL_PEAK), bump(t, THETA)), lambda t: (0.0, 0.0), cmd_dt=24.0 / max(rows - 1, 1))
        if rows == 0:
            with open(prefix + "vehicle_attitude_setpoint_0.csv", "w", newline="") as f:
                csv.writer(f).writerow(["timestamp", "q_d[0]", "q_d[1]", "q_d[2]", "q_d[3]"])
    return build


for n_rows in (8, 0):
    valid_case("attitude setpoint log with %d rows only nulls the commanded tilt" % n_rows, sparse_cmd_build(n_rows), ["--speeds", "5"],
               lambda res: (res["steps_down_to_zero"][0]["peak_pitch_deg"] is not None
                            and res["steps_down_to_zero"][0].get("peak_pitch_cmd_deg") is None, str(res["logs"])))


# ---- third review round (main merged, head 1b26aba)
# 1. a ramp logged every 0.2 s falls 1 m/s per row: the ramp is a sustained linear fall, not a step
sparse_inputs, sparse_ends, sparse_build = ramps_log(First(0.1), [(1, 5.0)], plateau_s=8.0, rest_s=8.0, sp_dt=0.2)
valid_case("ramp of 5 m/s^2 logged every 0.2 s (1 m/s per row) is a ramp", sparse_build,
           ["--speeds", "", "--ramp-decels", "5", "--ramp-window-s", "0.5", "--max-gap-s", "0.21"],
           lambda res: (res["all_ramps_found"] == 1 and near(res["ramps_down_to_zero"][0]["ramp_following_lag_s"], 0.1, 0.02)
                        and near(res["ramps_down_to_zero"][0]["decel_m_s2"], 5.0, 0.1), "found %s" % res["all_ramps_found"]))
# a single large drop is still a step, with or without one intermediate row
valid_case("a step down of 6 m/s is still a step, not a ramp",
           step_log(first, speed=6.0), ["--speeds", "6"],
           lambda res: (res["all_ramps_found"] == 0 and len(res["steps_down_to_zero"]) == 1, "ramps %s" % res["all_ramps_found"]))

# an isolated step to a lower plateau before the ramp is not part of the ramp
lower = [("step", 2.0, 8.0), ("step", 8.0, -3.0), ("ramp", 16.0, -1.0), ("ramp", 21.0, 1.0)]
valid_case("a ramp that starts from a plateau reached by a step down measures from that plateau",
           lambda prefix: write_logs(prefix, First(tau), lower, 40.0), ["--speeds", "", "--ramp-decels", "1"],
           lambda res: (res["all_ramps_found"] == 1 and near(res["ramps_down_to_zero"][0]["from_speed_m_s"], 5.0, 0.01)
                        and near(res["ramps_down_to_zero"][0]["ramp_following_lag_s"], tau, 0.03), ""))

# 2. the rate RMS is weighted by elapsed time: the same signal logged at 250 Hz and, in its second half, at 50 Hz
def rms_build(sparse_second_half):
    def build(prefix):
        step_log(first)(prefix)
        keep = (lambda t: t < 2.79 or abs(t / 0.02 - round(t / 0.02)) < 1e-6) if sparse_second_half else None
        write_attitude(prefix, 24.0, lambda t: (10.0 if 2.79 <= t < 4.0 else 0.0, 0.0, 0.0), lambda t: (0.0, 0.0),
                       lambda t: (0.0, 0.0), keep=keep)
    return build


rms_values = []
for sparse in (False, True):
    valid_case("roll-rate rms of a half-window signal, %s" % ("second half logged at 50 Hz" if sparse else "uniform 250 Hz"),
               rms_build(sparse), ["--speeds", "5"],
               lambda res: (rms_values.append(res["steps_up"][0]["roll_rate_rms_deg_s"]) or True,
                            "rms %s settle %s" % (res["steps_up"][0]["roll_rate_rms_deg_s"], res["steps_up"][0]["settle_s"])))
report("the rms does not depend on how densely each part of the window is logged",
       len(rms_values) == 2 and rms_values[0] is not None and rms_values[1] is not None
       and abs(rms_values[0] - rms_values[1]) < 0.03 * rms_values[0] and abs(rms_values[0] - 10.0 * math.sqrt(0.79 / 1.58)) < 0.4,
       "rms %s expected about %.2f" % (rms_values, 10.0 * math.sqrt(0.79 / 1.58)))


# 3. equality at a gap limit is accepted (timestamps are microseconds; seconds as floats differ by rounding)
def exact_gap_build(prefix):
    step_log(first)(prefix)
    write_attitude(prefix, 24.0, lambda t: (bump_rate(t, ROLL_PEAK), bump_rate(t, THETA), 0.0),
                   lambda t: (bump(t, ROLL_PEAK), bump(t, THETA)), lambda t: (0.0, 0.0), dt=0.05)


valid_case("rate and attitude rows exactly 50 ms apart meet the 0.050 s limit", exact_gap_build, ["--speeds", "5"],
           lambda res: (res["steps_up"][0]["problems"] == [] and res["steps_down_to_zero"][0]["problems"] == [], ""))
valid_case("position rows exactly 0.2 s apart meet the 0.2 s limit",
           step_log(first, pos_dt=0.2), ["--speeds", "5"],
           lambda res: (res["steps_down_to_zero"][0]["problems"] == [], str(res["steps_down_to_zero"][0]["problems"])))


# ---- fourth review round (head c0153f2)
# 1. a NaN velocity setpoint interrupts velocity control: a fall across it was not commanded continuously
nan_inputs, nan_ends, _ = ramps_log(First(0.4), [(1, 1.0)])
invalid_case("a ramp interrupted by NaN setpoints (position control) is not a ramp",
             lambda prefix: write_logs(prefix, First(0.4), nan_inputs, nan_ends[0] + 8.0, sp_nan=(10.0, 11.0)),
             ["--speeds", "", "--ramp-decels", "1"], "no complete response")
valid_case("NaN setpoints before the ramp, on its plateau, do not spoil the ramp",
           lambda prefix: write_logs(prefix, First(0.4), nan_inputs, nan_ends[0] + 8.0, sp_nan=(5.0, 6.0)),
           ["--speeds", "", "--ramp-decels", "1"],
           lambda res: (res["all_ramps_found"] == 1 and near(res["ramps_down_to_zero"][0]["ramp_following_lag_s"], 0.4, 0.02), ""))


# 2. every mean over samples is weighted by elapsed time: the yaw rate (0 then 10 deg/s halfway through the window)...
def yaw_build(sparse_second_half):
    def build(prefix):
        step_log(first)(prefix)
        keep = (lambda t: t < 2.79 or abs(t / 0.02 - round(t / 0.02)) < 1e-6) if sparse_second_half else None
        write_attitude(prefix, 24.0, lambda t: (0.0, 0.0, 10.0 if 2.79 <= t < 4.0 else 0.0), lambda t: (0.0, 0.0),
                       lambda t: (0.0, 0.0), keep=keep)
    return build


yaw_values = []
for sparse in (False, True):
    valid_case("mean yaw rate of a half-window signal, %s" % ("second half logged at 50 Hz" if sparse else "uniform 250 Hz"),
               yaw_build(sparse), ["--speeds", "5"],
               lambda res: (yaw_values.append(res["steps_up"][0]["yaw_rate_mean_deg_s"]) or True,
                            "yaw mean %s" % res["steps_up"][0]["yaw_rate_mean_deg_s"]))
report("the mean yaw rate does not depend on how densely each part of the window is logged",
       len(yaw_values) == 2 and all(v is not None and abs(v - 5.0) < 0.15 for v in yaw_values), "yaw means %s expected 5.0" % yaw_values)

# ... and the steady error of a ramp, when the position log is dense in the first half of the window and sparse in the second
slow_ramp, _, slow_ramp_end = ramp_block(2.0, 1, 3.0, 1.0, 8.0)
win = 1.4
lag_expected = (win - (math.exp(-(3.0 - win)) - math.exp(-3.0))) / win  # mean of 1 - exp(-t) over the last 1.4 s of the 3 s ramp
valid_case("ramp lag is a time-weighted mean when the position log is dense, then sparse, in the window",
           lambda prefix: write_logs(prefix, First(1.0), slow_ramp, slow_ramp_end + 14.0,
                                     pos_keep=lambda t: t <= slow_ramp_end - win / 2 or abs(t * 5 - round(t * 5)) < 1e-6),
           ["--speeds", "", "--ramp-decels", "1", "--ramp-window-s", str(win), "--ramp-drift-s", "0.5"],
           lambda res: (near(res["ramps_down_to_zero"][0]["ramp_following_lag_s"], lag_expected, 0.015),
                        "lag %s expected %.3f" % (res["ramps_down_to_zero"][0]["ramp_following_lag_s"], lag_expected)))


def tool_module():
    spec = importlib.util.spec_from_file_location("tool_under_test", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def nyquist_check():
    """Power fraction above 2 Hz of a 1 Hz sine plus a Nyquist-frequency alternation, against the two-sided Hann-windowed
    DFT sum computed here from the definition."""
    n, rate_hz, cut = 256, 25.0, 2.0
    t = [k / rate_hz for k in range(n)]
    x = [math.sin(2 * math.pi * 1.0 * tk) + (-1.0) ** k for k, tk in enumerate(t)]
    mean = sum(x) / n
    u = [(v - mean) * 0.5 * (1.0 - math.cos(2 * math.pi * k / (n - 1))) for k, v in enumerate(x)]
    num = den = 0.0
    for k in range(1, n):
        re = sum(u[m] * math.cos(2 * math.pi * k * m / n) for m in range(n))
        im = sum(u[m] * math.sin(2 * math.pi * k * m / n) for m in range(n))
        power = re * re + im * im
        freq = min(k, n - k) * rate_hz / n
        den += power
        if freq > cut:
            num += power
    got = tool_module().power_fraction_above(t, x, cut)
    return got, num / den


try:
    got, want = nyquist_check()
    report("power fraction weights the Nyquist bin once and the others twice", abs(got - want) < 1e-6, "got %.6f want %.6f" % (got, want))
except AttributeError as e:
    report("power fraction weights the Nyquist bin once and the others twice", False, "missing %s" % e)


def fly_refuses(extra):
    p = subprocess.run([sys.executable, TOOL, "fly", "--px4-bin", "/nonexistent/px4-cp_brake", *extra], capture_output=True, text=True)
    try:
        res = json.loads(p.stdout)
    except ValueError:
        res = {}
    return p.returncode == 3 and "error" in res and "ramp" in str(res["error"])


report("fly refuses a zero, negative or non-finite ramp deceleration before any flight command",
       all(fly_refuses(["--ramp-decels", v]) for v in ("0", "-1", "2,0", "nan")), "")

# ---- INVALID: the evidence rules (a response that violates them is no evidence, exit 2)
slow = First(3.0)
invalid_case("[old rule] step down that does not settle in its hold", step_log(slow, t_down=12.0, t_end=16.0), ["--speeds", "5"],
             "did not settle")
invalid_case("[old rule] position sample gap inside the step response",
             step_log(first, pos_keep=lambda t: not (12.2 < t < 13.0)), ["--speeds", "5"], "gap")
invalid_case("[old rule] setpoint log ends before the response does", step_log(first, sp_until=13.0), ["--speeds", "5"],
             "coverage")
invalid_case("[old rule] time goes backwards in the position log", step_log(first, swap_pos=(300, 301)), ["--speeds", "5"],
             "time goes backwards")
invalid_case("[old rule] no complete step at the wanted speed", step_log(first, speed=3.0), ["--speeds", "5"], ("no complete step", "no complete response"))
invalid_case("no step up at the wanted speed (the log starts after it)", step_log(first, pos_from=6.0, sp_from=6.0),
             ["--speeds", "5", "--require-steps-up"], "'step up'")
invalid_case("step up without a position sample before the command (position log starts after it)",
             step_log(first, pos_from=6.0), ["--speeds", "5", "--require-steps-up"], "no fresh position sample")
up_slow = [("step", 2.0, 5.0), ("step", 5.0, -5.0)]
invalid_case("step up that does not settle before the next command",
             lambda prefix: write_logs(prefix, First(1.0), up_slow, 20.0), ["--speeds", "5", "--require-steps-up"], "did not settle")
invalid_case("a requested ramp is absent", step_log(first), ["--speeds", "5", "--ramp-decels", "2"], "no complete response")

short_inputs, _, _ = ramp_block(2.0, 1, 3.0, 3.0, 8.0)
invalid_case("ramp shorter than twice the steady window",
             lambda prefix: write_logs(prefix, First(0.3), short_inputs, 24.0), ["--speeds", "", "--ramp-decels", "3"],
             "ramp shorter")
inputs_ns, ends_ns, build_ns = ramps_log(First(2.0), [(1, 2.0)], rest_s=16.0)
invalid_case("ramp error not steady (loop slower than the ramp)", build_ns, ["--speeds", "", "--ramp-decels", "2"], "not steady")
t_r = 2.0 + 8.0
kinked = [("step", 2.0, 8.0), ("ramp", t_r, -1.0), ("ramp", t_r + 2.0, -2.0), ("ramp", t_r + 4.0, 3.0)]
invalid_case("ramp setpoint that is not a straight line",
             lambda prefix: write_logs(prefix, First(tau), kinked, 40.0), ["--speeds", "", "--ramp-decels", "2"],
             "not linear")
invalid_case("angular velocity gap inside a response",
             lambda prefix: (step_log(first)(prefix),
                             write_attitude(prefix, 24.0, lambda t: (bump_rate(t, ROLL_PEAK), bump_rate(t, THETA), 0.0),
                                            lambda t: (bump(t, ROLL_PEAK), bump(t, THETA)), lambda t: (0.0, 0.0),
                                            keep=lambda t: not (12.3 < t < 12.6))),
             ["--speeds", "5"], "vehicle_angular_velocity log")
invalid_case("time goes backwards in the angular velocity log",
             lambda prefix: (step_log(first)(prefix),
                             write_attitude(prefix, 24.0, lambda t: (bump_rate(t, ROLL_PEAK), bump_rate(t, THETA), 0.0),
                                            lambda t: (bump(t, ROLL_PEAK), bump(t, THETA)), lambda t: (0.0, 0.0), back=True)),
             ["--speeds", "5"], "time goes backwards")
invalid_case("angular velocity and attitude logs too sparse for a spectrum",
             lambda prefix: (step_log(first)(prefix),
                             write_attitude(prefix, 24.0, lambda t: (bump_rate(t, ROLL_PEAK), bump_rate(t, THETA), 0.0),
                                            lambda t: (bump(t, ROLL_PEAK), bump(t, THETA)), lambda t: (0.0, 0.0), dt=0.2)),
             ["--speeds", "5"], "vehicle_angular_velocity log")

print("%d of %d cases passed" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
