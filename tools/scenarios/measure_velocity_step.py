#!/usr/bin/env python3
"""Measure the velocity loop of the simulated vehicle: steps, ramps, attitude chatter (bench tool).

usage:
  measure_velocity_step.py fly --px4-bin <path of px4-cp_brake> [--speeds 3,5,8] [--hold-s 8] [--altitude 12]
                               [--yaw-rate-deg-s 0] [--ramp-decels 1,2,3 --ramp-speed 8 --ramp-hold-s 5]
  measure_velocity_step.py analyze <csv-prefix> [--speeds 3,5,8] [--ramp-decels 1,2,3] [bench limits]

`fly` takes off, then in Offboard mode commands east velocity steps 0 -> +V -> 0 -> -V -> 0 for each V
(the sign alternates so the vehicle stays near the origin) and prints one JSON line. Offboard is used
because the limiter hook is not active there: the response is the vehicle's own, not the limiter's.
With --yaw-rate-deg-s the yaw rate is held at that value during every step cycle (the commanded velocity
stays in the local NED frame, so the yaw rotates the body under a constant ground-frame command).
With --ramp-decels each ramp goes 0 -> +-ramp-speed (a step, held --ramp-hold-s) and then the setpoint falls
to zero at the given deceleration (m/s^2); the sign alternates. A ramp excursion covers about
ramp-speed * ramp-hold-s + ramp-speed^2 / (2 * decel) m, so check the free space. Without these options the
flight is the plain step flight.

`analyze` reads the ulog2csv output (`vehicle_local_position_setpoint_0.csv`, `vehicle_local_position_0.csv`;
when present also `vehicle_angular_velocity_0.csv`, `vehicle_attitude_0.csv`, `vehicle_attitude_setpoint_0.csv`)
and prints for every response found:
  step down to zero: the equivalent lag (the extra distance travelled after the step divided by the speed
    before it; the lag that, put in front of an ideal instant stop, gives the same stopping distance, so it is
    the quantity the stopping model needs), the time to 63 % of the response and the settling time.
    CPB_T_LAT is the largest equivalent lag of the steps down rounded up to 0.05 s.
  every step (up and down): rise time 10 to 90 %, overshoot percent of the step, settling time to 2 % of the
    step, ringing count (crossings of the +-2 % band by the error, after the first crossing). For a step to
    zero the overshoot is the reversal below zero.
  ramp (setpoint falls linearly from V to zero; the line is fitted to the rows between 10 and 90 percent of V, so a
    ramp that starts or ends between two logged rows, as in a 10 Hz log, is measured): the following lag in seconds (mean error over the last
    --ramp-window-s of the ramp divided by the ramp rate), the final overshoot below zero (m/s and percent
    of V), the settling time to 2 % after the ramp end and the ringing count after the ramp end.
  attitude, per response, from the start of the command to the settled time: roll-rate and pitch-rate RMS,
    the fraction of the (mean-removed, Hann-windowed) power above --chatter-hz, the mean yaw rate, the peak
    roll and pitch from the attitude quaternion and the peak commanded roll and pitch from q_d.
INVALID (exit 2) when a selected response (steps down at --speeds; steps up with --require-steps-up; ramps at
--ramp-decels) has not settled (sustained, before the next command), lacks position
samples before the command, lacks setpoint coverage, has a sample gap above --max-gap-s, when time goes
backwards in any log, when a ramp setpoint is not linear, too short or has no steady error, or when a present
vehicle_attitude or vehicle_angular_velocity log has too few samples or a gap above --max-rate-gap-s inside a
response; every problem names its topic. The attitude setpoint is logged more slowly: a gap above --max-cmd-gap-s
only makes the commanded peaks null and adds a note. A ramp must last twice --ramp-window-s: for ramp-speed 6 and
decel 3 (2 s) use --ramp-window-s 0.8. Every limit is a
bench argument; none is an approved platform value. Run with python3 without -I (pymavlink) for `fly`.
"""
import argparse
import bisect
import csv
import json
import math
import os
import subprocess
import sys
import time

STEP_MIN = 1.0       # m/s, a setpoint change below this is not a step
SETTLE_BAND = 0.1    # m/s
SETTLE_STAY = 0.2    # m/s, after the onset the speed stays below this (the analyzer's rest rule: a small rollback is not motion)
SETTLE_HOLD_S = 1.0  # s the stop must last to count as stopped
MAX_RESPONSE_S = 15.0
ROUND_UP_S = 0.05    # CPB_T_LAT is rounded up to this resolution (owner, 2026-11-20: "rounded up")
BAND_FRACTION = 0.02  # settling and ringing band: 2 % of the step (the issue's definition)
RAMP_MIN_ROWS = 3    # setpoint rows on the falling line before a run counts as a ramp
MIN_RATE_SAMPLES = 16  # samples a rate window needs before a spectrum means anything
MASK_VELOCITY = 0b110111000111       # SET_POSITION_TARGET_LOCAL_NED: velocity only
MASK_VELOCITY_YAW_RATE = 0b010111000111  # velocity and yaw rate (bit 11 clear)


def fly(a):
    # a ramp that never reaches zero (deceleration 0, negative or not a number) would hold or raise the speed for good
    if a.ramp_decels and (not all(math.isfinite(d) and d > 0.0 for d in a.ramp_decels)
                          or not (math.isfinite(a.ramp_speed) and a.ramp_speed > 0.0)):
        print(json.dumps({"error": "every --ramp-decels value and --ramp-speed must be finite and above zero",
                          "steps": [], "ramps": []}))
        return 3
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

    result = {"error": None, "steps": [], "ramps": [], "yaw_rate_deg_s": a.yaw_rate_deg_s}
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

    def send(vy, yaw_rate_deg_s=0.0):
        mask = MASK_VELOCITY if yaw_rate_deg_s == 0.0 else MASK_VELOCITY_YAW_RATE
        link.mav.set_position_target_local_ned_send(
            0, link.target_system, link.target_component, mav.MAV_FRAME_LOCAL_NED, mask,
            0, 0, 0, 0, vy, 0, 0, 0, 0, 0, math.radians(yaw_rate_deg_s))

    def hold(vy, seconds, yaw_rate_deg_s=0.0):
        end = time.time() + seconds
        while time.time() < end:
            send(vy, yaw_rate_deg_s)
            time.sleep(0.05)

    hold(0.0, 2.0)  # PX4 needs a setpoint stream before it accepts Offboard
    result["offboard"] = run("px4-commander", "mode", "offboard")
    hold(0.0, 2.0)
    yaw = a.yaw_rate_deg_s  # held through every step cycle, also through the zero holds
    for v in a.speeds:
        for target in (v, 0.0, -v, 0.0):
            send(target, yaw)
            result["steps"].append({"t_unix": round(time.time(), 2), "to_m_s": target})
            hold(target, a.hold_s, yaw)
    # ramps: a step to the ramp speed (held), then the setpoint falls linearly to zero; the sign alternates
    for n, decel in enumerate(a.ramp_decels):
        v = (1.0 if n % 2 == 0 else -1.0) * a.ramp_speed
        send(v, yaw)
        result["steps"].append({"t_unix": round(time.time(), 2), "to_m_s": v})
        hold(v, a.ramp_hold_s, yaw)
        t_start = time.time()
        result["ramps"].append({"t_unix": round(t_start, 2), "from_m_s": v, "decel_m_s2": decel})
        while time.time() < t_start + a.ramp_speed / decel + 1.0:  # the bound is a guard; the loop ends at zero
            vy = math.copysign(max(0.0, a.ramp_speed - decel * (time.time() - t_start)), v)
            send(vy, yaw)
            if vy == 0.0:
                break
            time.sleep(0.05)
        hold(0.0, a.hold_s, yaw)
    run("px4-commander", "mode", "auto:loiter")
    print(json.dumps(result))
    return 0


def read(path, columns, min_rows=10):
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
    if len(kept) < min_rows:
        raise ValueError("fewer than %d finite rows in %s" % (min_rows, path))
    return {c: [row[i] for row in kept] for i, c in enumerate(columns)}


def read_setpoint(path):
    """(time, vy) of every setpoint row; vy is NaN where PX4 does not control velocity (position control)."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    t, vy = [], []
    for r in rows:
        try:
            ti = float(r["timestamp"])
        except (KeyError, ValueError, TypeError):
            continue
        try:
            v = float(r["vy"])
        except (KeyError, ValueError, TypeError):
            v = float("nan")
        if math.isfinite(ti):
            t.append(ti)
            vy.append(v if math.isfinite(v) else float("nan"))
    if len(t) < 10:
        raise ValueError("fewer than 10 rows in " + path)
    return {"timestamp": t, "vy": vy}


def load_optional(path, columns, soft=False):
    """None when the log does not exist; the columns plus "t" (s) when it does; ValueError when it is unusable.
    soft: a log with fewer than 2 usable rows counts as absent (the topic only feeds optional values)."""
    if not os.path.exists(path):
        return None
    try:
        d = read(path, columns, 2 if soft else 10)
    except ValueError:
        if soft:
            return None
        raise
    d["t"] = [v * 1e-6 for v in d["timestamp"]]
    if any(b < x for x, b in zip(d["t"], d["t"][1:])):
        raise ValueError("time goes backwards in " + os.path.basename(path))
    return d


def roll_pitch_deg(w, x, y, z):
    """Roll and pitch [deg] of a body-to-NED quaternion (the PX4 convention), yaw ignored."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        return None
    w, x, y, z = w / n, x / n, y / n, z / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def peaks_deg(log, prefix):
    """Per sample (roll, pitch) in degrees of the quaternion columns `<prefix>[0..3]`; rows of zero norm are dropped."""
    t, rp = [], []
    for n in range(len(log["t"])):
        v = roll_pitch_deg(*[log["%s[%d]" % (prefix, k)][n] for k in range(4)])
        if v is not None:
            t.append(log["t"][n])
            rp.append(v)
    return {"t": t, "roll": [v[0] for v in rp], "pitch": [v[1] for v in rp]}


def fft(x):
    """Iterative radix-2 FFT of a list of complex numbers whose length is a power of two."""
    n = len(x)
    x = list(x)
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j ^= bit
        if i < j:
            x[i], x[j] = x[j], x[i]
    size = 2
    while size <= n:
        wl = complex(math.cos(-2.0 * math.pi / size), math.sin(-2.0 * math.pi / size))
        half = size // 2
        for s in range(0, n, size):
            w = 1.0 + 0.0j
            for k in range(half):
                u, v = x[s + k], x[s + k + half] * w
                x[s + k], x[s + k + half] = u + v, u - v
                w *= wl
        size <<= 1
    return x


def power_fraction_above(t, x, cut_hz):
    """Fraction of the power above cut_hz: resampled to a uniform grid, mean removed, Hann window, FFT, DC excluded.
    0.0 when the mean-removed signal has no power; None when the window is too short or its Nyquist frequency does not exceed cut_hz."""
    n = len(t)
    if n < MIN_RATE_SAMPLES or t[-1] <= t[0]:
        return None
    dt = (t[-1] - t[0]) / (n - 1)
    if 0.5 / dt <= cut_hz:
        return None
    u, k = [], 0
    for m in range(n):
        tm = t[0] + m * dt
        while k < n - 2 and t[k + 1] < tm:
            k += 1
        span = t[k + 1] - t[k]
        f = 0.0 if span <= 0.0 else min(1.0, max(0.0, (tm - t[k]) / span))
        u.append(x[k] + f * (x[k + 1] - x[k]))
    mean = sum(u) / n
    u = [(v - mean) * 0.5 * (1.0 - math.cos(2.0 * math.pi * m / (n - 1))) for m, v in enumerate(u)]
    size = 1
    while size < n:
        size <<= 1
    spec = fft([complex(v, 0.0) for v in u] + [0j] * (size - n))
    # a real signal: every bin below Nyquist has a mirror image at the negative frequency, the Nyquist bin has none
    half = size // 2
    power = [abs(spec[k]) ** 2 * (1.0 if k == half else 2.0) for k in range(1, half + 1)]
    total = sum(power)
    if total <= 0.0:
        return 0.0  # no power after the mean is removed (a motion about no axis): nothing is above the cut
    return sum(p for k, p in enumerate(power, start=1) if k / (size * dt) > cut_hz) / total


def window_covered(times, lo, hi, t_a, t_b, max_gap_s, min_samples):
    """The samples times[lo:hi] cover [t_a, t_b]: enough of them and no gap (edges included) above max_gap_s."""
    st = times[lo:hi]
    if len(st) < min_samples or not st:
        return False
    return max([st[0] - t_a, t_b - st[-1]] + [y - x for x, y in zip(st, st[1:])]) <= max_gap_s


def cross_time(ts, ps, level):
    """First time the normalised progress ps reaches level, interpolated between samples; None if never."""
    for n in range(len(ps)):
        if ps[n] >= level:
            if n == 0 or ps[n] == ps[n - 1]:
                return ts[n]
            return ts[n - 1] + (level - ps[n - 1]) / (ps[n] - ps[n - 1]) * (ts[n] - ts[n - 1])
    return None


def crossings(err, band):
    """Number of sign changes of err, ignoring samples inside +-band (the first change is the first crossing)."""
    state, flips = 0, 0
    for v in err:
        s = 1 if v > band else (-1 if v < -band else 0)
        if s == 0:
            continue
        if state and s != state:
            flips += 1
        state = s
    return flips


def settle_band_time(ts, err, band, t_ref):
    """Time from t_ref after which err stays inside +-band to the end of the samples; None if it never does."""
    last_out = max((n for n, v in enumerate(err) if abs(v) > band), default=-1)
    if last_out == len(err) - 1:
        return None
    return 0.0 if last_out < 0 else ts[last_out + 1] - t_ref


def r3(v):
    return None if v is None else round(v, 3)


def analyze(a):
    try:
        sp = read_setpoint(a.prefix + "vehicle_local_position_setpoint_0.csv")
        ps = read(a.prefix + "vehicle_local_position_0.csv", ["timestamp", "vy"])
        rate = load_optional(a.prefix + "vehicle_angular_velocity_0.csv", ["timestamp", "xyz[0]", "xyz[1]", "xyz[2]"])
        att = load_optional(a.prefix + "vehicle_attitude_0.csv", ["timestamp"] + ["q[%d]" % k for k in range(4)])
        cmd = load_optional(a.prefix + "vehicle_attitude_setpoint_0.csv", ["timestamp"] + ["q_d[%d]" % k for k in range(4)], soft=True)
    except (OSError, ValueError) as e:
        print(json.dumps({"error": "INVALID: %s" % e}))
        return 2
    ts = [v * 1e-6 for v in sp["timestamp"]]
    tp = [v * 1e-6 for v in ps["timestamp"]]
    if any(b < x for x, b in zip(ts, ts[1:])) or any(b < x for x, b in zip(tp, tp[1:])):
        print(json.dumps({"error": "INVALID: time goes backwards in the setpoint or position log"}))
        return 2
    spv, pv = sp["vy"], ps["vy"]
    att_tilt = peaks_deg(att, "q") if att is not None else None
    cmd_tilt = peaks_deg(cmd, "q_d") if cmd is not None else None

    def pre_sample(t_pre):
        """(covered, j): the last position sample at or before t_pre, and whether it is fresh enough to be evidence."""
        have = tp[0] <= t_pre + 1e-9
        j = max(k for k in range(len(tp)) if tp[k] <= t_pre + 1e-9) if have else 0
        return have and t_pre - tp[j] <= a.max_gap_s, j

    def attitude(t_a, t_b, problems, notes):
        """Chatter and tilt of the window [t_a, t_b] from the logs that exist; a window they do not cover is a problem."""
        out = {}
        if rate is not None:
            lo, hi = bisect.bisect_left(rate["t"], t_a), bisect.bisect_right(rate["t"], t_b)
            if not (t_b > t_a and window_covered(rate["t"], lo, hi, t_a, t_b, a.max_rate_gap_s, MIN_RATE_SAMPLES)):
                problems.append("vehicle_angular_velocity log: too few samples or a gap above %.3f s in the response" % a.max_rate_gap_s)
            else:
                tt = rate["t"][lo:hi]
                for name, col in (("roll", "xyz[0]"), ("pitch", "xyz[1]")):
                    x = rate[col][lo:hi]
                    out[name + "_rate_rms_deg_s"] = r3(math.degrees(math.sqrt(sum(v * v for v in x) / len(x))))
                    frac = power_fraction_above(tt, x, a.chatter_hz)
                    out[name + "_rate_power_above_%ghz_fraction" % a.chatter_hz] = None if frac is None else round(frac, 4)
                    if frac is None:
                        problems.append("vehicle_angular_velocity log: the %s-rate window has no spectrum above %.2f Hz (sample rate too low)" % (name, a.chatter_hz))
                out["yaw_rate_mean_deg_s"] = r3(math.degrees(sum(rate["xyz[2]"][lo:hi]) / (hi - lo)))
        # the peak tilt comes from vehicle_attitude (a hole there is a problem); the commanded tilt from the attitude
        # setpoint, which is logged more slowly and has its own limit: a hole only makes the commanded tilt unavailable
        for log, label, topic, limit, hard in ((att_tilt, "peak_%s_deg", "vehicle_attitude", a.max_rate_gap_s, True),
                                               (cmd_tilt, "peak_%s_cmd_deg", "vehicle_attitude_setpoint", a.max_cmd_gap_s, False)):
            if log is None:
                continue
            lo, hi = bisect.bisect_left(log["t"], t_a), bisect.bisect_right(log["t"], t_b)
            if not (t_b > t_a and window_covered(log["t"], lo, hi, t_a, t_b, limit, 2)):
                msg = "%s log: too few samples or a gap above %.3f s in the response" % (topic, limit)
                if hard:
                    problems.append(msg)
                else:
                    notes.append("%s, so %s and %s are null" % (msg, label % "roll", label % "pitch"))
                    for name in ("roll", "pitch"):
                        out[label % name] = None
            else:
                for name in ("roll", "pitch"):
                    out[label % name] = r3(max(abs(v) for v in log[name][lo:hi]))
        return out

    def respond(kind, i, i_prev, sign, target, y0, covered, j, t0, t_next, ramp=None):
        """One response of the speed along `sign` to a command at t0: the evidence rules, then the metrics.
        target is the speed commanded after the command (sign-adjusted), y0 the speed before it."""
        t_ref = ramp["t_end"] if ramp else t0
        wend = min(t_next, t_ref + MAX_RESPONSE_S, tp[-1])
        idx = [k for k in range(j + 1, len(tp)) if t0 <= tp[k] <= wend]
        # the command stream must cover the whole response: a setpoint log that ends early does not show
        # that the command (and velocity control) held while the vehicle responded
        sp_times = [ts[m] for m in range(i_prev, len(ts)) if ts[m] <= wend] or [t0]  # from the last row before the command
        setpoint_gap = max([y - x for x, y in zip(sp_times, sp_times[1:])] + [wend - sp_times[-1]])
        ys = [sign * pv[k] for k in idx]
        worst_gap = max([tp[k] - tp[k - 1] for k in idx], default=0.0)  # the pair that spans the command counts
        # settled: the onset is a sample inside SETTLE_BAND of the target, the speed stays within SETTLE_STAY of it
        # from there to the end of the window, and that stay lasts at least SETTLE_HOLD_S; a dip into the band that
        # is followed by movement is no stop (the same rule as the rest of analyze_stop.py)
        violations = [n for n, y in enumerate(ys) if abs(y - target) >= SETTLE_STAY]
        first = violations[-1] + 1 if violations else 0
        if ramp:  # a ramp is over only when its setpoint is: a fast loop is inside the band before that
            first = max(first, next((n for n, k in enumerate(idx) if tp[k] >= ramp["t_end"]), len(idx)))
        onset = next((n for n in range(first, len(ys)) if abs(ys[n] - target) < SETTLE_BAND), None)
        settled_t = None
        if onset is not None and tp[idx[-1]] - tp[idx[onset]] >= SETTLE_HOLD_S:
            settled_t = tp[idx[onset]]
        end = onset if settled_t is not None else len(idx) - 1
        # the speed interpolated at the command time starts every series: giving a sample's speed to the whole
        # interval before it would understate the distance of a decreasing speed
        k0 = idx[0] if idx else None
        if k0 is not None and k0 > 0 and tp[k0] > tp[k0 - 1]:
            f = (t0 - tp[k0 - 1]) / (tp[k0] - tp[k0 - 1])
            y_t0 = sign * (pv[k0 - 1] + f * (pv[k0] - pv[k0 - 1]))
        else:
            y_t0 = y0
        series_t = [t0] + [tp[k] for k in idx]
        series_y = [y_t0] + ys
        problems = []
        if settled_t is None:
            problems.append("did not settle (sustained, before the next command)")
        if not covered:
            problems.append("vehicle_local_position log: no fresh position sample before the command")
        if worst_gap > a.max_gap_s:
            problems.append("vehicle_local_position log: sample gap above %.2f s" % a.max_gap_s)
        if setpoint_gap > a.max_gap_s:
            problems.append("vehicle_local_position_setpoint log: coverage gap above %.2f s" % a.max_gap_s)
        rec = {"t_s": round(t0, 2), "max_sample_gap_s": r3(worst_gap), "position_covered_before_step": covered,
               "max_setpoint_gap_s": r3(setpoint_gap)}
        if kind == "ramp_down":
            v_ramp, rate_ms2 = ramp["V"], ramp["rate"]
            line = lambda t: ramp["line0"] - rate_ms2 * (t - t0)
            dur = ramp["t_end"] - t0
            rec.update({"decel_m_s2": r3(rate_ms2), "from_speed_m_s": r3(v_ramp), "duration_s": r3(dur),
                        "setpoint_nonlinearity_m_s": r3(ramp["nonlinearity"])})
            win = [(series_t[n], series_y[n] - line(series_t[n])) for n in range(1, len(series_t))
                   if ramp["t_end"] - a.ramp_window_s <= series_t[n] <= ramp["t_end"]]
            mid = ramp["t_end"] - 0.5 * a.ramp_window_s
            h1, h2 = [e for t, e in win if t < mid], [e for t, e in win if t >= mid]
            if dur < 2.0 * a.ramp_window_s:
                problems.append("ramp shorter than twice --ramp-window-s (%.2f s)" % a.ramp_window_s)
            if len(h1) < 2 or len(h2) < 2:
                problems.append("fewer than 2 position samples in each half of the ramp window")
            else:
                steady = sum(e for t, e in win) / len(win)
                drift_s = abs(sum(h2) / len(h2) - sum(h1) / len(h1)) / rate_ms2
                rec.update({"steady_error_m_s": r3(steady), "ramp_following_lag_s": r3(steady / rate_ms2),
                            "steady_drift_s": r3(drift_s), "_lag_raw": steady / rate_ms2})
                if drift_s > a.ramp_drift_s:
                    problems.append("the ramp error is not steady (drift %.3f s above %.3f s)" % (drift_s, a.ramp_drift_s))
            if ramp["nonlinearity"] > a.ramp_linear_m_s:
                problems.append("the ramp setpoint is not linear (%.3f m/s above %.3f m/s)" % (ramp["nonlinearity"], a.ramp_linear_m_s))
            after = [(t, y) for t, y in zip(series_t, series_y) if t >= ramp["t_end"]]
            at, ay = [t for t, y in after], [y for t, y in after]
            band = BAND_FRACTION * v_ramp
            under = max([0.0] + [-y for y in ay])
            rec.update({"final_overshoot_m_s": r3(under), "final_overshoot_percent": r3(100.0 * under / v_ramp),
                        "ringing_count": max(0, crossings(ay, band) - 1),
                        "settle_2pct_after_ramp_s": r3(settle_band_time(at, ay, band, ramp["t_end"])) if ay else None,
                        "settle_s_after_ramp": None if settled_t is None else r3(settled_t - ramp["t_end"])})
        else:
            step = target - y0
            rec["speed_before_m_s"] = round(abs(y0), 2)
            rec["settle_s"] = None if settled_t is None else r3(settled_t - t0)
            if abs(step) > 1e-9 and len(series_t) > 1:
                prog = [(y - y0) / step for y in series_y]
                t10, t90 = cross_time(series_t, prog, 0.1), cross_time(series_t, prog, 0.9)
                err = [p - 1.0 for p in prog]
                rec.update({"step_m_s": r3(abs(step)),
                            "rise_10_90_s": None if t10 is None or t90 is None else r3(t90 - t10),
                            "overshoot_percent": r3(100.0 * max(0.0, max(prog) - 1.0)),
                            "settle_2pct_s": r3(settle_band_time(series_t, err, BAND_FRACTION, t0)),
                            "ringing_count": max(0, crossings(err, BAND_FRACTION) - 1)})
            else:
                problems.append("no step to measure (the speed before equals the target)")
            if kind == "step_down":
                # trapezoid rule over the response up to the onset of the stop
                area, t63, prev_t, prev_y = 0.0, None, t0, y_t0
                for n in range(0, end + 1):
                    k = idx[n]
                    area += 0.5 * (max(0.0, prev_y) + max(0.0, ys[n])) * (tp[k] - prev_t)
                    prev_t, prev_y = tp[k], ys[n]
                    if t63 is None and ys[n] <= 0.368 * abs(y0):
                        t63 = tp[k] - t0
                rec.update({"equivalent_lag_s": round(area / abs(y0), 3), "_lag_raw": area / abs(y0),
                            "t63_s": r3(t63)})
            else:
                rec["target_speed_m_s"] = r3(abs(target))
        notes = []
        for key, what in (("rise_10_90_s", "the speed did not reach 90 percent of the step"),
                          ("settle_2pct_s", "the speed was outside the 2 percent band when the response window ended (hold, next command or log)"),
                          ("settle_2pct_after_ramp_s", "the speed was outside the 2 percent band when the response window ended (hold, next command or log)")):
            if key in rec and rec[key] is None:
                notes.append("%s is null: %s" % (key, what))
        rec.update(attitude(t0, settled_t if settled_t is not None else wend, problems, notes))
        rec["notes"] = notes
        rec["problems"] = problems
        return rec

    downs, ups, ramps = [], [], []
    for i in range(1, len(ts)):
        p, c = spv[i - 1], spv[i]
        if abs(p) >= STEP_MIN and abs(c) < SETTLE_BAND:  # a step to zero: beyond STEP_MIN before, within the band now
            # the speed before the step comes from a position sample at or just before the command; a log that
            # starts after the step, or whose last earlier sample is stale, gives no pre-step evidence
            covered, j = pre_sample(ts[i - 1])
            v0 = pv[j] if covered else p
            if abs(v0) < STEP_MIN:
                continue
            # the response is bounded by the next velocity command or a switch away from velocity control (or MAX_RESPONSE_S): a stop that happens
            # during another commanded manoeuvre is not the response to this step
            # a NaN setpoint (position control took over) is a boundary too, not a continuing zero command
            t_next = next((ts[m] for m in range(i + 1, len(ts)) if not abs(spv[m]) < SETTLE_BAND), float("inf"))
            sign = 1.0 if v0 > 0 else -1.0
            downs.append(respond("step_down", i, i - 1, sign, 0.0, sign * v0, covered, j, ts[i], t_next))
        elif abs(p) < SETTLE_BAND and abs(c) >= STEP_MIN:  # a step up from rest
            covered, j = pre_sample(ts[i - 1])
            sign = 1.0 if c > 0 else -1.0
            v_pre = pv[j] if covered else p
            # the window ends when the setpoint leaves the commanded value (a step, a ramp, position control)
            t_next = next((ts[m] for m in range(i + 1, len(ts)) if not abs(spv[m] - c) < SETTLE_BAND), float("inf"))
            ups.append(respond("step_up", i, i - 1, sign, abs(c), sign * v_pre, covered, j, ts[i], t_next))
        elif abs(p) >= SETTLE_BAND and abs(c) < SETTLE_BAND:  # the end of a falling setpoint (a step down was handled above)
            sign = 1.0 if p > 0 else -1.0
            # the run of non-increasing setpoints that ends here starts at the plateau (repeated values, as in a
            # sample-and-hold log, belong to it); the ramp is measured on its body, between 10 and 90 percent of the
            # plateau value, because the rows next to the plateau and to zero are partial: a ramp starts and ends
            # between two logged rows, and a row is only a sample of the command
            m0 = i - 1
            while m0 > 0 and -1e-6 <= sign * (spv[m0 - 1] - spv[m0]) < STEP_MIN:
                m0 -= 1
            v_top = sign * spv[m0]
            body = [k for k in range(m0, i) if 0.1 * v_top <= sign * spv[k] <= 0.9 * v_top]
            if v_top < STEP_MIN or len(body) < RAMP_MIN_ROWS:
                continue
            # the falling line is the least-squares line through the body rows: exact for a linear setpoint, through the
            # middle of the stairs for a sample-and-hold one; it is extended to the plateau value and to zero
            tk, rk = [ts[k] for k in body], [sign * spv[k] for k in body]
            t_mean, r_mean = sum(tk) / len(tk), sum(rk) / len(rk)
            denom = sum((t - t_mean) ** 2 for t in tk)
            if denom <= 0.0:
                continue
            slope = -sum((t - t_mean) * (r - r_mean) for t, r in zip(tk, rk)) / denom  # the fall rate, m/s^2
            if slope <= 0.0:
                continue
            t_start = t_mean - (v_top - r_mean) / slope
            # every row of the fall is checked against the fitted line clamped to the plateau value and to zero, not only
            # the body: a change of slope in the first or last 10 percent is not a linear ramp
            nonlin = max(abs(sign * spv[k] - min(v_top, max(0.0, r_mean - slope * (ts[k] - t_mean)))) for k in range(m0, i + 1))
            i_prev = max([k for k in range(m0, i) if ts[k] <= t_start + 1e-9] or [m0])
            covered, j = pre_sample(ts[i_prev])
            v_pre = pv[j] if covered else spv[i_prev]
            t_next = next((ts[k] for k in range(i, len(ts)) if not abs(spv[k]) < SETTLE_BAND), float("inf"))
            ramp = {"V": v_top, "rate": slope, "line0": v_top, "t_end": t_start + v_top / slope, "nonlinearity": nonlin}
            ramps.append(respond("ramp_down", i_prev, i_prev, sign, 0.0, sign * v_pre, covered, j, t_start, t_next, ramp))
    wanted = [round(v, 1) for v in a.speeds]
    kept_down = [s for s in downs if any(abs(s["speed_before_m_s"] - w) < 0.5 for w in wanted)]
    kept_up = [s for s in ups if any(abs(s["target_speed_m_s"] - w) < 0.5 for w in wanted)]
    if a.ramp_decels:
        kept_ramps = [s for s in ramps if any(abs(s["decel_m_s2"] - w) < 0.5 for w in a.ramp_decels)]
    else:
        kept_ramps = list(ramps)
    out = {"steps_down_to_zero": kept_down, "steps_up": kept_up, "ramps_down_to_zero": kept_ramps,
           "all_steps_found": len(downs), "all_steps_up_found": len(ups), "all_ramps_found": len(ramps),
           "logs": {"angular_velocity": rate is not None, "attitude": att is not None, "attitude_setpoint": cmd is not None}}
    # a response that did not settle (the log or the 15 s window ended first) has a partial stopping distance:
    # it would understate the lag, so it is no evidence and the measurement is rejected
    # ramps are required only where --ramp-decels names them: a falling setpoint the flight did not plan (position
    # control, a smoothed command) is listed with its problems but does not reject a step measurement
    # steps up are required only with --require-steps-up: the rules of the tool before the step-up metrics did not need them
    incomplete = [s for s in kept_down + (kept_up if a.require_steps_up else []) + (kept_ramps if a.ramp_decels else [])
                  if s["problems"]]
    missing = [("step down", w) for w in wanted if not any(abs(s["speed_before_m_s"] - w) < 0.5 and not s["problems"] for s in kept_down)]
    if a.require_steps_up:
        missing += [("step up", w) for w in wanted
                    if not any(abs(s["target_speed_m_s"] - w) < 0.5 and not s["problems"] for s in kept_up)]
    # each requested ramp needs its own complete recorded ramp (one ramp cannot satisfy two requests): a bipartite
    # matching that reconsiders earlier choices, since the tolerance ranges of close requests overlap
    ok_ramps = [ri for ri, s in enumerate(kept_ramps) if not s["problems"]]
    owner = {}  # recorded ramp -> request index

    def assign(wi, seen):
        for ri in ok_ramps:
            if ri not in seen and abs(kept_ramps[ri]["decel_m_s2"] - a.ramp_decels[wi]) < 0.5:
                seen.add(ri)
                if ri not in owner or assign(owner[ri], seen):
                    owner[ri] = wi
                    return True
        return False

    unmatched = [wi for wi in range(len(a.ramp_decels)) if not assign(wi, set())]
    missing += [("ramp", a.ramp_decels[wi]) for wi in unmatched]
    worst_raw = max((s["_lag_raw"] for s in kept_down if "_lag_raw" in s), default=None)
    # the summary uses complete ramps only; the records of the others stay listed with their problems
    ramp_lags = [s["_lag_raw"] for s in kept_ramps if "_lag_raw" in s and not s["problems"]]
    for s in kept_down + kept_up + kept_ramps:
        s.pop("_lag_raw", None)
    if incomplete or missing:
        reasons = sorted({q for s in incomplete for q in s["problems"]})
        out["error"] = ("INVALID: %d response(s) are incomplete (%s); no complete response for %s"
                        % (len(incomplete), "; ".join(reasons) if reasons else "none", missing))
        print(json.dumps(out, indent=2))
        return 2
    # the upward rounding works on the unrounded lag; only the displayed figures are rounded
    if worst_raw is not None:
        out["worst_equivalent_lag_s"] = round(worst_raw, 3)
        out["t_lat_rounded_up_s"] = round(math.ceil(worst_raw / ROUND_UP_S - 1e-9) * ROUND_UP_S, 2)
    if ramp_lags:
        out["worst_ramp_following_lag_s"] = round(max(ramp_lags), 3)
    print(json.dumps(out, indent=2))
    return 0


def float_list(s):
    return [float(v) for v in s.split(",") if v]


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fly")
    f.add_argument("--connect", default="udpin:0.0.0.0:14550")
    f.add_argument("--px4-bin", required=True)
    f.add_argument("--speeds", type=float_list, default=[3.0, 5.0, 8.0])
    f.add_argument("--hold-s", type=float, default=8.0)
    f.add_argument("--altitude", type=float, default=12.0)
    f.add_argument("--hover-s", type=float, default=15.0)
    f.add_argument("--yaw-rate-deg-s", type=float, default=0.0,
                   help="yaw rate [deg/s] held through every step cycle and ramp; 0 keeps the yaw (default)")
    f.add_argument("--ramp-decels", type=float_list, default=[],
                   help="setpoint decelerations [m/s^2] of the ramps from --ramp-speed to zero, e.g. 1,2,3; none by default")
    f.add_argument("--ramp-speed", type=float, default=8.0, help="speed [m/s] a ramp starts from")
    f.add_argument("--ramp-hold-s", type=float, default=5.0, help="time [s] held at the ramp speed before the ramp")
    n = sub.add_parser("analyze")
    n.add_argument("prefix")
    n.add_argument("--max-gap-s", type=float, default=0.2,
                   help="bench value [s]: largest gap between position samples inside a response")
    n.add_argument("--speeds", type=float_list, default=[3.0, 5.0, 8.0],
                   help="speeds [m/s] of the steps that must be present and complete; empty (\"\") requires none")
    n.add_argument("--require-steps-up", action="store_true",
                   help="every step up at --speeds must be present and complete (steps down always must)")
    n.add_argument("--ramp-decels", type=float_list, default=[],
                   help="decelerations [m/s^2] of the ramps that must be present and complete; default requires none")
    n.add_argument("--ramp-window-s", type=float, default=1.0,
                   help="bench value [s]: the last part of a ramp over which the steady error is taken; a ramp must last twice this")
    n.add_argument("--ramp-drift-s", type=float, default=ROUND_UP_S,
                   help="bench value [s]: largest change of the ramp lag between the two halves of the window")
    n.add_argument("--ramp-linear-m-s", type=float, default=SETTLE_BAND,
                   help="bench value [m/s]: largest deviation of the ramp setpoint from a straight line")
    n.add_argument("--max-rate-gap-s", type=float, default=0.05,
                   help="bench value [s]: largest gap between angular velocity or attitude samples inside a response")
    n.add_argument("--max-cmd-gap-s", type=float, default=0.1,
                   help="bench value [s]: largest gap between attitude setpoint samples inside a response; a larger gap only makes the "
                        "commanded tilt null (that topic is logged at about 20 Hz)")
    n.add_argument("--chatter-hz", type=float, default=2.0,
                   help="frequency [Hz] above which the rate power counts as chatter (the issue's definition)")
    a = ap.parse_args()
    return fly(a) if a.cmd == "fly" else analyze(a)


if __name__ == "__main__":
    sys.exit(main())
