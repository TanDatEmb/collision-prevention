#!/usr/bin/env python3
"""Feel metrics of a manual Position flight with cp_brake (bench tool, not a verdict).

usage: make flight-check [LOG=path.ulg]   (default: the latest manual-sim log)
       python3 tools/manual_sim/flight_check.py log.ulg [log.ulg ...]

Per minute of flight with cp_brake on: stick held with no motion (often a push into an obstacle at the margin),
tilt above 20 deg, tilt-command steps above 10 deg, body-rate jitter while the hook switches on and off, the
feed-forward the hook clears, and forward sectors that fall from free to 0 in one step (coverage lost while tilted).
"""
import sys

import numpy as np
from pyulog import ULog


def tilt_deg(D, key):
    q = np.vstack([D["%s[%d]" % (key, i)] for i in range(4)]).T
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


def hold_last(t_src, v_src, t):
    """The last value of a sampled series at the times t (topics publish at a rate and on change)."""
    i = np.clip(np.searchsorted(t_src, t, side="right") - 1, 0, len(v_src) - 1)
    return v_src[i]


def active_mask(t, hook_t, hook_action, lp_t, altitude_m):
    """True where the vehicle is airborne (above 1 m) and the hook enforces (action >= 2), sample by sample, so
    Hold, other modes and time on the ground between flights do not count."""
    enforcing = hold_last(hook_t, np.asarray(hook_action), t) >= 2
    airborne = hold_last(lp_t, np.asarray(altitude_m), t) > 1.0
    return enforcing & airborne


def episodes(mask, t, min_s):
    out, start = [], None
    for k, m in enumerate(mask):
        if m and start is None:
            start = t[k]
        if not m and start is not None:
            if t[k] - start >= min_s:
                out.append((start, t[k] - start))
            start = None
    if start is not None and t[-1] - start >= min_s:  # still held when the log ends
        out.append((start, t[-1] - start))
    return out


def window_max(t, v, t0, t1):
    """Largest v with t0 < t < t1, or NaN when no sample falls in the window (topics have their own clocks)."""
    w = (t > t0) & (t < t1)
    return float(np.max(v[w])) if w.any() else float("nan")


def check(path):
    d = {x.name: x.data for x in ULog(path).data_list if x.multi_id == 0}
    h, lp, att, ats, av, mc, c = (d[n] for n in ("cp_brake_hook", "vehicle_local_position", "vehicle_attitude",
                                                 "vehicle_attitude_setpoint", "vehicle_angular_velocity",
                                                 "manual_control_setpoint", "cp_brake_constraints"))
    th = h["timestamp"] / 1e6
    tl = lp["timestamp"] / 1e6
    alt = -lp["z"]

    def active(t):
        return active_mask(t, th, h["action"], tl, alt)

    air = active(tl)
    minutes = np.sum(np.diff(tl) * air[:-1]) / 60
    if minutes <= 0:
        print("%s: no time airborne with cp_brake enforcing" % path.split("/")[-1])
        return
    speed = np.hypot(lp["vx"], lp["vy"])
    tm = mc["timestamp"] / 1e6
    stick = np.interp(tl, tm, np.hypot(mc["pitch"], mc["roll"]))
    stuck = episodes(air & (stick > 0.25) & (speed < 0.3), tl, 0.8)
    ta = att["timestamp"] / 1e6
    tilt = tilt_deg(att, "q")
    on_att = active(ta)
    big = []
    for k in np.where((tilt > 20) & on_att)[0]:
        if not big or ta[k] - big[-1] > 2.0:
            big.append(ta[k])
    tts = ats["timestamp"] / 1e6
    cmd = tilt_deg(ats, "q_d")
    steps = int(((np.abs(np.diff(cmd)) > 10) & active(tts)[1:]).sum())
    tav = av["timestamp"] / 1e6
    rate = np.hypot(av["xyz[0]"], av["xyz[1]"])
    n = max(int(0.3 / np.median(np.diff(tav))), 1)
    jitter = np.degrees(np.abs(rate - np.convolve(rate, np.ones(n) / n, mode="same")))
    lim = (h["action"] >= 3).astype(int)
    on_hook = active(th)
    tog = []
    for s in np.arange(tl[air].min(), tl[air].max() - 0.5, 0.5):
        m = (th >= s) & (th < s + 0.5) & on_hook
        if m.sum() > 1 and np.abs(np.diff(lim[m])).sum() >= 2:
            k = (tav >= s) & (tav < s + 0.5)
            tog.append(jitter[k].mean())
    ff = np.nan_to_num(np.hypot(h["acc_sp_in[0]"], h["acc_sp_in[1]"]))[(h["action"] >= 3) & on_hook]
    tc = c["timestamp"] / 1e6
    on_c = active(tc)
    caps = np.vstack([c["caps_m_s[%d]" % s] for s in range(72)]).T
    drops = []
    for k in range(2, len(tc)):
        if on_c[k]:
            fwd = [s for s in np.where((caps[k - 2] > 3.0) & (caps[k] < 0.05))[0] if s <= 9 or s >= 63]
            if fwd and (not drops or tc[k] - drops[-1] > 1.0):
                drops.append(tc[k])
    after = [window_max(ta, tilt, t, t + 1.5) for t in drops]
    after = [a for a in after if np.isfinite(a)]
    print("%s: %.1f min airborne with cp_brake enforcing, max altitude %.1f m, max speed %.1f m/s" % (
        path.split("/")[-1], minutes, alt.max(), speed[air].max()))
    print("  stick held but no motion (>= 0.8 s): %d episodes, %.1f s per minute" % (len(stuck), sum(e[1] for e in stuck) / minutes))
    print("  tilt above 20 deg: %.1f events per minute, max %.0f deg" % (len(big) / minutes, tilt[on_att].max()))
    print("  tilt command steps above 10 deg: %.1f per minute" % (steps / minutes))
    print("  hook switching windows: %d, body-rate jitter in them %.1f deg/s" % (len(tog), np.mean(tog) if tog else 0.0))
    print("  feed-forward cleared while limiting: median %.2f, 90%% %.2f m/s^2" % (np.median(ff) if len(ff) else 0, np.percentile(ff, 90) if len(ff) else 0))
    print("  forward sectors free -> 0 in one step: %.1f per minute, max tilt after %.0f deg" % (len(drops) / minutes, max(after) if after else 0))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    for p in sys.argv[1:]:
        check(p)
