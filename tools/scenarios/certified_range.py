#!/usr/bin/env python3
"""Certified free range and the speed it allows (ADR 0024), from the platform's physical inputs (bench tool).

usage: certified_range.py --h-obj 1.0 --w-obj 1.5 --allowance-deg 0.9 [--gap-v-deg 1.07] [--gap-h-deg 1.0]
                          [--dz 0.07] [--band 0.85] [--n-obstacle 3] [--t-lat 0.95] [--a-brake 3.0] [--jerk 4.0]
                          [--margin 2.0] [--range-cap 40.0] [--table] [--selftest]

`certified_range_m = min(r_h, r_w, range cap)` with
  r_h = (h_obj - dz) / tan(gap_v + allowance)                        smallest height crossed by a ray
  r_w = largest r with ceil(n_obstacle / k(r)) * r * tan(gap_h) <= w_obj,
        k(r) = floor(min(h_obj - dz, band) / (r tan gap_v)): the rings that cross the smallest object in the band
the speed is the cp_brake stopping model: v * t_lat + braking distance with jerk + margin <= range. The output is a
design aid: the values it prints are not approved; the owner chooses h_obj, w_obj and the allowance, and the gaps
are measured on the real sensor (OPEN-023). --table prints the trade-off of the allowance and the object size.
"""
import argparse
import math


def r_height(h_obj, dz, gap_v_deg, allowance_deg):
    return (h_obj - dz) / math.tan(math.radians(gap_v_deg + allowance_deg))


def r_width(w_obj, h_obj, dz, gap_v_deg, gap_h_deg, band, n_obstacle, cap):
    # Only the rings that cross the smallest object count as its returns; at least one is guaranteed below r_h.
    overlap = min(h_obj - dz, band)
    r = cap
    while r > 0.1:
        rings = max(1, math.floor(overlap / (r * math.tan(math.radians(gap_v_deg)))))
        if math.ceil(n_obstacle / rings) * r * math.tan(math.radians(gap_h_deg)) <= w_obj:
            return r
        r -= 0.05
    return 0.0


def stopping_distance(v, t_lat, a, j, margin):
    braking = v * math.sqrt(v / j) if v < a * a / j else v * v / (2 * a) + v * a / (2 * j)
    return v * t_lat + braking + margin


def speed_for(range_m, t_lat, a, j, margin, v_max=30.0):
    if range_m <= margin:
        return 0.0
    lo, hi = 0.0, v_max
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if stopping_distance(mid, t_lat, a, j, margin) <= range_m else (lo, mid)
    return lo


def evaluate(a):
    rh = r_height(a.h_obj, a.dz, a.gap_v_deg, a.allowance_deg)
    rw = r_width(a.w_obj, a.h_obj, a.dz, a.gap_v_deg, a.gap_h_deg, a.band, a.n_obstacle, a.range_cap)
    certified = max(0.0, min(rh, rw, a.range_cap))
    return rh, rw, certified, speed_for(certified, a.t_lat, a.a_brake, a.jerk, a.margin)


def selftest():
    class A:
        pass
    a = A()
    a.h_obj, a.w_obj, a.allowance_deg, a.gap_v_deg, a.gap_h_deg = 1.0, 1.5, 0.9, 1.07, 1.0
    a.dz, a.band, a.n_obstacle, a.t_lat, a.a_brake, a.jerk, a.margin, a.range_cap = 0.07, 0.85, 3, 0.95, 3.0, 4.0, 2.0, 40.0
    rh, rw, c, v = evaluate(a)
    checks = [
        ('r_h of the bench example', abs(rh - 27.0) < 0.1),
        ('certified range of the bench example', abs(c - 27.0) < 0.1),
        ('stopping distance at 12 m/s is 41.9 m', abs(stopping_distance(12.0, 0.95, 3.0, 4.0, 2.0) - 41.9) < 0.05),
        ('the speed meets the stopping distance', abs(stopping_distance(v, 0.95, 3.0, 4.0, 2.0) - c) < 1e-3),
        ('jerk-limited branch below a^2/j', abs(stopping_distance(1.0, 0.0, 3.0, 4.0, 0.0) - 0.5) < 1e-9),
    ]
    a.allowance_deg, a.w_obj = 0.2, 2.5
    checks.append(('allowance 0.2 deg and 2.5 m reach the cap', abs(evaluate(a)[2] - 40.0) < 1e-9))
    # A 0.5 m x 0.3 m object at 12 m crosses only two rings and one column: three returns are not guaranteed there.
    a.allowance_deg, a.h_obj, a.w_obj = 0.9, 0.5, 0.3
    checks.append(('a small object limits the range by the rings it crosses', evaluate(a)[2] < 12.0))
    for name, ok in checks:
        print('%s: %s' % ('PASS' if ok else 'FAIL', name))
    return all(ok for _, ok in checks)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--h-obj', type=float, help='smallest obstacle height seen in every frame [m]')
    ap.add_argument('--w-obj', type=float, help='smallest obstacle width seen in every frame [m]')
    ap.add_argument('--allowance-deg', type=float, help='attitude allowance, attitude_allowance_rad in degrees')
    ap.add_argument('--gap-v-deg', type=float, default=1.07, help='vertical angular gap in one frame (OPEN-023)')
    ap.add_argument('--gap-h-deg', type=float, default=1.0, help='horizontal angular gap in one frame (OPEN-023)')
    ap.add_argument('--dz', type=float, default=0.07, help='largest vertical origin motion in one frame [m]')
    ap.add_argument('--band', type=float, default=0.85, help='height_up_m + height_down_m [m]')
    ap.add_argument('--n-obstacle', type=int, default=3, help='obstacle_min_returns')
    ap.add_argument('--t-lat', type=float, default=0.95, help='CPB_T_LAT [s]')
    ap.add_argument('--a-brake', type=float, default=3.0, help='CPB_A_BRK [m/s^2]')
    ap.add_argument('--jerk', type=float, default=4.0, help='CPB_JERK [m/s^3]')
    ap.add_argument('--margin', type=float, default=2.0, help='CPB_MARGIN [m]')
    ap.add_argument('--range-cap', type=float, default=40.0, help='qualified range of the wire (ICD-CP-03) [m]')
    ap.add_argument('--table', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        raise SystemExit(0 if selftest() else 1)
    if a.table:
        print('allowance  h_obj  w_obj |   r_h    r_w  certified |  v_max')
        for allowance in (0.9, 0.5, 0.3, 0.2):
            for h, w in ((0.5, 1.5), (1.0, 1.5), (1.0, 2.5), (1.5, 2.5)):
                a.allowance_deg, a.h_obj, a.w_obj = allowance, h, w
                rh, rw, c, v = evaluate(a)
                print('%5.1f deg %5.1f m %5.1f m | %5.1f %6.1f %7.1f m | %4.1f m/s' % (allowance, h, w, rh, rw, c, v))
        print('speed at the range cap %.0f m: %.1f m/s; 12 m/s needs %.1f m' % (
            a.range_cap, speed_for(a.range_cap, a.t_lat, a.a_brake, a.jerk, a.margin),
            stopping_distance(12.0, a.t_lat, a.a_brake, a.jerk, a.margin)))
        return
    if a.h_obj is None or a.w_obj is None or a.allowance_deg is None:
        ap.error('--h-obj, --w-obj and --allowance-deg are required (or --table / --selftest)')
    rh, rw, c, v = evaluate(a)
    print('r_h %.1f m, r_w %.1f m -> certified_range_m %.1f m (attitude_allowance_rad %.4f) -> v_max %.1f m/s'
          % (rh, rw, c, math.radians(a.allowance_deg), v))


if __name__ == '__main__':
    main()
