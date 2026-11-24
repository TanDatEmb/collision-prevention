#!/usr/bin/env python3
"""Certified free range and the speed it allows (ADR 0024), from the platform's physical inputs (bench tool).

usage: certified_range.py --h-obj 1.0 --w-obj 3.0 --allowance-deg 0.9 [--gap-v-deg 1.07] [--gap-h-deg 1.0]
                          [--dz 0.07] [--band 0.85] [--n-obstacle 3] [--t-lat 0.95] [--a-brake 3.0] [--jerk 4.0]
                          [--margin 2.0] [--range-cap 40.0] [--table] [--selftest]

`certified_range_m` = the largest range r (at most the cap) at which one sector is guaranteed n_obstacle returns
from the smallest claimed object (h_obj tall, w_obj wide) lying in the band:
  k(r)    = floor((min(h_obj, band) - dz) / (r tan gap_v))   rings crossing the part of the object inside the band
  cols(r) = floor(min(w_obj / 2, r tan 5 deg) / (r tan gap_h)) columns in the sector holding more of the object
            (an object on a sector boundary is split, and a sector confirms only its own returns)
  k(r) >= 1 and k(r) * cols(r) >= n_obstacle
The rays are physical: an attitude error does not remove one; it can move a return across a band edge, which the
allowance covers in the band test (ADR 0024 decision 3), so the allowance does not shorten the range here.
the speed is the cp_brake stopping model: v * t_lat + braking distance with jerk + margin <= range. The output is a
design aid: the values it prints are not approved; the owner chooses h_obj, w_obj and the allowance, and the gaps
are measured on the real sensor (OPEN-023). --table prints the trade-off of the object size and the gaps.
"""
import argparse
import math


SECTOR_DEG = 5.0


def guaranteed_returns(r, h_obj, w_obj, dz, gap_v_deg, gap_h_deg, band):
    overlap = min(h_obj, band) - dz
    if overlap <= 0.0:
        return 0
    rings = math.floor(overlap / (r * math.tan(math.radians(gap_v_deg))))
    if rings < 1:
        return 0  # no ring is guaranteed on the object inside the band: not certifiable at r
    width = min(w_obj / 2.0, r * math.tan(math.radians(SECTOR_DEG)))
    return rings * math.floor(width / (r * math.tan(math.radians(gap_h_deg))))


def certified_range(h_obj, w_obj, dz, gap_v_deg, gap_h_deg, band, n_obstacle, cap):
    # The guaranteed count does not grow with the range, so the largest passing range is the certified one.
    r = cap
    while r > 0.1:
        if guaranteed_returns(r, h_obj, w_obj, dz, gap_v_deg, gap_h_deg, band) >= n_obstacle:
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
    rh = (min(a.h_obj, a.band) - a.dz) / math.tan(math.radians(a.gap_v_deg))  # one ring on the object in the band
    certified = certified_range(a.h_obj, a.w_obj, a.dz, a.gap_v_deg, a.gap_h_deg, a.band, a.n_obstacle, a.range_cap)
    return rh, certified, speed_for(certified, a.t_lat, a.a_brake, a.jerk, a.margin)


def selftest():
    class A:
        pass
    a = A()
    a.h_obj, a.w_obj, a.allowance_deg, a.gap_v_deg, a.gap_h_deg = 1.0, 1.5, 0.9, 1.07, 1.0
    a.dz, a.band, a.n_obstacle, a.t_lat, a.a_brake, a.jerk, a.margin, a.range_cap = 0.07, 0.85, 3, 0.95, 3.0, 4.0, 2.0, 40.0
    checks = [
        ('stopping distance at 12 m/s is 41.9 m', abs(stopping_distance(12.0, 0.95, 3.0, 4.0, 2.0) - 41.9) < 0.05),
        ('jerk-limited branch below a^2/j', abs(stopping_distance(1.0, 0.0, 3.0, 4.0, 0.0) - 0.5) < 1e-9),
    ]
    rh, c, v = evaluate(a)
    checks.append(('the speed meets the stopping distance', abs(stopping_distance(v, 0.95, 3.0, 4.0, 2.0) - c) < 1e-3))
    # Review counterexamples (PR #191): each must stay below the range where it fails.
    checks.append(('a 1.0 x 1.5 m object split by a sector boundary at 26.8 m', c < 26.8))
    a.h_obj, a.w_obj = 0.5, 0.3
    checks.append(('a 0.5 x 0.3 m object at 12 m (two rings, one column)', evaluate(a)[1] < 12.0))
    a.h_obj, a.w_obj, a.gap_v_deg = 1.0, 1.5, 2.0
    checks.append(('a 2 deg grid has no ring in the band at 26 m', evaluate(a)[1] < 26.0))
    a.gap_v_deg = 1.07
    # The certified range holds the condition at every shorter range (the count never grows with the range).
    c = evaluate(a)[1]
    checks.append(('monotonic below the certified range', all(
        guaranteed_returns(r, 1.0, 1.5, 0.07, 1.07, 1.0, 0.85) >= 3 for r in [0.5 + 0.1 * i for i in range(int(c * 10) - 5)])))
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
        print('h_obj  w_obj  gap_v  |  r_h(1 ring)  certified |  v_max')
        for gap in (1.07, 0.5):
            for h, w in ((0.5, 1.5), (1.0, 1.5), (1.0, 3.0), (1.0, 5.0), (2.0, 5.0)):
                a.h_obj, a.w_obj, a.gap_v_deg = h, w, gap
                rh, c, v = evaluate(a)
                print('%4.1f m %5.1f m %5.2f deg | %8.1f m %8.1f m | %4.1f m/s' % (h, w, gap, rh, c, v))
        print('speed at the range cap %.0f m: %.1f m/s; 12 m/s needs %.1f m' % (
            a.range_cap, speed_for(a.range_cap, a.t_lat, a.a_brake, a.jerk, a.margin),
            stopping_distance(12.0, a.t_lat, a.a_brake, a.jerk, a.margin)))
        return
    if a.h_obj is None or a.w_obj is None or a.allowance_deg is None:
        ap.error('--h-obj, --w-obj and --allowance-deg are required (or --table / --selftest)')
    rh, c, v = evaluate(a)
    print('one ring on the object up to %.1f m -> certified_range_m %.1f m (attitude_allowance_rad %.4f, for the band '
          'test) -> v_max %.1f m/s' % (rh, c, math.radians(a.allowance_deg), v))


if __name__ == '__main__':
    main()
