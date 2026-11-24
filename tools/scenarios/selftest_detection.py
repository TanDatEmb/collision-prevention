#!/usr/bin/env python3
"""Self-test of analyze_detection.py and detection_objects.py on synthetic frames. It tests the classification and
the geometry only; it says nothing about the sensor or PX4. Run: python3 -I tools/scenarios/selftest_detection.py
(exit 0 when every case passes).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_detection as A  # noqa: E402
import detection_objects as O  # noqa: E402


def frame(values):
    """72 reported distances (m): 40 (certified range) everywhere unless given as {sector: value}."""
    r = np.full(72, 40.0)
    for k, v in values.items():
        r[k] = v
    return r


def truth(values):
    t = np.full(72, np.inf)
    for k, v in values.items():
        t[k] = v
    return t


def main():
    checks = []
    c = A.classify_frame
    checks.append(('a confirmed distance at the truth is detected', c(frame({0: 20.1}), truth({0: 20.0}), 40, 0.5) == 'detected'))
    checks.append(('nearer than the truth (another return) is detected', c(frame({0: 12.0}), truth({0: 20.0}), 40, 0.5) == 'detected'))
    checks.append(('past the truth by more than the tolerance is not', c(frame({0: 21.0}), truth({0: 20.0}), 40, 0.5) == 'overclaim'))
    checks.append(('strict tolerance 0: 1 cm past the truth is an over-claim', c(frame({0: 20.01}), truth({0: 20.0}), 40, 0.0) == 'overclaim'))
    checks.append(('strict tolerance 0: at the truth is detected', c(frame({0: 20.0}), truth({0: 20.0}), 40, 0.0) == 'detected'))
    checks.append(('the certified range is free space, not a detection',
                   c(frame({0: 40.0}), truth({0: 39.8}), 40, 0.5) == 'overclaim'))
    checks.append(('an unknown sector holding the object is unknown', c(frame({0: np.nan}), truth({0: 20.0}), 40, 0.5) == 'unknown'))
    checks.append(('detected wins over unknown', c(frame({0: np.nan, 1: 20.0}), truth({0: 20.0, 1: 20.0}), 40, 0.5) == 'detected'))
    checks.append(('no sector holds the object: no class', c(frame({}), truth({}), 40, 0.5) is None))
    checks.append(('a sector without the object does not count', c(frame({5: 20.0}), truth({0: 20.0}), 40, 0.5) == 'overclaim'))

    d = A.decode_wire([4001, 65535, 0, 1234], 4000)
    checks.append(('wire: clear marker is the max range', d[0] == 40.0))
    checks.append(('wire: UINT16_MAX is unknown', math.isnan(d[1])))
    checks.append(('wire: 0 (too close) stays 0 and centimetres become metres', d[2] == 0.0 and abs(d[3] - 12.34) < 1e-9))

    # a point 20 m East of a vehicle heading East (yaw 90 deg) is in sector 0; 10 deg to the right is sector 2
    pts = np.array([[20.0, 0.0, 12.0], [20 * math.cos(math.radians(10)), -20 * math.sin(math.radians(10)), 12.0]])
    t = A.sector_truth(pts, 0.0, 0.0, 12.0, math.radians(90), 0.25, 0.60, 40.0)
    checks.append(('ahead is sector 0', abs(t[0] - 20.0) < 1e-6))
    checks.append(('10 deg to the right (clockwise) is sector 2', abs(t[2] - 20.0) < 1e-6))
    t_high = A.sector_truth(pts + [0, 0, 0.3], 0.0, 0.0, 12.0, math.radians(90), 0.25, 0.60, 40.0)
    checks.append(('above the band is not held', not np.isfinite(t_high).any()))
    t_far = A.sector_truth(pts * [3, 3, 1], 0.0, 0.0, 12.0, math.radians(90), 0.25, 0.60, 40.0)
    checks.append(('beyond the certified range is not held', not np.isfinite(t_far).any()))

    s = A.summarize([35, 30, 25, 18, 16, 12, 8, 4], ['detected', 'overclaim', 'detected', 'overclaim', 'unknown',
                                                     'detected', 'detected', 'detected'])
    checks.append(('no over-claim from the farthest frame after the last over-claim', s['no_overclaim_from_m'] == 16.0))
    checks.append(('changes between seen and over-claim are counted', s['seen_changes'] == 4))
    checks.append(('the first detection is the farthest detected frame', s['first_detected_m'] == 35.0))
    checks.append(('bin shares', s['bins']['15-20 m']['overclaim'] == 0.5 and s['bins']['15-20 m']['unknown'] == 0.5))

    checks.append(('every object has points in the band at the flight altitude',
                   all(len(O.surface_points(n)) > 0 for n in O.OBJECTS)))
    wire_all = O.surface_points('wire', supports=True)
    wire = O.surface_points('wire')
    checks.append(('the supports of the wire are left out of the truth', len(wire) < len(wire_all)
                   and np.all(np.abs(wire[:, 1]) <= 15.0 + 1e-6) and np.all(np.abs(wire[:, 2] - O.ALTITUDE_M) <= 0.011)))
    checks.append(('the SDF holds every primitive', all(O.sdf(n).count('<collision') == len(O.OBJECTS[n]) for n in O.OBJECTS)))

    for name, ok in checks:
        print('%s: %s' % ('PASS' if ok else 'FAIL', name))
    return all(ok for _, ok in checks)


if __name__ == '__main__':
    raise SystemExit(0 if main() else 1)
