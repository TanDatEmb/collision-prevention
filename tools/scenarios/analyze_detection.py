#!/usr/bin/env python3
"""Per-frame detection of a test object in the published ObstacleDistance of a ULog (bench tool, VM-56).

usage: analyze_detection.py --object <name> <log.ulg> --tolerance T [--band-up 0.25] [--band-down 0.60]
                            [--certified 40.0] [--corridor 2.0] [--min-approach-speed 1.0]

--tolerance is how much farther than the true distance a reported distance may be and still count as detected. It
moves frames between 'detected' and 'over-claim', so it is an acceptance threshold: TBD (owner), no default. 0 is
the strict reading (any overstatement is an over-claim); report the value used with every result.

The object is one of `detection_objects.py` (same frame and position). Only the part of the object within --corridor
of the leg (the line North = 0 flown East from the takeoff origin; give CPB_MARGIN) counts: that is the part the
vehicle would hit or pass inside the margin (ADR 0025), and it keeps the far ends of a long object (and what holds
them up) out of the verdict. For every `obstacle_distance` frame the ground truth of each sector is the horizontal
distance to the nearest such point inside the height band (lidar height - band_down to + band_up; give the band of
the platform file) whose bearing falls in the sector (sectors are centred on 5 i degrees clockwise from the heading,
ICD-CP-03), within the certified range. A frame is
- detected: a sector that holds the object reports a confirmed distance (below the certified range) no farther than
  truth + --tolerance;
- unknown: not detected, and a sector that holds the object is unknown (`UINT16_MAX`): its cap is 0 (fail closed);
- over-claim: neither, so every sector that holds the object reports free space past it.
Only frames while the vehicle flies faster than --min-approach-speed are counted. The output (one JSON object) gives
the share of each class per range bin, the range from which no frame over-claims, the number of changes between
'seen' (detected or unknown) and 'over-claim' along the range, and the closest 3-D approach to the object.
The vehicle position is `vehicle_local_position_groundtruth`, the heading `vehicle_attitude`; the lidar height is
taken as the vehicle height (give the band relative to it). Requires pyulog and numpy.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detection_objects  # noqa: E402

SECTORS = 72
UNKNOWN_CM = 65535
BINS_M = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 30), (30, 40)]


def decode_wire(distances_cm, max_cm):
    """ObstacleDistance values in metres: the clear marker (max + 1) is the max range, UINT16_MAX is NaN."""
    d = np.asarray(distances_cm, dtype=float)
    out = d / 100.0
    out[d == max_cm + 1] = max_cm / 100.0
    out[d == UNKNOWN_CM] = np.nan
    return out


def sector_truth(points_enz, north_m, east_m, height_m, yaw_rad, band_up_m, band_down_m, max_range_m):
    """Per sector the horizontal distance to the nearest object point in the band (inf where none)."""
    truth = np.full(SECTORS, np.inf)
    in_band = (points_enz[:, 2] >= height_m - band_down_m) & (points_enz[:, 2] <= height_m + band_up_m)
    if not in_band.any():
        return truth
    dn = points_enz[in_band, 1] - north_m
    de = points_enz[in_band, 0] - east_m
    r = np.hypot(dn, de)
    near = r <= max_range_m
    if not near.any():
        return truth
    bearing = (np.degrees(np.arctan2(de[near], dn[near]) - yaw_rad)) % 360.0
    sector = (((bearing + 2.5) % 360.0) // 5.0).astype(int) % SECTORS
    np.minimum.at(truth, sector, r[near])
    return truth


def classify_frame(reported_m, truth_m, certified_m, tolerance_m):
    """'detected', 'unknown', 'overclaim', or None when no sector holds the object."""
    held = np.isfinite(truth_m)
    if not held.any():
        return None
    rep, tru = reported_m[held], truth_m[held]
    confirmed = np.isfinite(rep) & (rep < certified_m - 0.05)
    if np.any(confirmed & (rep <= tru + tolerance_m)):
        return 'detected'
    if np.any(np.isnan(rep)):
        return 'unknown'
    return 'overclaim'


def summarize(ranges_m, classes):
    """Share of each class per bin, the range from which no frame over-claims, and the changes along the range."""
    ranges_m = np.asarray(ranges_m, dtype=float)
    classes = np.asarray(classes)
    bins = {}
    for a, b in BINS_M:
        m = (ranges_m >= a) & (ranges_m < b)
        if m.sum() == 0:
            bins['%d-%d m' % (a, b)] = None
            continue
        bins['%d-%d m' % (a, b)] = {c: round(float(np.mean(classes[m] == c)), 3) for c in ('detected', 'unknown', 'overclaim')}
        bins['%d-%d m' % (a, b)]['frames'] = int(m.sum())
    order = np.argsort(-ranges_m)
    seen = classes[order] != 'overclaim'
    no_overclaim_from = 0.0
    for i in range(len(order)):
        if seen[i:].all():
            no_overclaim_from = float(ranges_m[order[i]])
            break
    changes = int(np.sum(seen[1:] != seen[:-1])) if len(seen) > 1 else 0
    first = float(ranges_m[classes == 'detected'].max()) if np.any(classes == 'detected') else None
    return {'bins': bins, 'no_overclaim_from_m': round(no_overclaim_from, 2), 'seen_changes': changes,
            'first_detected_m': None if first is None else round(first, 2)}


def topic(log, name):
    for d in log.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data
    raise SystemExit('the log has no %s' % name)


def analyze(path, name, band_up_m, band_down_m, tolerance_m, certified_m, corridor_m, min_speed_m_s):
    from pyulog import ULog
    log = ULog(path)
    gt = topic(log, 'vehicle_local_position_groundtruth')
    od = topic(log, 'obstacle_distance')
    att = topic(log, 'vehicle_attitude')
    tg = gt['timestamp'] * 1e-6
    q = [att['q[%d]' % i] for i in range(4)]
    yaw = np.unwrap(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
    ta = att['timestamp'] * 1e-6
    points = detection_objects.surface_points(name)
    points = points[np.abs(points[:, 1]) <= corridor_m]  # Gazebo y is North; the leg is North = 0
    ranges, classes = [], []
    for k, t in enumerate(od['timestamp'] * 1e-6):
        if t < tg[0] or t > tg[-1]:
            continue
        speed = math.hypot(np.interp(t, tg, gt['vx']), np.interp(t, tg, gt['vy']))
        if speed <= min_speed_m_s:
            continue
        n, e, z = np.interp(t, tg, gt['x']), np.interp(t, tg, gt['y']), -np.interp(t, tg, gt['z'])
        truth = sector_truth(points, n, e, z, np.interp(t, ta, yaw), band_up_m, band_down_m, certified_m)
        reported = decode_wire([od['distances[%d]' % i][k] for i in range(SECTORS)], int(od['max_distance'][k]))
        c = classify_frame(reported, truth, certified_m, tolerance_m)
        if c is not None:
            ranges.append(float(truth[np.isfinite(truth)].min()))
            classes.append(c)
    result = {'log': os.path.basename(path), 'object': name, 'tolerance_m': tolerance_m, 'frames': len(classes)}
    if classes:
        result.update(summarize(ranges, classes))
    every = detection_objects.surface_points(name, z_range_m=(-1.0, 40.0))
    step = max(1, len(tg) // 4000)
    north, east, up = gt['x'][::step], gt['y'][::step], -gt['z'][::step]
    closest = min(float(np.min(np.sqrt((every[:, 0] - east[i]) ** 2 + (every[:, 1] - north[i]) ** 2
                                      + (every[:, 2] - up[i]) ** 2))) for i in range(len(north)))
    result['closest_approach_m'] = round(closest, 2)
    result['final_east_m'] = round(float(gt['y'][-1]), 2)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--object', required=True, choices=sorted(detection_objects.OBJECTS))
    ap.add_argument('log')
    ap.add_argument('--band-up', type=float, default=0.25, help='height_up_m of the platform file [m]')
    ap.add_argument('--band-down', type=float, default=0.60, help='height_down_m of the platform file [m]')
    ap.add_argument('--tolerance', type=float, required=True,
                    help='reported distance allowed past the truth [m]: an acceptance threshold, TBD (owner); 0 = strict')
    ap.add_argument('--certified', type=float, default=40.0, help='certified_range_m of the run [m]')
    ap.add_argument('--corridor', type=float, default=2.0, help='half-width around the leg that counts, CPB_MARGIN [m]')
    ap.add_argument('--min-approach-speed', type=float, default=1.0, help='frames counted above this speed [m/s]')
    a = ap.parse_args()
    print(json.dumps(analyze(a.log, a.object, a.band_up, a.band_down, a.tolerance, a.certified, a.corridor,
                             a.min_approach_speed)))


if __name__ == '__main__':
    main()
