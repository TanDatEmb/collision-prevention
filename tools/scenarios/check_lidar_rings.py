#!/usr/bin/env python3
"""Static check of the bench multi-ring lidar against a known wall and the ground (issue #138).

The vehicle must be static in a Gazebo world with a flat ground (z = 0) and a wall face at x = --wall-x
(spawn_wall.sh), yaw 0. The script reads one `gz.msgs.LaserScan` (all rings, ring-major: index =
ring * count + azimuth step) from the gz topic, ray-casts the same scene analytically from the sensor
pose and compares every ray. It also compares the scan layout of the message with the SDF of the model.

A gpu_lidar samples a rendered depth image, so its rays are not exact: at a grazing angle a small angular
error moves the range a lot (about r^2 / h per radian for the ground). The comparison is therefore made
in angle, not in metres: a ray agrees when its range lies inside the range envelope of the same scene
cast with the ray turned by up to `eps` in azimuth and in elevation. The script reports the smallest
`eps` of a fixed ladder that makes each ray agree, and fails when a ray needs more than
--max-angle-error-deg or when a ray is shorter than any envelope (a hit on the vehicle itself).

Bench tool, simulation only: it proves what the simulated sensor does, not what a Livox does.

usage: check_lidar_rings.py [--topic T] [--vehicle-z 2.0] [--wall-x 5.0]
Exit code 0 when every ray agrees, 1 otherwise.
"""
import argparse
import math
import sys
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_TOPIC = '/world/default/model/bench_vehicle/link/lidar_3d_link/sensor/lidar_3d_bench/scan'
MODELS = Path(__file__).resolve().parent / 'models'
# Mount R1 in the model frame of x500_lidar_3d_bench (ADR 0022): x, y, z and pitch of the sensor.
MOUNT_X, MOUNT_Y, MOUNT_Z, MOUNT_PITCH = 0.12, 0.0, 0.38, 0.17453293
EPS_LADDER_DEG = (0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5)


def sdf_layout(path):
    """The scan numbers of the sensor in the SDF file."""
    sensor = ET.parse(path).getroot().find('.//sensor')
    scan, rng = sensor.find('ray/scan'), sensor.find('ray/range')
    horizontal, vertical = scan.find('horizontal'), scan.find('vertical')

    def value(node, tag):
        return float(node.find(tag).text)

    return {
        'azimuth_count': int(value(horizontal, 'samples')),
        'azimuth_min': value(horizontal, 'min_angle'),
        'azimuth_max': value(horizontal, 'max_angle'),
        'ring_count': int(value(vertical, 'samples')),
        'elevation_min': value(vertical, 'min_angle'),
        'elevation_max': value(vertical, 'max_angle'),
        'range_min': value(rng, 'min'),
        'range_max': value(rng, 'max'),
    }


def yaml_layout_problems(yaml_path, layout):
    """Differences between the ring layout YAML of the node and the SDF of the model."""
    import yaml
    params = yaml.safe_load(Path(yaml_path).read_text())['/**']['ros__parameters']['ring_layout']
    az_step = (layout['azimuth_max'] - layout['azimuth_min']) / (layout['azimuth_count'] - 1)
    el_step = (layout['elevation_max'] - layout['elevation_min']) / (layout['ring_count'] - 1)
    want = {
        'azimuth_count': layout['azimuth_count'], 'azimuth_min_rad': layout['azimuth_min'],
        'azimuth_step_rad': az_step, 'ring_count': layout['ring_count'],
        'elevation_min_rad': layout['elevation_min'], 'elevation_step_rad': el_step,
        'range_min_m': layout['range_min'], 'range_max_m': layout['range_max'],
    }
    return [f'layout YAML {key}: {params.get(key)} != SDF {value}' for key, value in want.items()
            if params.get(key) is None or abs(float(params[key]) - value) > 1e-8]


def cast(origin, az, el, args, range_min, range_max):
    """Expected range of the rays (arrays of sensor-frame azimuth and elevation), inf for no hit."""
    ds = (np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el))
    sp, cp = math.sin(MOUNT_PITCH), math.cos(MOUNT_PITCH)
    # R_y(pitch): a positive pitch turns +x towards -z (boresight down).
    dx, dy, dz = cp * ds[0] + sp * ds[2], ds[1], -sp * ds[0] + cp * ds[2]
    best = np.full(dx.shape, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        t_wall = (args.wall_x - origin[0]) / dx
        y_hit, z_hit = origin[1] + t_wall * dy, origin[2] + t_wall * dz
        wall = (dx > 1e-9) & (t_wall > 0) & (np.abs(y_hit) <= args.wall_half_width) & \
               (z_hit >= 0.0) & (z_hit <= args.wall_height)
        best = np.where(wall, np.minimum(best, t_wall), best)
        t_ground = (0.0 - origin[2]) / dz
        ground = (dz < -1e-9) & (t_ground > 0)
        best = np.where(ground, np.minimum(best, t_ground), best)
    best = np.where(best > range_max, np.inf, best)
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--topic', default=DEFAULT_TOPIC)
    parser.add_argument('--vehicle-x', type=float, default=0.0)
    parser.add_argument('--vehicle-y', type=float, default=0.0)
    parser.add_argument('--vehicle-z', type=float, default=2.0, help='height of the model origin (m)')
    parser.add_argument('--wall-x', type=float, default=5.0, help='x of the wall face (m)')
    parser.add_argument('--wall-half-width', type=float, default=40.0)
    parser.add_argument('--wall-height', type=float, default=30.0)
    parser.add_argument('--range-tolerance', type=float, default=0.02, help='range tolerance (m)')
    parser.add_argument('--max-angle-error-deg', type=float, default=0.3,
                        help='largest angular error a ray may need (bench value of the simulated sensor)')
    parser.add_argument('--sdf', default=str(MODELS / 'lidar_3d_bench' / 'model.sdf'))
    parser.add_argument('--layout-yaml', default=str(Path(__file__).resolve().parent / 'lidar_3d_bench_layout.yaml'),
                        help='ring layout of the obstacle_sector_node source, compared with the SDF')
    parser.add_argument('--timeout', type=float, default=15.0)
    args = parser.parse_args()

    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.transport13 import Node

    event, box = threading.Event(), {}

    def on_scan(message):
        box.setdefault('scan', message)
        event.set()

    node = Node()
    if not node.subscribe(LaserScan, args.topic, on_scan):
        print('cannot subscribe to', args.topic)
        return 1
    if not event.wait(args.timeout):
        print('no scan on', args.topic)
        return 1
    scan = box['scan']
    layout = sdf_layout(args.sdf)
    problems = yaml_layout_problems(args.layout_yaml, layout)
    print('layout YAML agrees with the SDF' if not problems else 'layout YAML differs from the SDF')

    # 1. The message agrees with the SDF.
    if scan.count != layout['azimuth_count'] or scan.vertical_count != layout['ring_count']:
        problems.append(f'size {scan.count}x{scan.vertical_count} != SDF '
                        f'{layout["azimuth_count"]}x{layout["ring_count"]}')
    if len(scan.ranges) != scan.count * scan.vertical_count:
        problems.append(f'{len(scan.ranges)} ranges != count * vertical_count')
        return 1
    for name, got, want in (
            ('azimuth min', scan.angle_min, layout['azimuth_min']),
            ('azimuth max', scan.angle_max, layout['azimuth_max']),
            ('elevation min', scan.vertical_angle_min, layout['elevation_min']),
            ('elevation max', scan.vertical_angle_max, layout['elevation_max']),
            ('range min', scan.range_min, layout['range_min']),
            ('range max', scan.range_max, layout['range_max'])):
        if abs(got - want) > 1e-5:
            problems.append(f'{name}: message {got} != SDF {want}')
    azimuth_step = (layout['azimuth_max'] - layout['azimuth_min']) / (layout['azimuth_count'] - 1)
    elevation_step = (layout['elevation_max'] - layout['elevation_min']) / (layout['ring_count'] - 1)
    print(f'layout: {scan.count} azimuth steps x {scan.vertical_count} rings, azimuth step '
          f'{math.degrees(azimuth_step):.4f} deg, elevation step {math.degrees(elevation_step):.4f} deg, '
          f'azimuth {math.degrees(scan.angle_min):.2f}..{math.degrees(scan.angle_max):.2f}, '
          f'elevation {math.degrees(scan.vertical_angle_min):.2f}..{math.degrees(scan.vertical_angle_max):.2f}, '
          f'range {scan.range_min}..{scan.range_max} m')
    if abs((layout['azimuth_max'] + azimuth_step - layout['azimuth_min']) - 2 * math.pi) > 1e-4:
        problems.append('azimuth grid is not a closed 360 degrees without a repeated azimuth')

    # 2. Ray-cast the scene with angle tolerance and compare every ray.
    origin = (args.vehicle_x + MOUNT_X, args.vehicle_y + MOUNT_Y, args.vehicle_z + MOUNT_Z)
    rings, steps = np.meshgrid(np.arange(scan.vertical_count), np.arange(scan.count), indexing='ij')
    el = layout['elevation_min'] + rings * elevation_step
    az = layout['azimuth_min'] + steps * azimuth_step
    got = np.array(scan.ranges, dtype=float).reshape(scan.vertical_count, scan.count)
    tol = args.range_tolerance

    needed = np.full(got.shape, np.inf)       # smallest eps (deg) at which the ray agrees
    nominal = cast(origin, az, el, args, layout['range_min'], layout['range_max'])
    shorter = np.zeros(got.shape, dtype=bool)  # measured finite and shorter than the widest envelope
    for eps_deg in EPS_LADDER_DEG:
        eps = math.radians(eps_deg)
        cands = [nominal]
        if eps > 0:
            for d_az, d_el in ((eps, 0), (-eps, 0), (0, eps), (0, -eps), (eps, eps), (-eps, -eps),
                               (eps, -eps), (-eps, eps)):
                cands.append(cast(origin, az + d_az, el + d_el, args, layout['range_min'], layout['range_max']))
        stack = np.stack(cands)
        lo, hi = stack.min(axis=0), stack.max(axis=0)
        finite_got = np.isfinite(got)
        agree = np.where(finite_got, (got >= lo - tol) & (got <= hi + tol),
                         (got > 0) & np.isinf(hi))
        newly = agree & np.isinf(needed)
        needed[newly] = eps_deg
        if eps_deg == EPS_LADDER_DEG[-1]:
            shorter = finite_got & ~agree & (got < lo - tol)

    total = got.size
    print(f'sensor at {tuple(round(v, 3) for v in origin)} m, pitch {math.degrees(MOUNT_PITCH):.2f} deg down, '
          f'wall face x = {args.wall_x} m, ground z = 0')
    print('smallest angular error that makes a ray agree with the known scene:')
    for eps_deg in EPS_LADDER_DEG:
        count = int(np.sum(needed == eps_deg))
        print(f'  {eps_deg:4.2f} deg: {count:6d} rays ({100.0 * count / total:5.1f} %)')
    never = int(np.sum(np.isinf(needed)))
    print(f'  more than {EPS_LADDER_DEG[-1]} deg: {never} rays; of them {int(shorter.sum())} shorter than any envelope')
    # Front ray of every sixth ring: expected and measured.
    front = int(round((0.0 - layout['azimuth_min']) / azimuth_step))
    print(f'front ray (azimuth step {front}) per ring, expected / measured (m):')
    for ring in list(range(0, scan.vertical_count, 6)) + [scan.vertical_count - 1]:
        e, g = nominal[ring, front], got[ring, front]
        print(f'  ring {ring:2d} (sensor elevation {math.degrees(el[ring, front]):+6.2f} deg): '
              f'{e:7.3f} / {g:7.3f}')
    limit = args.max_angle_error_deg
    worst = float(np.max(needed[np.isfinite(needed)])) if np.any(np.isfinite(needed)) else math.nan
    over = int(np.sum(~(needed <= limit)))
    print(f'rays {total}; need at most {limit} deg: {total - over}; over: {over}; own-body style hits '
          f'(shorter than any envelope): {int(shorter.sum())}; worst agreeing eps {worst} deg')
    if over:
        idx = np.argwhere(~(needed <= limit))[:5]
        print('  first rays over the limit (ring, step, expected, measured):',
              [(int(r), int(s), round(float(nominal[r, s]), 3), round(float(got[r, s]), 3)) for r, s in idx])
        problems.append(f'{over} rays need more than {limit} deg')
    for problem in problems:
        print('PROBLEM:', problem)
    print('RESULT:', 'PASS' if not problems else 'FAIL')
    return 0 if not problems else 1


if __name__ == '__main__':
    sys.exit(main())
