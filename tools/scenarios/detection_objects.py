#!/usr/bin/env python3
"""Thin and composite obstacles for the detection scenes (bench tool, VM-56).

usage: detection_objects.py <name>      spawn the object into the running Gazebo SITL world
       detection_objects.py --list      print the names and the primitive counts

Gazebo frame: x East, y North, z up. The leg flies East from the origin at the altitude ALTITUDE_M; every object
stands across the path near x = OBJECT_X_M. A primitive is ('box', centre, size, rpy) or
('cyl', centre, radius, length, rpy), rpy as in SDF (R = Rz Ry Rx); a trailing 'support' marks a part that holds the
object up and is left out of the ground truth of `analyze_detection.py` (the poles of the wire). The sizes are test
fixtures, not claimed obstacle classes: DET-01 and ADR 0024 decide what is claimed. A frame with no valid ray cannot be
encoded, so the world must hold something the lidar can hit (the ground does at the flight altitude).
"""
import math
import subprocess
import sys

import numpy as np

OBJECT_X_M = 50.0
ALTITUDE_M = 12.0


def lattice(leg_m, brace_m, half_m=1.5, z0_m=4.0, z1_m=20.0, panel_m=1.5):
    """A square lattice tower: four legs and, between z0 and z1, a horizontal and an X brace per panel and face."""
    parts = [('box', (OBJECT_X_M + sx * half_m, sy * half_m, 15.0), (leg_m, leg_m, 30.0), (0, 0, 0))
             for sx in (-1, 1) for sy in (-1, 1)]
    diagonal = math.hypot(2 * half_m, panel_m)
    tilt = math.atan2(panel_m, 2 * half_m)
    z = z0_m
    while z < z1_m - 1e-6:
        zc = z + panel_m / 2
        for sy in (-1, 1):  # the two faces along x
            parts.append(('box', (OBJECT_X_M, sy * half_m, z), (2 * half_m, brace_m, brace_m), (0, 0, 0)))
            parts.append(('box', (OBJECT_X_M, sy * half_m, zc), (diagonal, brace_m, brace_m), (0, -tilt, 0)))
            parts.append(('box', (OBJECT_X_M, sy * half_m, zc), (diagonal, brace_m, brace_m), (0, tilt, 0)))
        for sx in (-1, 1):  # the two faces across the path
            parts.append(('box', (OBJECT_X_M + sx * half_m, 0, z), (brace_m, 2 * half_m, brace_m), (0, 0, 0)))
            parts.append(('box', (OBJECT_X_M + sx * half_m, 0, zc), (brace_m, diagonal, brace_m), (tilt, 0, 0)))
            parts.append(('box', (OBJECT_X_M + sx * half_m, 0, zc), (brace_m, diagonal, brace_m), (-tilt, 0, 0)))
        z += panel_m
    return parts


X, H, D45 = OBJECT_X_M, ALTITUDE_M, math.radians(45)
OBJECTS = {
    'pole10': [('cyl', (X, 0, 15), 0.05, 30, (0, 0, 0))],             # vertical pole, 0.10 m diameter
    'pole20': [('cyl', (X, 0, 15), 0.10, 30, (0, 0, 0))],
    'pole40': [('cyl', (X, 0, 15), 0.20, 30, (0, 0, 0))],
    'hbar10': [('box', (X, 0, H), (0.1, 20, 0.1), (0, 0, 0))],        # horizontal bar across the path at flight height
    'hbar20': [('box', (X, 0, H), (0.2, 20, 0.2), (0, 0, 0))],
    'vdiag20': [('box', (X, 0, H), (0.2, 0.2, 34), (D45, 0, 0))],     # 45 deg bar in the vertical plane across the path
    'hdiag20': [('box', (X, 0, H), (0.2, 20, 0.2), (0, 0, D45))],     # horizontal bar 45 deg to the path
    'powerpole': [('cyl', (X, 0, 7.5), 0.15, 15, (0, 0, 0)),          # 0.30 m pole with a cross-arm at flight height
                  ('box', (X, 0, H + 0.1), (0.15, 3.0, 0.15), (0, 0, 0))],
    'wire': [('cyl', (X, 15, 9), 0.15, 18, (0, 0, 0), 'support'),     # 0.02 m wire across the path (non-claim, DET-01)
             ('cyl', (X, -15, 9), 0.15, 18, (0, 0, 0), 'support'),
             ('cyl', (X, 0, H), 0.01, 30, (math.radians(90), 0, 0))],
    'lattice': lattice(0.10, 0.05),                                   # 0.10 m legs, 0.05 m braces
}


def rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def surface_points(name, step_m=0.04, z_range_m=(10.5, 14.0), supports=False):
    """Surface points of the object (Gazebo frame) between the heights z_range_m, for the ground truth."""
    points = []
    for prim in OBJECTS[name]:
        if prim[-1] == 'support' and not supports:
            continue
        if prim[0] == 'box':
            _, centre, size, rpy = prim[:4]
            axes = [np.linspace(-s / 2, s / 2, max(2, int(s / step_m) + 1)) for s in size]
            grid = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
            on_face = np.zeros(len(grid), bool)
            for k in range(3):
                on_face |= np.isclose(np.abs(grid[:, k]), size[k] / 2)
            local = grid[on_face]
        else:
            _, centre, radius, length, rpy = prim[:5]
            theta = np.linspace(0, 2 * math.pi, max(8, int(2 * math.pi * radius / step_m)), endpoint=False)
            along = np.linspace(-length / 2, length / 2, int(length / step_m) + 1)
            t, z = np.meshgrid(theta, along)
            local = np.stack([radius * np.cos(t).ravel(), radius * np.sin(t).ravel(), z.ravel()], 1)
        world = local @ rotation(rpy).T + np.array(centre)
        points.append(world[(world[:, 2] >= z_range_m[0]) & (world[:, 2] <= z_range_m[1])])
    return np.vstack(points)


def sdf(name):
    """One static model holding every primitive as a collision and a visual."""
    elements = []
    for k, prim in enumerate(OBJECTS[name]):
        if prim[0] == 'box':
            _, centre, size, rpy = prim[:4]
            geometry = '<box><size>%g %g %g</size></box>' % tuple(size)
        else:
            _, centre, radius, length, rpy = prim[:5]
            geometry = '<cylinder><radius>%g</radius><length>%g</length></cylinder>' % (radius, length)
        pose = '%g %g %g %g %g %g' % (tuple(centre) + tuple(rpy))
        elements.append("<collision name='c%d'><pose>%s</pose><geometry>%s</geometry></collision>"
                        "<visual name='v%d'><pose>%s</pose><geometry>%s</geometry></visual>"
                        % (k, pose, geometry, k, pose, geometry))
    return ("<sdf version='1.9'><model name='%s'><static>true</static><link name='link'>%s</link></model></sdf>"
            % (name, ''.join(elements)))


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    if sys.argv[1] == '--list':
        for name, parts in OBJECTS.items():
            print('%-10s %3d primitives' % (name, len(parts)))
        return
    name = sys.argv[1]
    if name not in OBJECTS:
        raise SystemExit('unknown object %s; --list prints the names' % name)
    request = 'sdf: "%s"' % sdf(name).replace('"', '\\"')
    result = subprocess.run(['gz', 'service', '-s', '/world/default/create', '--reqtype', 'gz.msgs.EntityFactory',
                             '--reptype', 'gz.msgs.Boolean', '--timeout', '10000', '--req', request],
                            capture_output=True, text=True, check=False)
    if result.returncode != 0 or 'true' not in result.stdout:
        raise SystemExit('spawn of %s failed: %s %s' % (name, result.stdout.strip(), result.stderr.strip()[:200]))
    print('%s spawned (%d primitives)' % (name, len(OBJECTS[name])))


if __name__ == '__main__':
    main()
