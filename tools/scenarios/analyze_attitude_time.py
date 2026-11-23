#!/usr/bin/env python3
"""Split the error of the leveling pose into attitude error and time offset, against ground truth (bench tool).

usage: analyze_attitude_time.py <run.npz> [--mount-xyz 0.12,0,0.38] [--mount-pitch-deg 10]
                                [--plane nx,ny,nz,c ...] [--max-clouds N]

<run.npz> is the output of record_attitude_run.py (its layout is described there). The ground plane z = 0 is always used; --plane adds a
plane n . p = c in world ENU (default: the vertical wall of spawn_wall.sh 8, x = 8, and the leaning wall of
spawn_lean_wall.sh 6 2.9 30). The mount is the sensor pose in the model frame (FLU, pitched down by the
given angle). All attitude angles are pitch up and roll (left side up) of the body, in degrees.

Four measurements, each printed with the numbers it rests on:
  1. EKF attitude minus ground truth at the same simulation stamp, in samples where the ground truth
     moves less than 2 deg/s: the static attitude error.
  2. The same difference against the ground-truth rate: the slope is a time offset of the EKF attitude
     (EKF at stamp t equals the truth at t + slope, slope negative = the attitude lags its stamp).
  3. For each cloud the rigid correction (rotation about the sensor, translation) that puts its points on
     the known planes, starting from the ground-truth pose at the cloud stamp. Its slope against the rate is
     the time offset of the cloud (render time minus header stamp); its static part is the lidar-implied
     attitude error of the ground truth itself (the check of the method).
  4. The end-to-end error of what the node does: EKF attitude at the cloud stamp minus the attitude the
     cloud was rendered at (ground truth plus correction), static, dynamic and against the rate; and
     the share of clouds for which no odometry sample at or after the stamp had arrived when the cloud did.
Linear interpolation of the 20 ms ground truth is exact enough for rates below about 150 deg/s (it
under-states the peak by under 0.01 deg at 0.7 Hz). Bench measurement of one simulator and one controller:
it gives no requirement value.
"""
import argparse
import math
import sys

import numpy as np

STATIC_RATE_DEG_S = 2.0
DYNAMIC_RATE_DEG_S = 20.0


def rotation_from_quaternion(q):
    """(N, 3, 3) rotation matrices from (N, 4) quaternions w, x, y, z (body -> world)."""
    w, x, y, z = q.T
    r = np.empty((len(q), 3, 3))
    r[:, 0, 0], r[:, 0, 1], r[:, 0, 2] = 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)
    r[:, 1, 0], r[:, 1, 1], r[:, 1, 2] = 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)
    r[:, 2, 0], r[:, 2, 1], r[:, 2, 2] = 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
    return r


def attitude_gt_deg(r):
    """Pitch up and roll of a ground-truth rotation (world ENU, body FLU)."""
    return (np.degrees(np.arcsin(np.clip(r[:, 2, 0], -1, 1))),
            np.degrees(np.arcsin(np.clip(r[:, 2, 1], -1, 1))))


def attitude_ekf_deg(r):
    """Pitch up and roll of an EKF rotation (world NED, body FRD): same angles as attitude_gt_deg."""
    return (np.degrees(np.arcsin(np.clip(-r[:, 2, 0], -1, 1))),
            np.degrees(np.arcsin(np.clip(r[:, 2, 1], -1, 1))))


def slope_fit(x, y):
    """Least-squares y = k x + b: (k, b, residual std)."""
    a = np.vstack([x, np.ones(len(x))]).T
    k, b = np.linalg.lstsq(a, y, rcond=None)[0]
    return k, b, float((y - (k * x + b)).std())


class GroundTruth:
    """Ground-truth pose and attitude rates on the 20 ms grid, interpolated linearly."""

    def __init__(self, gt):
        self.t = gt[:, 0]
        self.p = gt[:, 1:4]
        q = gt[:, 4:8]
        self.r = rotation_from_quaternion(q)
        self.q = q
        self.pitch, self.roll = attitude_gt_deg(self.r)
        dt = np.gradient(self.t / 1e9)
        self.pitch_rate, self.roll_rate = np.gradient(self.pitch) / dt, np.gradient(self.roll) / dt

    def interp(self, t_ns, series):
        return np.interp(t_ns, self.t, series)

    def pose(self, t_ns):
        i = int(np.searchsorted(self.t, t_ns))
        f = (t_ns - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        q = self.q[i - 1] * (1 - f) + self.q[i] * f
        q = q / np.linalg.norm(q)
        return self.p[i - 1] * (1 - f) + self.p[i] * f, rotation_from_quaternion(q[None])[0]

    def inside(self, t_ns, margin_ns=10**9):
        return (t_ns > self.t[0] + margin_ns) & (t_ns < self.t[-1] - margin_ns)


def ekf_vs_ground_truth(gt, odo):
    """Measurements 1 and 2. Returns a dict of the printed numbers."""
    t = odo[:, 0]
    keep = gt.inside(t)
    t = t[keep]
    pitch, roll = attitude_ekf_deg(rotation_from_quaternion(odo[keep, 5:9]))
    rate_p, rate_r = gt.interp(t, gt.pitch_rate), gt.interp(t, gt.roll_rate)
    err_p, err_r = pitch - gt.interp(t, gt.pitch), roll - gt.interp(t, gt.roll)
    static = (np.abs(rate_p) < STATIC_RATE_DEG_S) & (np.abs(rate_r) < STATIC_RATE_DEG_S)
    out = {'n': int(len(t)), 'n_static': int(static.sum())}
    for name, e in (('pitch', err_p), ('roll', err_r)):
        s = e[static]
        out['static_%s_mean' % name], out['static_%s_std' % name] = float(s.mean()), float(s.std())
        out['static_%s_p99' % name] = float(np.percentile(np.abs(s), 99))
    out['pitch_slope_ms'] = 1000 * slope_fit(rate_p, err_p)[0]
    out['roll_slope_ms'] = 1000 * slope_fit(rate_r, err_r)[0]
    # the lag by a scan, the same quantity without assuming the error is linear in the rate
    dyn = np.abs(rate_p) > DYNAMIC_RATE_DEG_S
    taus = np.arange(-30.0, 30.5, 0.5)
    rms = [np.sqrt(((pitch[dyn] - gt.interp(t[dyn] + tau * 1e6, gt.pitch)) ** 2).mean()) for tau in taus] \
        if dyn.any() else [float('nan')]
    out['pitch_scan_lag_ms'] = float(taus[int(np.argmin(rms))]) if dyn.any() else float('nan')
    out['dynamic_pitch_max_abs'] = float(np.abs(err_p[dyn]).max()) if dyn.any() else float('nan')
    out['gt_rate_max'] = float(max(np.abs(rate_p).max(), np.abs(rate_r).max()))
    return out


def assign_planes(points, planes, threshold_m=0.5):
    residual = np.stack([points @ n - c for n, c in planes], axis=1)
    k = np.argmin(np.abs(residual), axis=1)
    r = residual[np.arange(len(points)), k]
    return k, r, np.abs(r) < threshold_m


def fit_correction(world, origin, planes, iterations=3):
    """Small rotation omega (rad, about origin) and translation t (m) that put world on its planes."""
    omega, shift, moved = np.zeros(3), np.zeros(3), world.copy()
    for _ in range(iterations):
        k, r, ok = assign_planes(moved, planes)
        normals = np.array([planes[i][0] for i in k[ok]])
        q = moved[ok] - origin
        # n . (w x q) = w . (q x n)
        a = np.hstack([np.cross(q, normals), normals])
        x = np.linalg.lstsq(a, -r[ok], rcond=None)[0]
        moved = moved + np.cross(x[:3], moved - origin) + x[3:]
        omega, shift = omega + x[:3], shift + x[3:]
    k, r, ok = assign_planes(moved, planes)
    return omega, shift, float(np.sqrt((r[ok] ** 2).mean())), int(ok.sum())


def rotation_vector(omega):
    """Rotation matrix of the rotation vector omega (rad), Rodrigues' formula."""
    angle = float(np.linalg.norm(omega))
    if angle < 1e-12:
        return np.eye(3)
    k = omega / angle
    cross = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def rotation_y(angle_rad):
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def cloud_corrections(gt, meta, clouds, planes, mount_xyz, mount_pitch_rad, max_clouds=None, stride=3):
    """Measurement 3: per cloud (stamp, d_pitch, d_roll, d_yaw, rms, n, pitch_rate, roll_rate, pitch, roll),
    the correction in degrees (pitch up and left side up positive, as the attitude angles)."""
    mount = rotation_y(mount_pitch_rad)
    rows = []
    for i in range(len(clouds)):
        stamp = meta[i, 0]
        if not gt.inside(stamp):
            continue
        pts = clouds[i].astype(float)
        pts = pts[np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 0.05)][::stride]
        pos, rot = gt.pose(stamp)
        origin = pos + rot @ mount_xyz
        world = (rot @ mount @ pts.T).T + origin
        omega, _, rms, n = fit_correction(world, origin, planes)
        # the correction moves the assumed world points onto the planes, so the attitude the cloud was
        # rendered at is the correction applied to the ground-truth rotation; its pitch and roll are read
        # with the same angle definitions, which holds for any heading
        rendered = rotation_vector(omega) @ rot
        pitch_r, roll_r = attitude_gt_deg(rendered[None])
        pitch_t, roll_t = float(gt.interp(stamp, gt.pitch)), float(gt.interp(stamp, gt.roll))
        pitch_g, roll_g = attitude_gt_deg(rot[None])
        rows.append((stamp, float(pitch_r[0] - pitch_g[0]), float(roll_r[0] - roll_g[0]), math.degrees(omega[2]),
                     rms, n, float(gt.interp(stamp, gt.pitch_rate)), float(gt.interp(stamp, gt.roll_rate)),
                     pitch_t, roll_t))
        if max_clouds and len(rows) >= max_clouds:
            break
    return np.array(rows)


def end_to_end(rows, odo):
    """Measurement 4 (attitude part) from the rows of cloud_corrections."""
    pitch, roll = attitude_ekf_deg(rotation_from_quaternion(odo[:, 5:9]))
    stamp = rows[:, 0]
    d_pitch, d_roll, rate_p, rate_r = rows[:, 1], rows[:, 2], rows[:, 6], rows[:, 7]
    err_p = np.interp(stamp, odo[:, 0], pitch) - (rows[:, 8] + d_pitch)
    err_r = np.interp(stamp, odo[:, 0], roll) - (rows[:, 9] + d_roll)
    static = (np.abs(rate_p) < STATIC_RATE_DEG_S) & (np.abs(rate_r) < STATIC_RATE_DEG_S)
    dyn = np.abs(rate_p) > DYNAMIC_RATE_DEG_S
    out = {'n': int(len(rows)), 'n_static': int(static.sum()), 'n_dynamic': int(dyn.sum()),
           'plane_rms_median_m': float(np.median(rows[:, 4])),
           'cloud_pitch_slope_ms': 1000 * slope_fit(rate_p, d_pitch)[0],
           'cloud_roll_slope_ms': 1000 * slope_fit(rate_r, d_roll)[0],
           'cloud_static_pitch_mean': float(d_pitch[static].mean()),
           'cloud_static_pitch_std': float(d_pitch[static].std()),
           'cloud_static_roll_mean': float(d_roll[static].mean()),
           'cloud_static_roll_std': float(d_roll[static].std()),
           'e2e_static_pitch_p99': float(np.percentile(np.abs(err_p[static]), 99)),
           'e2e_static_roll_p99': float(np.percentile(np.abs(err_r[static]), 99)),
           'e2e_pitch_slope_ms': 1000 * slope_fit(rate_p, err_p)[0],
           'e2e_roll_slope_ms': 1000 * slope_fit(rate_r, err_r)[0]}
    if dyn.any():
        out['e2e_dynamic_pitch_p99'] = float(np.percentile(np.abs(err_p[dyn]), 99))
        out['e2e_dynamic_pitch_max'] = float(np.abs(err_p[dyn]).max())
    return out


def arrival_race(meta, odo):
    """Measurement 4 (timing part): clouds for which no odometry sample with stamp >= the cloud stamp had
    arrived when the cloud did (a leveling lookup at the stamp then has no sample after it). The order of
    arrival is the callback sequence number (meta column 2, odo column 9), not the simulation clock, which
    a callback only caches."""
    missing = 0
    for stamp, _, seq in meta[:, :3]:
        missing += not np.any((odo[:, 0] >= stamp) & (odo[:, 9] < seq))
    lag = (meta[:, 1] - meta[:, 0]) / 1e6
    latency = (odo[:, 1] - odo[:, 0]) / 1e6
    return {'clouds': int(len(meta)), 'missing': int(missing), 'missing_percent': 100.0 * missing / len(meta),
            'cloud_lag_ms_median': float(np.median(lag)), 'cloud_lag_ms_max': float(lag.max()),
            'odometry_latency_ms_median': float(np.median(latency)),
            'odometry_latency_ms_max': float(latency.max())}


def parse_plane(text):
    nx, ny, nz, c = (float(v) for v in text.split(','))
    n = np.array([nx, ny, nz])
    return n / np.linalg.norm(n), c / np.linalg.norm(n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--mount-xyz', default='0.12,0,0.38')
    ap.add_argument('--mount-pitch-deg', type=float, default=10.0)
    ap.add_argument('--plane', action='append')
    ap.add_argument('--max-clouds', type=int)
    args = ap.parse_args()
    data = np.load(args.run)
    gt, odo, meta, clouds = GroundTruth(data['gt']), data['odo'], data['meta'], data['clouds']
    kept = meta[data['cloud_index']]
    planes = [(np.array([0.0, 0.0, 1.0]), 0.0)]
    planes += [parse_plane(p) for p in (args.plane or ['1,0,0,8', '0,-0.8660254,0.5,-3.746'])]

    a = ekf_vs_ground_truth(gt, odo)
    print('1. EKF minus ground truth, same stamp (n=%d, static %d, max GT rate %.0f deg/s)'
          % (a['n'], a['n_static'], a['gt_rate_max']))
    print('   static pitch %.3f +- %.3f (p99 %.3f) deg, roll %.3f +- %.3f (p99 %.3f) deg'
          % (a['static_pitch_mean'], a['static_pitch_std'], a['static_pitch_p99'],
             a['static_roll_mean'], a['static_roll_std'], a['static_roll_p99']))
    print('2. time offset of the EKF attitude: slope pitch %.2f ms, roll %.2f ms, scan %.1f ms; '
          'largest dynamic pitch error %.3f deg' % (a['pitch_slope_ms'], a['roll_slope_ms'],
                                                    a['pitch_scan_lag_ms'], a['dynamic_pitch_max_abs']))
    rows = cloud_corrections(gt, kept, clouds, planes, np.array([float(v) for v in args.mount_xyz.split(',')]),
                             math.radians(args.mount_pitch_deg), args.max_clouds)
    b = end_to_end(rows, odo)
    print('3. cloud against ground truth at the header stamp (n=%d, plane rms %.3f m): time offset pitch %.2f ms, '
          'roll %.2f ms; static correction pitch %.3f +- %.3f, roll %.3f +- %.3f deg'
          % (b['n'], b['plane_rms_median_m'], b['cloud_pitch_slope_ms'], b['cloud_roll_slope_ms'],
             b['cloud_static_pitch_mean'], b['cloud_static_pitch_std'], b['cloud_static_roll_mean'],
             b['cloud_static_roll_std']))
    print('4. end to end (EKF at the cloud stamp minus the rendered attitude): static p99 pitch %.3f, roll %.3f deg; '
          'time slope pitch %.2f ms, roll %.2f ms; dynamic p99 pitch %s, max %s deg'
          % (b['e2e_static_pitch_p99'], b['e2e_static_roll_p99'], b['e2e_pitch_slope_ms'], b['e2e_roll_slope_ms'],
             '%.3f' % b['e2e_dynamic_pitch_p99'] if 'e2e_dynamic_pitch_p99' in b else 'n/a',
             '%.3f' % b['e2e_dynamic_pitch_max'] if 'e2e_dynamic_pitch_max' in b else 'n/a'))
    c = arrival_race(meta, odo)
    print('   arrival: %d of %d clouds (%.1f %%) had no odometry sample at or after the stamp when they arrived; '
          'cloud lag median %.0f max %.0f ms, odometry latency median %.0f max %.0f ms'
          % (c['missing'], c['clouds'], c['missing_percent'], c['cloud_lag_ms_median'], c['cloud_lag_ms_max'],
             c['odometry_latency_ms_median'], c['odometry_latency_ms_max']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
