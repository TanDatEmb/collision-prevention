#!/usr/bin/env python3
"""Self-test of analyze_attitude_time.py on synthetic data with known answers. It tests the arithmetic and
the sign conventions of the analyzer only; it says nothing about PX4 or Gazebo.
Run: python3 -I tools/scenarios/selftest_attitude_time.py (exit 0 when every case passes).

Cases: an EKF attitude that lags its stamp by 4 ms and has a 0.1 deg pitch bias (the analyzer must return a
-4 ms slope and scan lag and the bias); a rigid correction of rotated and shifted plane points (it must undo
the known rotation and shift); clouds ray-cast from an attitude 0.3 deg pitched and 0.2 deg rolled from the
ground truth (the correction must be +0.3 and +0.2 deg, the end-to-end error must be -0.3 and -0.2 deg for an
EKF that equals the truth); the arrival check on three hand-made clouds, one of them the same cached clock with a later callback; the cloud
attitude case uses a heading of 90 deg, where pitch and roll are not the world axes.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_attitude_time as an  # noqa: E402

FAILS = []


def check(name, value, expected, tolerance):
    ok = abs(value - expected) <= tolerance
    print('%s %-52s %9.4f (expected %.4f +- %.4f)' % ('ok  ' if ok else 'FAIL', name, value, expected, tolerance))
    if not ok:
        FAILS.append(name)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def body_flu_to_enu(pitch_up_deg, roll_deg, heading_rad=0.0):
    """Rotation with the attitude_gt_deg conventions: forward axis rises by pitch_up, left side by roll."""
    return rot_z(heading_rad) @ rot_y(-math.radians(pitch_up_deg)) @ rot_x(math.radians(roll_deg))


def quaternion(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    return np.array([w, (r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w)])


ENU_TO_NED = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])
FLU_TO_FRD = np.diag([1.0, -1, -1])


def motion(t_s):
    """Pitch up and roll (deg) of the synthetic vehicle at times t_s."""
    return 12 * np.sin(2 * np.pi * 0.5 * t_s), 8 * np.sin(2 * np.pi * 0.35 * t_s + 1)


def gt_rows(t_s):
    rows = []
    for t in t_s:
        p, r = motion(np.array([t]))
        q = quaternion(body_flu_to_enu(p[0], r[0]))
        rows.append((t * 1e9, 0.0, 0.0, 2.5, *q))
    return np.array(rows)


def odo_rows(t_s, lag_s, pitch_bias_deg):
    rows = []
    for t in t_s:
        p, r = motion(np.array([t - lag_s]))
        q = quaternion(ENU_TO_NED @ body_flu_to_enu(p[0] + pitch_bias_deg, r[0]) @ FLU_TO_FRD)
        rows.append((t * 1e9, t * 1e9 + 4e6, 0.0, 0.0, -2.5, *q, len(rows)))
    return np.array(rows)


def case_ekf_lag_and_bias():
    gt = an.GroundTruth(gt_rows(np.arange(0, 20, 0.02)))
    odo = odo_rows(np.arange(0.5, 19.5, 0.008), 0.004, 0.1)
    a = an.ekf_vs_ground_truth(gt, odo)
    check('time slope of a 4 ms lag (ms)', a['pitch_slope_ms'], -4.0, 0.3)
    check('time slope of the roll (ms)', a['roll_slope_ms'], -4.0, 0.3)
    check('scan lag (ms)', a['pitch_scan_lag_ms'], -4.0, 0.5)
    # the bias is seen only where the vehicle is almost still, so add a still stretch
    t = np.arange(0, 20, 0.02)
    still = np.concatenate([t, 20 + np.arange(0, 10, 0.02)])
    rows = gt_rows(still)
    rows[len(t):, 4:8] = rows[len(t) - 1, 4:8]
    gt2 = an.GroundTruth(rows)
    odo2 = odo_rows(np.arange(0.5, 29.5, 0.008), 0.0, 0.1)
    keep = odo2[:, 0] > 20e9
    odo2[keep, 5:9] = quaternion(ENU_TO_NED @ body_flu_to_enu(
        an.attitude_gt_deg(an.rotation_from_quaternion(rows[len(t) - 1:len(t), 4:8]))[0][0] + 0.1,
        an.attitude_gt_deg(an.rotation_from_quaternion(rows[len(t) - 1:len(t), 4:8]))[1][0]) @ FLU_TO_FRD)
    b = an.ekf_vs_ground_truth(gt2, odo2)
    check('static pitch bias (deg)', b['static_pitch_mean'], 0.1, 0.01)
    check('static roll bias (deg)', b['static_roll_mean'], 0.0, 0.01)


PLANES = [(np.array([0.0, 0, 1]), 0.0), (np.array([1.0, 0, 0]), 8.0), (np.array([0, -0.8660254, 0.5]), -3.746)]


def ray_directions():
    az = np.radians(np.arange(-180, 180, 1.0))
    el = np.radians(np.arange(-17, 42, 1.0))
    a, e = np.meshgrid(az, el)
    return np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], axis=-1).reshape(-1, 3)


def cast(dirs_world, origin):
    """Range along each ray to the nearest plane in front; inf where none."""
    best = np.full(len(dirs_world), np.inf)
    for n, c in PLANES:
        denom = dirs_world @ n
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (c - origin @ n) / denom
        t = np.where((denom != 0) & (t > 0.1), t, np.inf)
        best = np.minimum(best, t)
    return best


def case_rigid_correction():
    origin = np.array([2.0, 0.5, 2.9])
    dirs = ray_directions()
    rng = cast(dirs, origin)
    world = origin + dirs[np.isfinite(rng)] * rng[np.isfinite(rng)][:, None]
    omega0 = np.radians([0.4, -0.3, 0.2])
    shift0 = np.array([0.02, -0.03, 0.01])
    moved = world + np.cross(omega0, world - origin) + shift0
    omega, shift, rms, n = an.fit_correction(moved, origin, PLANES)
    check('recovered rotation x (deg)', math.degrees(omega[0]), -math.degrees(omega0[0]), 0.02)
    check('recovered rotation y (deg)', math.degrees(omega[1]), -math.degrees(omega0[1]), 0.02)
    check('recovered rotation z (deg)', math.degrees(omega[2]), -math.degrees(omega0[2]), 0.02)
    check('recovered shift z (m)', shift[2], -shift0[2], 0.003)
    check('plane rms after the fit (m)', rms, 0.0, 0.003)


def case_cloud_attitude():
    mount_xyz, mount_pitch = np.array([0.12, 0.0, 0.38]), math.radians(10)
    dirs = ray_directions()
    heading = math.pi / 2                         # not aligned with world +x: pitch and roll are not world axes
    gt_p, gt_r = 5.0, -4.0                       # truth attitude of the vehicle at the cloud stamp
    render_p, render_r = gt_p + 0.3, gt_r + 0.2  # the cloud was rendered at this attitude
    pos = np.array([1.0, 0.0, 2.5])
    t_s = np.arange(0, 6, 0.02)
    rows = np.array([(t * 1e9, *pos, *quaternion(body_flu_to_enu(gt_p, gt_r, heading))) for t in t_s])
    r_render = body_flu_to_enu(render_p, render_r, heading)
    sensor = r_render @ rot_y(mount_pitch)
    origin = pos + r_render @ mount_xyz
    rng = cast((sensor @ dirs.T).T, origin)
    with np.errstate(invalid='ignore'):
        pts = np.where(np.isfinite(rng)[:, None], dirs * rng[:, None], np.nan).astype(np.float32)
    meta = np.array([[3e9, 3.012e9, 1.0]])
    gt = an.GroundTruth(rows)
    got = an.cloud_corrections(gt, meta, pts[None], PLANES, mount_xyz, mount_pitch, stride=1)
    check('cloud correction pitch (deg)', got[0, 1], 0.3, 0.02)
    check('cloud correction roll (deg)', got[0, 2], 0.2, 0.02)
    odo = np.array([(t * 1e9, t * 1e9 + 4e6, 0, 0, -2.5,
                     *quaternion(ENU_TO_NED @ body_flu_to_enu(gt_p, gt_r, heading) @ FLU_TO_FRD), 0)
                    for t in np.arange(0, 6, 0.008)])
    e = an.end_to_end(got, odo)
    # an EKF equal to the truth is 0.3 / 0.2 deg away from the attitude the cloud was rendered at
    check('end-to-end static pitch p99 (deg)', e['e2e_static_pitch_p99'], 0.3, 0.02)
    check('end-to-end static roll p99 (deg)', e['e2e_static_roll_p99'], 0.2, 0.02)


def case_arrival():
    # columns used: stamp, arrival clock, ..., arrival sequence number (last of 10 for odometry, 3rd of 3 for clouds)
    def sample(stamp, seq):
        return [stamp, 110, 0, 0, 0, 1, 0, 0, 0, seq]
    odo = np.array([sample(100, 1), sample(108, 4)], dtype=float)
    # cloud 1: stamp 100, sample 100 arrived first (seq 1 < 2): covered. Cloud 2: stamp 105, needs a sample
    # >= 105: sample 108 has seq 4, after the cloud (seq 3) although both saw the same cached clock: missing.
    # Cloud 3: stamp 105 arrives after sample 108 (seq 5 > 4): covered.
    meta = np.array([[100, 110, 2], [105, 110, 3], [105, 130, 5]], dtype=float)
    check('clouds without a later sample (callback order)', an.arrival_race(meta, odo)['missing'], 1, 0)


def main():
    case_ekf_lag_and_bias()
    case_rigid_correction()
    case_cloud_attitude()
    case_arrival()
    print('FAILED: %s' % ', '.join(FAILS) if FAILS else 'all cases passed')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
