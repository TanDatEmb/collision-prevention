# 0023. The Livox gyro for the rotation inside one frame

Status: Accepted. The owner chose this direction in chat on 2026-11-24 after the analysis below and
approved this record in chat on 2026-11-24 (issues #183 and #185). It amends ADR 0018 (deskew pose per ray), ADR 0019 (the Livox IMU is
not read) and ADR 0022 decision 2 (bounded wait for a pose). Values marked TBD stay TBD.

## Context
Leveling takes the pose of every ray from `VehicleOdometry` (ADR 0018, 0019; node wiring #137, merged
in #179). Known problems, each with its kind (evidence or estimate):
1. Missing pose. A ray needs FMU samples on both sides of its time, so the last rays of a frame need a
   sample after them. Gazebo SITL, bench values: rays were dropped in 0.3, 10.7, 9.9 and 2.3 % of frames
   in four sessions (evidence, #137); each drop breaks continuity and steps the cap (ADR 0021). ADR 0022
   decision 2 plans a bounded wait (TBD) before a frame is finalised: a mechanism and added latency on
   every frame.
2. Motion inside one frame. A Mid-360S frame spans about 100 ms: 1.2 m at 12 m/s and 17.6 deg at the
   176 deg/s hard-stop pitch rate (estimate from gains, ADR 0022). Every ray depends on the DDS pose
   stream (125 Hz in SITL; rate and latency on the target are OPEN-030).
3. Time offset between the FMU and lidar time stamps. The attitude error is `omega * dt`. Sources:
   the uXRCE-DDS timesync (read in the pinned PX4 `src/lib/timesync/Timesync.cpp`: an NTP-style offset
   that assumes equal delay both ways, accepts samples with a round trip below 10 ms and smooths them, so
   its bias is half the path asymmetry, up to 5 ms, and the protocol cannot see it); the EKF attitude
   lag (about 3.4 ms against ground truth in SITL, evidence, #156); PTP between the lidar and the Orin
   (ADR 0003, steady state not measured, #58).
4. The free-evidence over-claim `(r^2 / h) * dtheta` for any common attitude error (ADR 0022 item h).

Livox IMU facts (one Mid-360S, firmware 35.1.1.8, at rest; measured, ADR 0019): 200 samples per second
in the same packet format and time stamp as the points (`time_type`), steps of 4.1 to 6.0 ms (about
0.7 ms uncertain), gyro bias at most 0.345 deg/s (0.035 deg per 100 ms), range not readable on this
firmware. A bench capture right after the PTP lock (2026-11-20, notes outside the repository, not
repeated) saw the IMU time stamps step by about +0.5 s while the point times drifted by milliseconds; a
steady capture after the lock was not made. The rotation between the IMU axes and the point frame is not
in the Livox pages read; the FAST-LIO configuration for the Mid-360 (`config/mid360.yaml`) uses the
identity and a translation of (-0.011, -0.02329, 0.04412) m. That is a third-party file, not a vendor
specification.

ADR 0019 left the Livox IMU out of V1 but allowed it "for the rotation inside one frame only if the
measured rate or latency of `VehicleOdometry` proves too coarse". Problem 1 is that case, measured in
SITL, not on the target.

## Decision
1. Rotation at a ray time from the gyro. With `t_ref` the earliest accepted ray (unchanged, ICD-CP-06),
   the code today maps a sensor point as `p_ned = q(t_i) * F * M * p_sensor` (`ray_leveling.cpp`: `M`
   sensor FLU to body FLU, `F = diag(1, -1, -1)` body FLU to FRD, `q` body FRD to NED). The new rule is
   `p_ned = q_fmu(t_ref) * F * M * dR(t_ref, t_i) * p_sensor`, with `dR` the rotation of the sensor from
   `t_ref` to `t_i` in the sensor frame at `t_ref`, from the Livox gyro. The lever arm uses the same
   attitude, `q_fmu(t_ref) * F * M * dR * M^T * F`. The IMU axes are taken equal to the sensor axes until
   check 5a. Writing the rule without `F` flips the sign of pitch and yaw inside the frame.
2. Integration of `dR`. Trapezoid rule between consecutive IMU samples (exact for a rate that changes
   linearly; holding each rate over a step would cost `alpha * T * dt / 2`, 0.44 deg over 100 ms at
   1760 deg/s^2 and 5 ms steps). After the newest sample the newest rate is held for at most one IMU
   step, so a ray in the last 6 ms of a frame does not wait for the next sample; that tail costs
   `alpha * dt^2 / 2`, 0.03 deg at 1760 deg/s^2 over 6 ms (176 deg/s reached in 0.1 s, estimate). The
   bias is not estimated: it adds 0.035 deg over 100 ms. The 0.7 ms stamp uncertainty adds up to
   `omega * 0.7 ms` at each end of `dR`, 0.12 deg at 176 deg/s (estimate). A gap above the limit of
   item 4, or a time stamp going backwards, restarts the integration and increments the reset counter
   of the gyro samples, so a frame that spans it is refused by the reset check that exists today
   (`kResetInBracket`, `kResetSinceReference`). A ray with `t_i = t_ref` needs no gyro (`dR` is the
   identity).
3. Position at a ray time from the FMU velocity at `t_ref`: `x(t_i) = x(t_ref) + v(t_ref) * (t_i - t_ref)`.
   The origin offset of ADR 0022 decision 1 item d and its per-frame rise and fall bounds use this
   position and the attitude of item 1. Ignoring the acceleration over 100 ms costs `a * 0.1^2 / 2`:
   0.015 m at 3 m/s^2 and 0.048 m at 9.5 m/s^2 (44 deg tilt), vertically the same formula with the
   vertical acceleration. `poseFromOdometry` does not read velocity today; the `VehicleOdometry` velocity
   (with `velocity_frame` checked to be NED) or the slope of the two samples around `t_ref` is chosen in
   the implementation, and a new field is counted there.
4. No pose wait. ADR 0022 decision 2 (bounded wait) is withdrawn. The FMU pose is looked up only at
   `t_ref`. A frame is cut when the first ray at least one coverage window after the frame origin
   arrives (`frame_segmenter.cpp`; the window is the `coverage_window_ms` parameter, about 100 ms), so the
   FMU samples around `t_ref` have arrived when the odometry latency plus one odometry period is below the
   window minus `(t_ref - frame origin)` (uORB in SITL: p99 24 ms and 8 ms, OPEN-032; DDS on the
   target: OPEN-030). The gyro samples are kept in a second `PoseHistory` (orientation only, position
   zero) with its own depth and gap limit, platform values TBD; the gap limit must be above the 6.0 ms
   step measured. A frame without an FMU pose at `t_ref`, or a ray without gyro cover from `t_ref` to its
   time, gives `kNoPose` and breaks continuity, as today. No new status. The per-ray FMU reset check of
   ADR 0018 becomes a check at `t_ref`: rays are expressed relative to `t_ref`, so a reset later in the
   frame does not change them.
5. Checks before any evidence run uses this source (bench; nothing is written to the lidar):
   a. IMU axes against the point frame: the static acceleration against a fitted floor plane (roll and
      pitch), and a hand rotation about each axis (the owner moves the unit).
   b. Gyro range: compare the Livox gyro with a reference at the highest rate the bench reaches (the
      range is unreadable, so saturation is unknown).
   c. IMU and point time stamps on one base: a steady capture after the PTP lock, with the offset between
      the IMU and the point time stamps (the +0.5 s step above).
   d. End-to-end time offset: rotate the rigid lidar and FMU assembly and cross-correlate the Livox gyro
      with the rate derived from the `VehicleOdometry.q` samples, the attitude the leveler uses (as
      `tools/scenarios/analyze_attitude_time.py` does against ground truth in SITL). The PX4 angular
      velocity is not enough: it can be aligned while the attitude output lags its `timestamp_sample`,
      so it would miss the EKF lag. This measures problem 3 on the target chain (G5).
6. Simulation. Gazebo stamps a cloud once (ADR 0015), so every ray has `t_i = t_ref` and needs no gyro,
   and the cloud arrives about 12 ms after `t_ref` (evidence, #156), so the SITL drop of problem 1 remains.
   It is a simulator artefact: counted and reported, not fixed by a mechanism. A Gazebo IMU on the lidar
   link is optional, only to exercise the input. Unit tests with a time per ray check items 1 to 3,
   including the sign of pitch and yaw inside the frame.

Alternatives not chosen:
- Keep the FMU pose per ray with the bounded wait (ADR 0022 decision 2): keeps problems 1 and 2 and adds
  a wait to every frame.
- A complementary filter (the gyro for high frequencies, the FMU attitude low-pass): would reduce the
  time-offset term to the mean rate over the filter window, but adds a filter, a time constant and a
  gyro-bias error of about the bias times that constant. Not chosen until check 5d shows the offset
  matters.
- LiDAR-inertial odometry or SLAM: SYS-CP-010, a second pose estimate (ADR 0019), and it degenerates over
  flat ground or open sky at altitude.

## What this does not fix
- Problem 3 at `t_ref`. `q_fmu(t_ref)` carries `omega(t_ref) * dt` into every ray of the frame, the same
  size as today. It is limited by the sync gate of OPEN-033 (round trip) and measured by check 5d. No
  compensation of the EKF lag is decided here.
- Problem 4 and the loss of the horizon above 17 deg nose-up (ADR 0021). `dtheta_b` and `epsilon` stay
  TBD (#156); a cap on the certified free range from them is a separate decision.
- The heading of `cp_brake` (ADR 0019) is unchanged.

Error inside the frame, at the 176 deg/s estimate: about 0.035 deg (bias) + 0.03 deg (tail) + up to
0.24 deg (stamp uncertainty, both ends), against `omega * dt` per ray today, 0.6 deg with the 3.4 ms SITL
lag alone. These are estimates; the unit tests check the formulas, check 5 the inputs.

## Consequences
- ADR 0018: the rotation inside a frame comes from the gyro and the position from the FMU velocity at
  `t_ref`; the reset check moves to `t_ref`. ADR 0019: `livox_ray_source` reads the Livox IMU for this
  rotation only; it is never a heading or attitude source. ADR 0022 decision 2: withdrawn. OPEN-030
  shrinks to "odometry latency plus one period below the frame span". The docs-sync PR changes those
  texts after approval.
- The node of #179 changes: the leveler takes `dR` and the FMU pose at `t_ref`. New names (the IMU input,
  the gyro integration, a velocity field) are listed and counted in the implementation PR.
- The SDK adapter (#58) passes the IMU samples with their packet time.
