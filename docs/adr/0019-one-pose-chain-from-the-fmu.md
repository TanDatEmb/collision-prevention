# 0019. One pose chain from the FMU for heading and deskew

Amended by ADR 0023 (approved 2026-11-24): `livox_ray_source` reads the Livox IMU for the rotation inside
one frame only, under the reconsideration clause below; it is never a heading or attitude source. The FMU
stays the one pose chain at the reference time and for the heading of `cp_brake`.

## Context
ADR 0018 takes the deskew pose from `VehicleOdometry`. `cp_brake` also needs the heading at the
sample time of a map (ICD-CP-04, issue #94). Heading error is the yaw rate times the timing error,
about 1 deg per 11 ms at 90 deg/s, so this is the alignment that matters. The owner asked on
2026-11-20 whether the Livox IMU can supply the rotation and whether FAST-LIO2 or FAST-LIVO2 could
make the chain independent of the FMU.

Facts, measured on one Mid-360S (firmware app 35.1.1.8, lidar at rest, 2026-11-20) or read in
source:
- The lidar sends IMU samples (data type 0, gyro in rad/s and acceleration in g) at 200.0 packets
  per second, one sample each. Their time steps average 5.000 ms but are not even: two clusters of
  4.1 to 4.7 ms and 5.1 to 6.0 ms, standard deviation 0.73 ms, so one sample time is uncertain by
  about 0.7 ms.
- At rest over 30 s the gyro mean was 0.065, 0.073 and 0.345 deg/s on x, y and z, with a standard
  deviation of 0.06 to 0.10 deg/s. The z bias alone adds 0.035 deg in 100 ms and 3.4 deg in 10 s.
  One unit, one temperature: not a specification.
- The IMU range and rate cannot be read or set on this firmware (the lidar did not return key
  0x002B, return code 0x20).
- After the lidar locked to a PTP master (version 2.0 messages), its packets reported PTP time. Just
  after the lock its point times were about 18 ms behind the host clock and still moving: the steady
  state offset is not measured (#58).
- In the pinned PX4: `VehicleAttitude` has `q` (FRD to NED), `timestamp_sample` and
  `quat_reset_counter`; `VehicleLocalPosition` has `heading`, `heading_reset_counter` and
  `heading_good_for_control`; `VehicleOdometry` has `q`, `timestamp_sample` and `reset_counter`.
  `cp_brake` runs inside PX4 and reads these topics through uORB, with no DDS hop.
- The ray frame of ICD-CP-06 is gravity aligned, yaw only, at one reference time, the map timestamp.
- FAST-LIO2, FAST-LIVO2 and ikd-Tree are LiDAR-inertial odometry (IMU propagation, an iterated
  Kalman filter and a local map); GitHub lists them as GPL-2.0 and FAST-LIVO2 also needs a camera.
  SYS-CP-010 excludes SLAM and the license policy is open (OPEN-014).

## Decision
- One pose chain: the PX4 estimator. Rays are deskewed with `VehicleOdometry` as ADR 0018 says. The
  heading at the sample time comes from a timestamped history inside `cp_brake`, fed from the same
  estimator through uORB and keyed by `timestamp_sample`; a reset counter change invalidates
  interpolation across it, and `heading_good_for_control` gives its validity. Which topic carries
  the value and the depth of the history are chosen in #94 from SITL measurements (OPEN-032, a
  sentinel until then). No heading for the sample time means the map is not used: its sectors are
  unknown, never free, and the state machine treats it as safe-stop.
- Time stays as ADR 0003: lidar PTP time, Orin `CLOCK_REALTIME` on the same master, and the
  uXRCE-DDS timesync to PX4 time. The quality gate reads `timesync_status` with limits that are
  OPEN-033, measured at G5.
- The Livox IMU is not part of the pose chain in V1 and `livox_ray_source` does not read it. It
  gives no absolute orientation (the bias above reaches degrees in seconds), and its relative
  rotation over one frame would not remove the need for the FMU heading at the reference time. It
  can be reconsidered for the rotation inside one frame only if the measured rate or latency of
  `VehicleOdometry` (OPEN-030) proves too coarse.
- LiDAR-inertial odometry or SLAM (FAST-LIO2, FAST-LIVO2) is not used in V1. It would add a second
  pose estimate that can disagree with the FMU's, it conflicts with SYS-CP-010, and its license
  is unsettled (OPEN-014). Running it offline on recorded data as a cross-check is allowed if the
  owner wants it; it does not enter the product.

## Consequences
- #94 can start once the owner approves this record. #57 still waits for the origin-motion
  threshold (OPEN-031).
- There is no second source of heading: without a valid FMU heading maps are unusable and the
  vehicle is held by the safe-stop rule. This is the price of one pose chain.
- Open: the rate and latency of `VehicleOdometry` on the target network (OPEN-030), the history
  source and depth (OPEN-032), the sync limits (OPEN-033) and the steady-state PTP offset (#58).
