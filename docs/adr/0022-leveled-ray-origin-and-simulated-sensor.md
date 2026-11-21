# 0022. Leveled-ray origin, band reference and the simulated sensor

Status: Proposed. The owner agreed to the decisions below in chat on 2026-11-21; the approval is
recorded in a separate PR. Issue #143. It amends ADR 0020 D1 and D2 and ICD-CP-03 text; those texts
are changed by the docs-sync PR after approval, not here. Values marked TBD stay TBD.

## Context
- D2 computes `r_free = min(end * cos(el), h / tan|el|)` as if the ray started at the centre of the
  band. The lidar sits above the vehicle reference. The D1 transform shifts the origin by
  `x(t_i) - x(t_ref)`, and `RaySample` (time, range, azimuth, elevation, masked, return kind) has no
  origin field. A return is shifted exactly by re-expressing its end point (`RayLeveler::level` does
  it). A no-return has no end point, so it loses the shift.
- The merged leveler puts the origin at the body origin at `t_ref` and does not model the lever arm
  (`ray_leveling.hpp`).
- Bench mount as built in the SDF (Gazebo, X500). The "model frame" is the frame of the
  `x500_lidar_2d` model. `x500_base/model.sdf` has the model pose `0 0 .24`, so `base_link` is at z 0.24
  in the model frame. The lidar include pose `.12 0 .26` in `x500_lidar_2d/model.sdf` is also in the model
  frame (the fixed joint pose of -0.1 only places the joint frame and does not move the child link). So the
  lidar link is at (0.12, 0, 0.26 - 0.24) = (0.12, 0, 0.02) from `base_link`. The `gpu_lidar` sensor is
  0.055 above the link (`lidar_2d_v2/model.sdf`, sensor pose `0 0 0.055` in the link frame), so the sensor
  is at (0.12, 0, 0.075) from `base_link` (z 0.315 in the model frame). An earlier text of this ADR took
  the model-frame z (0.26 and 0.315) as heights from `base_link`; that was wrong. This is read from the SDF
  files and SDF semantics and agreed by two independent reviews; it is not checked by a static run (issue
  #153). With the 10 deg nose-down pitch of decision 4, the reviews found that the X500 propellers enter the
  lower field of view at this height (no propeller geometry is computed here).
- Pitch moves the lidar. With `z` the sensor height above `base_link`, its forward offset from the vehicle
  origin at a nose-up pitch `theta` is `0.12 cos(theta) - z sin(theta)` (m, horizontal). The analyzer uses
  the constant 0.12 (next item). Arithmetic (cos 10 deg = 0.98481, sin 10 deg = 0.17365; cos 17 deg =
  0.95630, sin 17 deg = 0.29237):

  | Nose-up pitch | Current SDF pose, z 0.075 m | Error vs 0.12 | Chosen pose, z 0.14 m | Error vs 0.12 |
  | --- | --- | --- | --- | --- |
  | 0 deg | 0.12000 | 0 | 0.12000 | 0 |
  | 10 deg | 0.11818 - 0.075 x 0.17365 = 0.11818 - 0.01302 = 0.10516 m | 0.015 m | 0.11818 - 0.14 x 0.17365 = 0.11818 - 0.02431 = 0.09387 m | 0.026 m |
  | 17 deg | 0.11476 - 0.075 x 0.29237 = 0.11476 - 0.02193 = 0.09283 m | 0.027 m | 0.11476 - 0.14 x 0.29237 = 0.11476 - 0.04093 = 0.07383 m | 0.046 m |

  The error is 0.12 minus the offset: the real lidar is nearer to the vehicle origin than the constant
  assumes. A closest approach measured from the lidar with the constant 0.12 therefore reads smaller than
  the true lidar-to-wall gap by that amount. This is the error of this term alone; the +-20 cm band the
  owner set for the bench (criterion a) is a separate criterion, not a budget for this term.
- The 1-D offset of the analyzer ignores the pitch term: `tools/scenarios/analyze_stop.py` subtracts one
  `--lidar-offset` that is constant along the leg whatever the pitch. The fix is issue #152.
- Owner choice for the X500 bench, chat, 2026-11-21 (approval recorded in a separate PR): the sensor at
  (0.12, 0, 0.14) m from `base_link`, rpy (0, +0.1745, 0) rad. In SDF a positive pitch tips +x toward -z,
  so the boresight points down (0.1745 rad = 10 deg). The lever arm from `base_link` is
  sqrt(0.12^2 + 0.14^2) = sqrt(0.0144 + 0.0196) = 0.184 m. For the planar bench model `lidar_2d_v2` at pitch
  0, the sensor 0.14 above `base_link` is 0.14 - 0.055 = 0.085 m for the link above `base_link`, so the
  include pose in the model frame is z = 0.24 + 0.085 = 0.325 (check: 0.325 + 0.055 - 0.24 = 0.14).

## Decision
### 1. Band reference: the lidar centre at `t_ref` (option a)
The band is taken around the lidar centre at `t_ref`, with `h_up` and `h_down` relative to the lidar.
The origin shift stays inside the transform. The rays that leave `RayLeveler` start from that origin,
so the builder and `RaySample` are unchanged.

Option (b), a band around the vehicle reference with a per-ray lever arm, is not chosen: it changes
`RaySample`, is more complex, and nothing shows it is needed. The lever-arm error under rotation is
at most 0.184 m x 0.307 rad = 0.057 m, at the 176 deg/s hard-stop pitch rate of decision 2 over a 0.1 s
window (176 deg/s x 0.1 s = 17.6 deg = 0.307 rad) [estimate: the rate is from gains and limits, not from
logs].

Consequences:
- This amends D2, which says the band is around the vehicle. The platform file carries band values
  relative to the lidar.
- Cap distances and `CPB_MARGIN` then refer to the lidar (0.12 m ahead of the vehicle origin on the
  bench). The sector bearing shifts by up to about `0.12 / r` (3.4 deg at 2 m). ICD-CP-04 states it.
- "Exact" holds only at `t_ref`. A no-return from a later ray is off by `dz / tan|el|`: a 0.1 m climb
  at 3 deg gives 1.9 m. This is the origin-motion threshold of OPEN-031.
- Coupling to state: `kOriginMotionExceeded` breaks continuity (`breaksFrameContinuity`), so a
  threshold below `v * window` (1.2 m at 12 m/s with a 0.1 s window) would break every frame at speed.

### 2. Missing pose: wait for a bounded time, do not hold the last pose
The leveler returns `kNoPose` for a ray with no pose, and `breaksFrameContinuity` tells the caller to
drop it and mark the frame discontinuous (ICD-CP-06; the node wiring is #137, not merged). One missing
pose then makes no sector clearable: a step to zero, the 0.3 s dwell, and the 2 m/s^2 rise (about
6 s to return to 12 m/s).

Rejected: hold the last pose. The error is about `omega * dt` per 50 ms [estimates from gains and
limits, not from logs]:

| Case | Rate | Error per 50 ms |
| --- | --- | --- |
| Pitch while limiting | not estimated | 1 to 1.5 deg |
| Pitch on a hard stop | about 176 deg/s (P 4 x 44 deg; `MC_PITCHRATE_MAX` caps at 220) | 6.5 to 11 deg |
| Yaw | 150 deg/s | about 7.5 deg |

A 6.5 deg error moves a return at 20 m by 2.3 m in height and can push an obstacle out of the band
into free evidence. Near the horizon `h / tan(el)` swings by tens of metres. A constant-rate
extrapolation is good for about 10 to 20 ms only.

Decision: `PoseHistory` already interpolates across gaps up to `max_bracket_gap_ns`. Before it
finalises a frame, the node waits for a pose newer than the last ray, for a bounded time (a required
parameter, no default, TBD). If none comes, the rays are dropped and continuity is broken as today.

Measure first: how often `kNoSampleAfter` fires at frame end (without a wait it may fire on every
frame), and the rate and p99 latency of `/fmu/out/vehicle_odometry` through DDS (`dds_topics.yaml`
has no `rate_limit`; OPEN-030). The only data is the uORB topic in Gazebo SITL (OPEN-032: 125 Hz,
`timestamp - timestamp_sample` p99 24 ms), not the DDS path or the target network.

### 3. Two yaw sources: keep both, add a contract test
The Orin leveler takes yaw from the `VehicleOdometry` quaternion; `cp_brake` takes it from
`vehicle_local_position.heading`. Both are the Euler yaw of the EKF2 quaternion (the leveler uses
`atan2` of the rotation entries; EKF2 sets `lpos.heading = Eulerf(q).psi()`). The reset counters
differ: `odom.reset_counter` is the sum of the quaternion, velocity (NE, D) and position (NE, D)
reset counts, so the Orin side also refuses across position and velocity resets, which is
conservative. `cp_brake` uses `heading_reset_counter` only. Both refuse to interpolate across a
reset. A contract test compares the two yaws on one recorded or synthetic stream.

### 4. Simulated sensor for evidence: a multi-ring lidar
A planar horizontal scan makes no leveling claim. A pitched plane loses the 0 deg ray that lets a
sector certify both halves, and leveling cannot return a short wall to a plane that has left it
(bench observation, not reproduced here: a 3 m wall at 2.5 m altitude gave a closest approach of
1.15 m against 2.23 m for a tall wall). Any leveling, ground or short-obstacle claim uses a
multi-ring lidar over the Mid-360S field (-7 to +52 deg from the boresight, ICD-CP-01) mounted
pitched down 10 deg (#138). One 3D sensor is preferred to several `LaserScan` planes.

Unverified, to check with one `gz topic` sample: the stock `ros_gz` `LaserScan` conversion copies
only the middle ring (from memory), so a multi-ring scan reaches ROS as a point cloud
(`PointCloud2`). A no-return has infinite or NaN xyz, so azimuth and elevation come from the ray
index, with the layout configured from the SDF and a self-check that each return's `atan2` matches
its index angle.

### 5. ICD-CP-03 text
ICD-CP-03 names frame 12 (`MAV_FRAME_BODY_FRD`). After leveling the sectors are horizontal and aligned
with the heading at the sample time; the frame constant is used for its yaw convention. The text is
corrected by the docs-sync PR.

## Open gap, not decided here
The builder counts no rays per half-band, so a confirmed obstacle can stay a valid distance when one
half lies outside the field of view (`finalize`: "no evidence never shortens a confirmed obstacle
distance"). It belongs to issue #148.

## Consequences
- ICD and requirement texts are amended by the docs-sync PR after approval.
- The multi-ring lidar (#138) is needed before any SITL result on leveling is valid evidence.
- The node wiring (#137) waits for this ADR and for two bench values from the owner: the origin-motion
  threshold of OPEN-031, which stays TBD until the owner decides (it must be above the 1.2 m that the
  vehicle travels in the 0.1 s window at 12 m/s, or every frame at speed breaks continuity; that bound
  does not make it conservative, because of the dz / tan|el| error above), and the mount, chosen by the owner in chat for the bench on 2026-11-21, with the approval
  recorded in a separate PR: the sensor at (0.12, 0, 0.14) m from `base_link`, rpy (0, +0.1745, 0) rad (boresight
  down), lever arm 0.184 m (numbers and frames in Context; for `lidar_2d_v2` at pitch 0 the include pose
  z is 0.325 in the model frame). The SDF values above are unverified by a static run (issue #153).

## To measure first
The odometry rate and latency; the angular rate and acceleration in the step logs; the `kNoSampleAfter`
rate; the gz cloud layout and no-hit encoding; the analyzer offset used in the proof (the 1-D mode of
`analyze_stop.py` subtracts `--lidar-offset` along the leg whatever the heading).

## Not decided yet
A per-ray lever arm, the fraction threshold for dropping a whole frame, an extrapolation model, and
Livox scan pattern fidelity.
