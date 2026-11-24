# 0022. Leveled-ray origin, band reference and the simulated sensor

Status: Accepted. The owner agreed to the decisions below in chat on 2026-11-21 and approved this record in
chat on 2026-11-22 (issue #165). Issue #143; the origin-motion mechanism of decision 1 is issue #155 (owner
decision in chat, 2026-11-21). It amends ADR 0020 D1 and D2, the conservative filter of ADR 0018 and
ICD-CP-03 and ICD-CP-06 text; those texts are changed by the docs-sync PR after approval, not here.
Values marked TBD stay TBD.

## Context
- D2 computes `r_free = min(end * cos(el), h / tan|el|)` as if the ray started at the centre of the
  band. The lidar sits above the vehicle reference. The D1 transform shifts the origin by
  `x(t_i) - x(t_ref)`, and `RaySample` (time, range, azimuth, elevation, masked, return kind) has no
  origin field. A return is shifted exactly by re-expressing its end point (`RayLeveler::level` does
  it). A no-return has no end point, so it loses the shift.
- The merged leveler puts the origin at the body origin at `t_ref` and does not model the lever arm
  (`ray_leveling.hpp`). It also drops any ray, return or no-return, whose Euclidean origin motion
  `|x(t_i) - x(t_ref)|` exceeds `origin_motion_threshold_m` (`kOriginMotionExceeded`, `ray_leveling.cpp`),
  and a drop breaks frame continuity (`breaksFrameContinuity`). The builder computes the free part of a
  ray in `addFreeEvidence` as `min(end * cos(el), 40 m)` and, per half, `h / tan|el|`, from the ray
  direction as it leaves the leveler, with no origin.
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
  origin at a nose-up pitch `theta` is `0.12 cos(theta) - z sin(theta)` (m, horizontal). The proofs so far
  used the constant 0.12 (next item). Arithmetic (cos 10 deg = 0.98481, sin 10 deg = 0.17365; cos 17 deg =
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
- The earlier proofs ignored the pitch term: `tools/scenarios/analyze_stop.py` subtracted one
  `--lidar-offset` constant along the leg whatever the heading and pitch. Issue #152 (merged) projects the
  offset on the leg and adds the pitch term (`--lidar-offset-z`); those earlier results are also in the
  EKF frame, not the ground-truth frame (the analyzer now defaults to `--reference truth`).
- Owner choice for the X500 bench, chat, 2026-11-21 (approval recorded in a separate PR): the sensor at
  (0.12, 0, 0.14) m from `base_link`, rpy (0, +0.1745, 0) rad. In SDF a positive pitch tips +x toward -z,
  so the boresight points down (0.1745 rad = 10 deg). The lever arm from `base_link` is
  sqrt(0.12^2 + 0.14^2) = sqrt(0.0144 + 0.0196) = 0.184 m. For the planar bench model `lidar_2d_v2` at pitch
  0, the sensor 0.14 above `base_link` is 0.14 - 0.055 = 0.085 m for the link above `base_link`, so the
  include pose in the model frame is z = 0.24 + 0.085 = 0.325 (check: 0.325 + 0.055 - 0.24 = 0.14).

## Decision
### 1. Band reference and origin handling: the lidar centre at `t_ref`
Items b, c, e and h amended by ADR 0024 (approved 2026-11-24): the free distance no longer comes from where a ray
leaves the band, so the frame-level origin bounds and their guards are removed and the `(r^2 / h) dtheta`
over-claim no longer exists; the attitude error acts on the band test, bounded by the allowance. Items a and d
(returns shifted exactly, the lever arm) stand.

Band reference (option a, unchanged in substance). The band is taken around the lidar centre at `t_ref`,
with `h_up` and `h_down` relative to the lidar. With `L` the lidar position in the PX4 vehicle reference
frame, that is from `base_link` (0.075 m above `base_link` for the current X500 SDF, 0.14 m for the
owner's bench choice, numbers in Context), the lidar origin of ray `i` is `o_i = x_i + R_i * L` and the
reference origin is `o_ref = x_ref + R_ref * L`. The origin offset of ray `i` is `d_i = o_i - o_ref`, with
up component `dz_i` and horizontal component `dh_i`, in the gravity-aligned frame.

Option (b), a band around the vehicle reference with a per-ray lever arm, is not chosen: it is more
complex and nothing shows it is needed. The lever arm is not ignored: it enters through `d_i` (item d).
The term it adds is at most 0.184 m x 0.307 rad = 0.057 m under rotation, at the 176 deg/s hard-stop
pitch rate of decision 2 over a 0.1 s window (176 deg/s x 0.1 s = 17.6 deg = 0.307 rad) [estimate: the
rate is from gains and limits, not from logs].

Origin handling (owner decision in chat, 2026-11-21: the scalar origin-motion threshold is not the
mechanism). `t_ref` is the earliest ray of the frame, which is the map timestamp of ICD-CP-06 (the
builder takes the minimum over the accepted rays in `addRay`); the node must pass that same time to
`beginFrame`.

a. Returns are never dropped for origin motion. `RayLeveler::level` adds `x(t_i) - x(t_ref)` to the end
   point, so a return is exact at every `t_i` (up to the pose error), not only at `t_ref`. The merged
   drop on the Euclidean norm of `x(t_i) - x(t_ref)` against `origin_motion_threshold_m` is removed. It
   is wrong for returns, and it mixes the horizontal part, which the shift handles exactly, with the
   vertical part, which is the harmful one for free evidence. The one remaining drop of a return in the
   transform is geometric, not a threshold: the shifted end point lies on the reference origin, the
   range is 0, and the ray has no direction (`ray_leveling.cpp`); it breaks continuity as today.

b. Free evidence is corrected where the builder computes `r_free` (`addFreeEvidence`), for no-returns
   and for the free part of returns (a return outside the band or beyond 40 m). For the half `s`
   (`s = +1` the up half with `h_up`, `s = -1` the down half with `h_down`):

       r_free_ref = min(end * cos(el), 40 m, (h_s - dz_b,s) / tan|el|)   with   end = 40 m for a no-return

   and the return range for a return, as in the builder today. `dz_b,s = max over the rays of the frame
   of max(s * dz_i, 0)` is a conservative per-frame bound of the vertical origin motion towards the
   boundary of half `s`, with respect to the reference ray (the earliest ray). It is not negative.
   An origin raised by `dz_i` leaves `h_up - dz_i` of room above it; an origin lowered by `|dz_i|` leaves
   `h_down - |dz_i|` below it. The horizontal term `min(0, dh.u)`,
   with `u` the ray's horizontal direction, is not added: the owner judged it nearly inert with `t_ref`
   the earliest ray. Its size is bounded by `|dh|` (1.2 m at 12 m/s over 0.1 s) and it only concerns rays
   with a component against the motion; the over-claim in a rear sector is not quantified here.

c. Three guards, derived and not thresholds:
   - A numerator `h_s - dz_b,s <= 0` (the origin is outside the band of half `s`) gives `r_free_ref = 0`.
     It is tested before the division.
   - `el = 0` with a numerator of 0 must not produce NaN (0 / 0). The builder pushes the values into
     `std::nth_element` with `std::greater` (`nthLargestOrZero`), and a NaN breaks the strict weak ordering.
     The numerator test above comes first; for a numerator above 0, `el = 0` gives infinity and the
     `min` with the cap applies, as the builder does today.
   - The formula uses the real ray direction `P - o_i` and the half it points into, not the chord
     elevation from `o_ref` to `P` that is left after leveling. Counterexample, `o = (0, 0, 0.3)`,
     `P = (10, 0, 1.1)`, `h_up = 1`: the code today gives `1 / 0.11 = 9.09 m`, the formula with the chord
     gives `(1 - 0.3) / 0.11 = 6.36 m`, the true exit of the band is `(1 - 0.3) / 0.08 = 8.75 m`. This is an
     implementation requirement. A no-return already carries the real direction after leveling. For a
     return the builder sees only the chord, and how it gets the real direction without a field in
     `RaySample` is not decided (see "Not decided yet"). A random check of the vertical part only (the
     chord form against the true exit, origin offset up to 0.6 m, `h` 0.3 to 2 m) found the chord form
     never above the true exit, so the chord costs free distance and does not over-claim; the horizontal
     term of item b is excluded from that check.

d. The lever arm is added to the origin offset: `d_i = x_i + R_i * L - x_ref - R_ref * L`. The merged
   `ray_leveling.hpp` says it is not modelled; that text is changed with the implementation. Frame trap:
   `L` is from `base_link`; the Gazebo ground truth is the model origin, 0.24 m below `base_link`
   (`x500_base` model pose z 0.24). The analyzer pitch term (`--lidar-offset-z`) with `--reference truth`
   is therefore 0.315 m for the current SDF (0.075 + 0.24) and 0.38 m for the owner's bench choice
   (0.14 + 0.24).

e. No field is added to `RaySample`. The builder receives the frame-level bound per half (the two numbers
   `dz_b,up` and `dz_b,down`, in metres, not negative). The leveling step computes them over the frame's
   rays while the frame is buffered, and they travel with the continuity flag: ICD-CP-06 buffers the whole
   frame before `reset(continuity)`, `addRay` and `finalize`, so they are known before the first
   `addRay`. This touches the ICD-CP-06 contract by one frame field. The owner approves the contract; the
   names are listed for approval in the implementation PR and no name or value is invented here. A frame
   without the bound is invalid; there is no default.

f. "Drop without breaking continuity" is rejected as unsafe: dropping a return that is inside the band
   can hide an obstacle (`ray_leveling.hpp`, `breaksFrameContinuity`). Only a ray that is free evidence
   could be dropped silently. Any drop limit on pose uncertainty (a tilt variance or `eph` of the
   odometry) is a new safety threshold: TBD, owner.

g. Nothing is claimed about climbs. The vertical travel after the sample, `v_z * age` (0.45 to 1.5 m), is
   corrected nowhere, and a climb with horizontal motion is OS-18, outside the ODD.

h. The attitude and mount error term is not absorbed by the formula. For a common attitude or mount error
   `dtheta` the over-claim of the certified free distance is about `(r^2 / h) * dtheta / (1 - r * dtheta / h)`
   in distance at true free distance `r`, and the `n_free`-th largest rule does not filter a common
   error. Illustration only (exact tangent, with the claim at the 40 m cap; the true free distance is then
   37.4, 34.1, 29.6 and 23.5 m for `h` = 1 m):

   | `dtheta` | 0.1 deg | 0.25 deg | 0.5 deg | 1.0 deg |
   | --- | --- | --- | --- | --- |
   | over-claim, `h` = 1 m | 2.6 m | 5.9 m | 10.4 m | 16.5 m |
   | over-claim, `h` = 0.5 m | 4.9 m | 10.4 m | 16.4 m | 23.3 m |

   The owner decides the attitude allowance `dtheta_b` and the error budget `epsilon` (the allowed
   over-claim of a certified free distance at the range where the cap binds, about 30 m at 12 m/s, near
   the 28.5 m braking distance of the architecture) after measurement (issue #156). `h_up`, `h_down` and
   `n_free` stay TBD. ICD-CP-01 gives 0.5 deg mount repeatability as a requirement, not a measurement.
   The only tilt data: the EKF against the simulator ground truth in SITL, p99 0.4 to 0.66 deg below
   10 deg/s body rate and 2.0 to 3.2 deg (maximum 3.7 deg) above it; SITL only, timing artefacts not
   excluded, not the flight EKF.

i. Measured vertical origin motion, 81 SITL runs, airborne with horizontal speed above 2 m/s: `|dz|` over
   100 ms has p99 0.045 m and maximum 0.068 m; `|dh|` has maximum 1.26 m. The frame-level bound therefore
   costs about 0.07 m of band height in the ODD. SITL only (analysis supplied by the owner in chat; the
   logs are not in this repository and not re-run here).

j. Availability. With a climb of 0 the correction changes nothing. At a 3 m/s climb over the 0.1 s window
   the origin rises 0.3 m, so the numerator of the up half falls to `h_up - 0.3`; whether `n_free` rays
   remain in the up half cannot be answered without ray data (the multi-ring lidar, built in PR #162). The old drop
   with a threshold below 1.2 m would make every frame that spans the window unusable at 12 m/s
   (12 m/s x 0.1 s = 1.2 m; `kOriginMotionExceeded` breaks continuity).

Consequences:
- This amends D2, which says the band is around the vehicle. The platform file carries band values
  relative to the lidar.
- Cap distances and `CPB_MARGIN` then refer to the lidar (0.12 m ahead of the vehicle origin on the
  bench). The sector bearing shifts by up to about `0.12 / r` (3.4 deg at 2 m). ICD-CP-04 states it.
- Returns are exact at every `t_i`. The no-return error without the correction is one-sided (it
  over-claims) and equals `dz / tan|el|`, which is at most `r * dz / h` for the certified distance `r`
  and at most `40 * dz / h` at the cap. The earlier example of a 0.1 m climb at 3 deg giving 1.9 m is
  `0.1 / tan(3 deg)`, independent of `h`; it is a 10% error of the certified distance only for `h` = 1 m
  (`r` = 19 m), and 20% for `h` = 0.5 m. With the frame-level bound the vertical part of this error is
  removed for the rays of the frame, at the cost of item i.
- The implementation removes `origin_motion_threshold_m` from `LevelingConfig` and with it the coupling of
  a threshold to `breaksFrameContinuity`: the drops that break continuity are then no reference, no pose,
  a reset since the reference and the degenerate return of item a.
- The leveler, the builder and the ICD-CP-06 text change together in the implementation work (the node
  wiring #137 is its first user); this record changes none of them. Implemented in issue #177: the threshold
  and its drop are removed (the status `kOriginMotionExceeded` is renamed `kDegenerateReturn` because only the
  degenerate return of item a is left), `LevelingConfig::lever_arm_flu_m` and the origin offset of item d,
  `RayFrame::origin_rise_bound_m` and `origin_fall_bound_m` (default NaN, "not provided", for a source that does not
  level, so a bench source keeps its behaviour; 0 is a real bound, as in PX4 setpoints), `SectorMapBuilder::reset(continuity, rise, fall)`, and the decorator
  `LevelingRaySource`. The real direction of a return stays the chord form (item c, never above the true exit).

### 2. Missing pose: wait for a bounded time, do not hold the last pose
Withdrawn by ADR 0023 (approved 2026-11-24): the FMU pose is looked up only at the reference time, which
has arrived before a real frame ends, and the rotation inside the frame comes from the Livox gyro, so no
wait is built. The text below is kept as the record of the rejected options (hold the last pose).

The leveler returns `kNoPose` for a ray with no pose, and `breaksFrameContinuity` tells the caller to
drop it and mark the frame discontinuous (ICD-CP-06; the node wiring is #137, not merged). One missing
pose then makes no sector clearable: a step to zero, the 0.3 s dwell, and the 2 m/s^2 rise (about
6 s to return to 12 m/s). After decision 1 the drops that break continuity are no reference, no pose,
a reset since the reference and the degenerate return of item a; none is an origin-motion threshold.
The frame-level bound of item e also needs a pose at every ray
time, so the bounded wait below serves it too.

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

Implementation state (issue #137): the node has no wait yet. It counts what the decorator drops
(`LevelingRaySource::raysDropped()`) and what the odometry input rejects, and logs them every 5 s, so
that the first leveled SITL run gives the measurement below. The wait is added only if that run shows the need. The pose time is
`timestamp_sample` in microseconds scaled to nanoseconds, with no clock mapping, as ADR 0003 puts PX4 and the rays in one time
base in both profiles; the first run checks it against the node clock.

Measure first: how often `kNoSampleAfter` fires at frame end (without a wait it may fire on every
frame), and the rate and p99 latency of `/fmu/out/vehicle_odometry` through DDS (`dds_topics.yaml`
has no `rate_limit`; OPEN-030). The only data is the uORB topic in Gazebo SITL (OPEN-032: 125 Hz,
`timestamp - timestamp_sample` p99 24 ms), not the DDS path or the target network. Any limit on pose
uncertainty that would drop a ray (a tilt variance or `eph`) is a new safety threshold: TBD, owner
(decision 1, item f).

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

Checked (issue #138, PR #162): the stock `ros_gz` `LaserScan` conversion keeps one ring (720 ranges of the
43200 of the Gazebo message), so a multi-ring scan reaches ROS as a point cloud (`PointCloud2`, organised
720 x 60, ring-major, the `ring` field always 0, a no-hit point +-inf in x, y and z, no NaN seen). A no-return has infinite or NaN xyz, so azimuth and elevation come from the ray
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
- ICD and requirement texts, including ICD-CP-06 (one frame-level field, decision 1 item e), are amended by
  the docs-sync PR after approval.
- The multi-ring lidar (built in PR #162) is needed before any SITL result on leveling is valid evidence, and
  before the availability question of decision 1 item j can be answered; the node does not level yet (#137), so
  no such result exists.
- The node wiring (#137) waits for this ADR, for the owner's approval of the ICD-CP-06 frame field, for
  the owner's values (below) and for the mount. There is no scalar origin-motion threshold any more; the
  values the owner still decides are `epsilon` and `dtheta_b` (after the measurement of issue #156),
  `h_up`, `h_down` and `n_free`, and a pose-uncertainty drop limit if one is wanted. All stay TBD. The
  mount was chosen by the owner in chat for the bench on 2026-11-21, with the approval recorded in a
  separate PR: the sensor at (0.12, 0, 0.14) m from `base_link`, rpy (0, +0.1745, 0) rad (boresight
  down), lever arm 0.184 m (numbers and frames in Context; for `lidar_2d_v2` at pitch 0 the include pose
  z is 0.325 in the model frame). The SDF values above were confirmed by a static run in Gazebo (issue #153, 2026-11-21).
- The attitude and mount error is not covered by the origin correction (decision 1 item h): until the
  owner sets `dtheta_b` and `epsilon` a certified free distance carries that over-claim, and a climb
  with horizontal motion carries no claim (item g).

## To measure first
The odometry rate and latency; the angular rate and acceleration in the step logs; the `kNoSampleAfter`
rate; the gz cloud layout and no-hit encoding; the analyzer offset used in the earlier proofs (a constant
`--lidar-offset` along the leg, before issue #152); the attitude and mount error against the ground truth
(issue #156); the vertical origin motion per frame on a flight log (the 81 SITL runs of decision 1 item i
are SITL only); how many rays per half remain at a climb (needs ray data, available since PR #162).

## Not decided yet
The fraction threshold for dropping a whole frame, an extrapolation model, and Livox scan pattern
fidelity. For the origin-motion mechanism of decision 1:
- `epsilon`, `dtheta_b`, `h_up`, `h_down`, `n_free` and any pose-uncertainty drop limit (TBD, owner; no
  value is proposed here).
- The names of the ICD-CP-06 frame field (the two bounds), listed in the implementation PR.
- How the builder gets the real ray direction of a return without a field in `RaySample` (item c), and
  whether the chord is acceptable in the meantime.
- The horizontal term `min(0, dh.u)` (item b): not added by the owner's judgement; the rear-sector
  over-claim is not quantified.
- Vertical travel after the sample and climbs (item g): not corrected, OS-18, outside the ODD.
