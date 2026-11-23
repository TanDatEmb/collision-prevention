# 0021. Braking tilt bound and the controller baseline

Status: Accepted. The owner agreed to decisions 1 to 4 below in chat on 2026-11-21, and to the revised
definition of decision 5 the same day, and approved this record in chat on 2026-11-22 (issue #165). Bench
values only: no value here is approved for a vehicle. Issues #142 and #154.
The IDs it uses (SYS-CP-012, 018, 027, ICD-CP-04, OPEN-003, 004, 039, 043) are defined in
`docs/architecture.md`.

## Context
Measured in Gazebo SITL, X500, planar lidar, forward flight with the nose along the leg, hook on, a tall
wall. The records are bench logs outside the repository (bench values). A claim carries its kind:
"evidence" is a measured number from those logs; "reasoning" is derived from them or from the source.

**Reference frame (owner rule, chat 2026-11-21).** In SITL the reference is ground truth (position and
velocity), never the EKF, because the EKF is part of the system under test. The analyzer defaults to
`--reference truth` since PR #158. Every number below is in the truth frame unless it is marked "EKF frame,
not evidence". The EKF frame overstates the closest approach by 0.36 to 0.48 m per run (cell means 0.39 to
0.46 m, 18 runs of the campaign below). Every closest-approach number of the earlier text of this ADR
(2.44 m, 2.29 to 2.30 m, 2.28 to 2.43 m, 1.60 m) was in the EKF frame; so was the reading "1.94 m is
accepted" of the acceptance band. Numbers of earlier runs that carry no frame in the records (cap excess,
rise, overshoot, settling, ramp lag of the defaults) are read as EKF frame, informational, until repeated.
Frame trap, stated once: the Gazebo ground truth is the origin of the model, which is 0.24 m below
`base_link`; the sensor is 0.075 m above `base_link` and 0.315 m above the model origin (ADR 0022, read from
the SDF files). The analyzer pitch term (`--lidar-offset-z`) uses 0.315 m for the current model with
`--reference truth`. Evidence for that value: the wire range minus the geometric slant range of a sensor at
0.12 m ahead and 0.315 m above the truth origin has mean -0.006 m and rms 0.007 m over 2679 samples, flat
across the pitch bins; with 0.075 m the rms is 0.014 m and the difference grows with pitch (+0.041 m at 11
to 15 deg).

**The vehicle runs above the velocity cap while braking.** The excess is 1.44 m/s at 12 m/s and 1.07 m/s at
5 m/s with the PX4 defaults (`MPC_XY_VEL_P_ACC` 1.8, I 0.4, D 0.2; EKF frame, not evidence). The PX4 code
computes `acc_sp = ff + Kp*e + I - Kd*a_measured` (`PositionControl.cpp`; the derivative acts on the
measured, low-passed acceleration), and the hook clears the horizontal feed-forward. The predicted
ramp-following lag is about (1+Kd)/Kp: 0.67 s predicted and 0.61 s measured with the defaults (EKF frame, not
evidence), 0.45 s predicted with the chosen set below. Measured with the chosen set in the truth frame,
hook-on braking ramps of 1 to 3 m/s^2: tau_r is 0.475 s at 5 m/s and 0.499 s at 12 m/s (evidence; 18 runs, per
run 0.47 to 0.505 s). The EKF figure is 0.43 s and 0.39 s, so truth minus EKF is 0.05 s and 0.11 s.

**Integrator.** The PX4 horizontal anti-windup runs only when the produced acceleration is below the
commanded one (tilt or thrust saturation). Without saturation the integrator integrates a real
tracking error and acts like the missing feed-forward during a ramp. With I = 0 the rollback after
the stop disappears, but the vehicle creeps to 1.60 m from the wall (EKF frame, not evidence).

**Attitude gains.** `MC_ROLL_P` and `MC_PITCH_P` from 3.0 to 6.5 do not change the velocity response
(rise 1.08 to 1.12 s) and raise the pitch-rate power above 2 Hz from 0.7 to 5.4 percent, so the
default 4.0 stays. `MPC_XY_P` 0.6, 0.95 and 1.3 gave identical stop results in Mission mode with the
hook limiting (EKF frame, not evidence).

**Velocity loop with set A** (P 3.0, D 0.35, I 0.4): rise 10 to 90 percent 0.95 s, overshoot 2.1
percent, no ringing, settling 3.5 s, pitch-rate power above 2 Hz 4.5 percent (earlier runs, frame not
recorded). Earlier closed loop (hook on, forward, tall wall, `CPB_T_LAT` 0.95 s, two repeats each):

| | Default | Set A |
| --- | --- | --- |
| Cap excess at 12 m/s | 1.44 m/s | 0.98 m/s |
| Cap excess at 5 m/s | 1.07 m/s | 0.74 m/s |
| Reversal after the stop | one | none |

Gains x1.5 (P 4.5, D 0.525) and x2 (P 6.0, D 0.7) stayed clean in those runs, with no reversals (closest
2.28 to 2.43 m, EKF frame, not evidence). The effective gain is about Kp/(1+Kd): P 6 with D 0.2 gives 5.0, P 8
with D 0.2 gives 6.7, P 6 with D 0.7 gives 3.5. The earlier finding "P 6 is worse and P 8 fails" is
therefore explained by the effective gain, not necessarily by damping. The test with P 8.5 and D 1.4
(also 3.5) was run and is invalid: pitch chatter 38.5 percent of the pitch-rate power, ramp lag 0.25 s,
no settle. It does not separate gain from damping.

**Campaign in the truth frame (evidence).** Set A, `CPB_T_LAT` 0.5, 0.7 and 0.95 s, 5 and 12 m/s, 3 valid
runs per cell (one run of 5 m/s with 0.5 s did not start the leg and was repeated), nose along the leg, tall
wall at 125 m, hook on. Closest approach, mean and min to max, the margin being 2.0 m:

| Speed | `CPB_T_LAT` | Closest (truth) | Closest (EKF frame, not evidence) | Sign changes (truth) | Rest minus closest (truth) |
| --- | --- | --- | --- | --- | --- |
| 5 m/s | 0.50 s | 1.58 (1.53 to 1.62) | 1.98 | 0 | 0.04 m |
| 5 m/s | 0.70 s | 1.76 (1.75 to 1.78) | 2.18 | 0 | 0.05 m |
| 5 m/s | 0.95 s | 1.90 (1.89 to 1.91) | 2.31 | 0 | 0.03 m |
| 12 m/s | 0.50 s | 1.63 (1.62 to 1.64) | 2.09 | 1 | 0.62 m |
| 12 m/s | 0.70 s | 1.86 (1.86 to 1.87) | 2.29 | 1 | 0.39 m |
| 12 m/s | 0.95 s | 2.03 (2.02 to 2.05) | 2.42 | 0 | 0.01 m |

Against the margin 2.0 m alone only the three 12 m/s runs with 0.95 s pass. Acceptance band on the bench
(owner decision, chat 2026-11-21): closest approach at least the margin minus 0.20 m, so 1.80 m, with at most
one sign change and the vehicle at rest before the wall. The cells that meet it are 5 m/s 0.95 s and 12 m/s
0.70 s and 0.95 s; 0.95 s is the smallest tested value that meets it at both speeds (reasoning from the table).

**Why the offline model predicted about 2.0 m (evidence and reasoning).** The model has the plant of the
velocity loop (dead time 0.02 s, lag 0.24 s), the cap law as coded, the 0.1 s map with its age and the 50 ms
evaluation hold. It predicts a closest approach of 1.99 to 2.05 m in all six cells above, while the truth
shows 0.41 m less at 0.5 s. Two effects are missing from it, each with its measured value:
- (a) Velocity-estimate error. The velocity loop and the hook close on the EKF velocity. During braking it
  reads below the truth velocity: by 0.11 to 0.13 m/s at 5 m/s and 0.22 to 0.27 m/s at 12 m/s (cell means
  0.109 to 0.126 and 0.219 to 0.271). The difference delta = truth minus EKF velocity is described by
  d(delta)/dt = (G*(-a_truth) - delta)/tau with G 0.135 s (per run 0.125 to 0.145) and tau 1.8 s at 5 m/s
  and 1.3 s at 12 m/s (per run 1.70 to 1.98 and 1.25 to 1.33). Once the wire range reaches the margin the cap
  is 0 and there is no position term (`speedCap` returns 0 when the range is not above the margin), so the
  vehicle creeps on: 0.34 m (5 m/s) and 0.24 m (12 m/s) after the cap first reaches 0 with 0.5 s; 0.22 and
  0.10 m with 0.7 s; 0.10 m and 0.01 m with 0.95 s (evidence). With this effect alone the model closes 62
  percent (5 m/s) and 73 percent (12 m/s) of its deficit at 0.5 s.
- (b) Lidar pitch. The planar lidar is fixed to the body, so the range reads high when the nose is up. The
  wire range minus the truth gap averages +0.058 m at 5 to 8 deg, +0.122 m at 8 to 11 deg and +0.157 m at
  11 to 14 deg (2679 samples), and equals the slant geometry within 0.007 m rms. At the instant the cap
  reaches 0 the pitch is 7.4 and 9.0 deg with 0.5 s and the range bias is 0.058 and 0.075 m (5 and 12 m/s);
  with 0.95 s the pitch is 2.7 and 2.1 deg and the bias 0.017 and 0.013 m. Leveling (#137; the multi-ring lidar of #138 is built) removes this
  effect. No range latency beyond the slant geometry was found (best fit -0.002 s); the median sample age
  in the ramp is 0.084 s (p5 0.036, p95 0.136), which the model carries.

Measured minus predicted closest approach, cell means in m (negative: closer than predicted):

| Model | 5 m/s 0.5 s | 5 m/s 0.7 s | 5 m/s 0.95 s | 12 m/s 0.5 s | 12 m/s 0.7 s | 12 m/s 0.95 s |
| --- | --- | --- | --- | --- | --- | --- |
| As is | -0.41 | -0.29 | -0.14 | -0.41 | -0.18 | -0.02 |
| With (b) only | -0.34 | -0.27 | -0.14 | -0.40 | -0.18 | -0.02 |
| With (a) only | -0.16 | -0.11 | -0.06 | -0.11 | -0.05 | 0.00 |
| With (a) and (b) | -0.08 | -0.06 | -0.04 | 0.00 | +0.02 | +0.04 |

With both effects at their measured values the model is within -0.10 and +0.06 m of every run. The slope of
the closest approach against `CPB_T_LAT` is 0.705 m/s (5 m/s) and 0.880 m/s (12 m/s) measured, 0.62 and 0.80
with both effects, and 0.12 and 0.02 as is. A residual of 0.04 to 0.08 m at 5 m/s is unexplained.
Limits of this check: G, tau and tau_r were fitted on the same 18 runs, so the agreement is in-sample, not an
independent validation. The falsification test (the velocity from ground truth inside the velocity loop and
the hook) was NOT run, so (a) is a fit that reproduces the data, not a confirmed cause. The velocity-estimate
error is an effect of the SITL estimator; on a vehicle it differs and must be measured on target (HIL, flight
with an external position reference).

**Earlier proposals, withdrawn.** `T_LAT` = tau_r + t_hold (tau_r + t_s about 0.59 s) is withdrawn: 0.5 s, the
nearest measured value, leaves 0.42 m (5 m/s) and 0.37 m (12 m/s) short of the margin, and a linear
interpolation of the measured slopes to 0.6 s gives about 1.65 and 1.72 m, below the band (reasoning, not
run). The dead time "fitted to hook ramps" of the earlier decision 5 is circular: it returns the `T_LAT` put
into the run. The step-response value counts the braking twice in the model, because the step saturates the
tilt and the cap law already models that with `A_BRK`.

**Braking attitude while the hook limits.** Peak nose-up is 7.3 to 8.2 deg at 5 m/s and 8.5 to 10.2
deg at 12 m/s. Nose-down is 3 deg (5 m/s) and 10.5 deg (12 m/s) in cruise and about -22 deg while
accelerating. A velocity step to zero from 8 m/s reaches 40 to 44 deg of tilt (`MPC_TILTMAX_AIR` is
45). Geometry (mount pitched down 10 deg, vertical field of view -7 to +52 deg from the boresight):
ahead, the world elevation of the field is (theta - 17) to (theta + 42) deg, with theta the nose-up
angle, so the horizon stays in view for -42 <= theta <= +17 deg. While limiting at 12 m/s the lower
edge is at -6.8 deg, which leaves 6.8 deg for gusts and mount tolerance. In a step to zero the horizon
is out of view by 23 to 27 deg. ADR 0020 D3 sizes forward braking as g*tan(17 deg), `CPB_A_BRK` 3.0;
that holds only while the cap falls along the stopping curve. ICD-CP-04 says `A_BRK` and `JERK` are
model inputs, not limits on the setpoint rate, and a new obstacle tightens the cap at once (ADR 0008).

**Cap steps have more causes than a new obstacle:** SAFE_STOP, a stale map, one missing pose, a reset
that breaks continuity, late confirmation of a thin target, tightening of the travelled distance. Each
is a step to zero, so tilt can reach 40 to 44 deg and the horizon is lost. Lost evidence then keeps
the cap at 0, so the stop sustains itself. This is an availability problem, not yet measured.

## Decisions
| # | Decision | Rejected or deferred, and why |
| --- | --- | --- |
| 1 | Tilt on a cap step: accept, the PX4 tilt limit is the only bound; no code change | (b) The hook slews the setpoint decrease: REJECTED. The hook is a stateless pure function called up to three times per cycle (ADR 0020), and a cap falling along the stopping curve already asks for about `A_BRK`, so a slew binds only on the excess beyond the curve, which is the safety-critical part. (c) Same as (a) with a stated non-claim. (d) Lower `MPC_TILTMAX_AIR` globally: no code, but trades hard-stop authority for the horizon; decide with hardware data. `MPC_ACC_HOR` and `MPC_JERK_AUTO` cannot bound the tilt: they shape the Mission trajectory before the hook (Position mode uses `MPC_ACC_HOR_MAX` 5 and `MPC_JERK_MAX` 8) |
| 2 | Hook feed-forward equal to the cap rate: not now; keep the cleared feed-forward (ADR 0020 table row 1) | With ff equal to the cap rate the ramp lag becomes about Kd/Kp (0.12 s), not zero (zero needs (1+Kd)*rate). The rate needs a new field in `cp_brake_constraints` (an ICD-CP-04 change). The slack is in the final creep, not in the ramp. Re-evaluate only if the lag limits the speed claim |
| 3 | Integrator: keep I | Conditional integration (stop integrating while the hook limits) is dropped: it would increase the ramp lag, needs a change inside the upstream velocity controller, would have to freeze only the component along the limiting sector normal, and reverses the contract that the hook does not touch the integrators |
| 4 | Baseline: one parameter set, below | Every limit stays TBD until the baseline data is shown |
| 5 | `CPB_T_LAT` is derived by the validated model from measured characteristics, below; the bench value in use is 0.95 s | Never tuned to make a run pass. Step-response and ramp-fit definitions rejected as the definition (below) |

### Decision 1. Tilt on a cap step and SYS-CP-018
Decision 1 requires an amendment of SYS-CP-018, whose first clause says the horizon stays in view
through braking limits set per direction. Proposed: the envelope of OPEN-039 excludes cap steps, and
the horizon is not claimed during a stop caused by a cap step. The amendment is made by the docs-sync
PR after approval, not by this PR.

### Decision 4. Baseline parameter set
Kept outside the repository until the owner approves it (it is a PX4 platform value): `MPC_XY_VEL_P_ACC`
3.0, `MPC_XY_VEL_D_ACC` 0.35, `MPC_XY_VEL_I_ACC` 0.4, `MC_ROLL_P` and `MC_PITCH_P` 4.0, `MPC_XY_P` 0.95,
all else default. These are gains only: `MPC_ACC_HOR` and `MPC_JERK_AUTO` are setpoint limits and stay.

Fingerprint rule: the harness loads the baseline at every start. The analyzer compares the full ULog
parameter set with the baseline, except a listed allow-list, and gives INVALID on a difference. The
baseline hash is recorded in the ICD-CP-05 manifest with the PX4 and repository commits and the model
and world hashes. PX4 SITL saves parameters between starts, and the earlier rule "MPC and CPB
parameters" was too narrow: `MC_*`, `MPC_TILTMAX_AIR`, `MPC_ACC_HOR_MAX`, `MPC_JERK_MAX`,
`MPC_POS_MODE`, `EKF2_*`, `SIM_GZ_EN_LIDAR` and `CP_DIST` also matter.

### Decision 5. `CPB_T_LAT` definition and margin check
Owner decision, chat 2026-11-21. `CPB_T_LAT` (SYS-CP-027, AGENTS.md) is a value derived by the validated
model from the measured characteristics, and never tuned to make a run pass:
- tau_r, the ramp-following lag of the velocity loop with the hook on, in the truth frame (0.475 s at 5 m/s,
  0.499 s at 12 m/s on the bench);
- the velocity-estimate error delta during braking (G and tau above), measured against ground truth on the
  bench and against an external reference on target;
- the map age and the 50 ms evaluation hold;
- while the ray source is planar, the pitch effect of the lidar (effect (b)).

The model prediction and the measured residual per speed are reported with every value, in the form of
the table above. The residual band `B_model` and the number of repeats per cell are TBD (the owner decides
after more data); until `B_model` exists the model is a candidate, not validated, and the agreement above is
in-sample.

Bench value in use: 0.95 s, approved by the owner in chat on 2026-11-22 as a bench value for the X500 in SITL
with the controller baseline of decision 4. It meets the band of the Consequences in the truth frame at both
speeds (1.90 and 2.03 m). It is a bench platform value, not an approved safety value for a vehicle, and it is
expected to fall when leveling removes the slant part (effect (b) is 0.058 to 0.075 m at the instant the
cap reaches 0 with 0.5 s, and 0.013 to 0.017 m with 0.95 s). The model with both effects predicts 1.82 m (5
m/s) and 1.84 m (12 m/s) for 0.7 s, inside the band, while 1.76 m was measured at 5 m/s (residual -0.06 m); how
a residual of that size enters the choice of the value is decided with `B_model`, not here.

Step-response value. The bench rule approved for the SITL proof ("measured from a velocity step response in
the same SITL and rounded up", `docs/architecture.md`) stays the approved rule until the owner approves a
change. The step value grows with speed (0.56 to 0.90 s; frame not recorded) and counts the braking twice,
because a step saturates the tilt, which the cap law models with `A_BRK`. For set A it is 0.68 s rounded up
to 0.70 s (0.73 s and 0.75 s for the defaults). In the truth frame 0.7 s gives 1.76 m at 5 m/s, below the
band. The step value is therefore reported next to the model value, not used as the definition. The
proposed replacement of the bench rule is made by the docs-sync PR after approval.

Outer-loop gain and margin check. `T_LAT` is also an outer-loop gain: near the margin the cap is about
(range - margin)/T_LAT, a proportional loop through the lidar at 10 Hz. Margin check: `T_LAT`/1.5 and
`T_LAT`/2, judged on criterion c in the truth frame (overshoot and sign changes), plus a test that holds the
stick toward the wall for 30 s after the stop and counts sign changes. The inner gains x1.5 and x2 are a
separate robustness check, also judged on criterion c in the truth frame. The campaign cells 0.5 s and 0.7 s
are near 0.95/2 = 0.475 s and 0.95/1.5 = 0.63 s but are not that check; they had at most one sign change at
both speeds (truth frame, 3 runs each).

A linear loop does not limit-cycle; a sustained oscillation would need the nonlinear parts: the
one-sided half-plane clamp, the 0.3 s dwell with the 2 m/s^2 rise, the 10 Hz sample and hold.

### The hook only slows (owner requirement, chat 2026-11-21)
The hook never commands a retreat; after an overshoot the vehicle stops where it is.
- Met by design for the hook output (reasoning from the source): the limiter projects the setpoint onto
  half-planes whose caps are at least 0 (SYS-CP-012), the origin lies inside every half-plane, so a
  projection never lengthens the vector, and `speedCap` returns 0 when the range is not above the margin.
  The unit test "never faster than asked" checks that the output speed does not exceed the input speed.
- The rollback after the stop at 12 m/s (0.62 m with 0.5 s, 0.39 m with 0.7 s; none at 5 m/s or with 0.95 s)
  comes from the PX4 velocity-loop integrator, not from the hook (reasoning; with I = 0 the rollback
  disappeared in the earlier runs, EKF frame, not evidence). Criterion c allows one sign change. Decision 3
  (keep I) stays.
- Tests to add with #139 and #140: a log-level check that for every hook sample |v_out| <= |v_in| and
  v_out . v_in >= 0. The unit test covers the first part only.
- Consequence for the error budget: because the hook cannot push back, the velocity-estimate error during
  braking consumes margin after the cap is 0 (0.24 to 0.34 m creep at 0.5 s). It is a line of the budget,
  measured on target.

## Consequences
- The baseline is bench X500 and not for the M350 until repeated in HIL and flight.
- Any gain change makes earlier stopping results stale, and `CPB_T_LAT` is derived again.
- SYS-CP-018 is amended (decision 1); the hook and the integrators are untouched, and decision 1 needs no code change.
- Acceptance band for criterion a on the bench (owner decision 2026-11-21), in the truth frame: closest
  approach at least the margin minus 0.20 m, so 1.80 m against 2.0 m; at most one sign change and the
  vehicle at rest before the wall. The upper side is a tuning target. The earlier reading that 1.94 m is
  accepted was in the EKF frame and is withdrawn.
- Proposed, made by the docs-sync PR after approval (this PR changes none of them): SYS-CP-027 and the bench
  rule "measured from a velocity step response and rounded up" are amended to the definition of decision 5;
  the ICD-CP-04 sentence "`T_LAT` is the lag between the hook output and the vehicle response" no longer
  describes the value, which also absorbs the velocity-estimate error and, while the source is planar, the
  lidar pitch effect (its "not counted again" for the sample-to-hook age stays true for the part the
  travelled-distance correction covers; the 50 ms evaluation hold is not covered by it); the closest approach
  2.00 to 2.25 m of the proof of PR #124 in the stopping-proof text is an EKF-frame number and is marked so
  there; OPEN-043 lists `B_model` and the number of repeats.

## To measure first (not decided yet)
- Time and count of events with nose-up above 17 deg or roll above 7 deg, by trigger (cap step cause).
- Roll in the sideways logs.
- The velocity-estimate error and delta on target (HIL, flight with an external position reference).
- The falsification test: the velocity from ground truth inside the velocity loop and the hook (NOT run).
- Terminal damping zeta of about 0.63 to 0.71 (model, unverified).
- `T_LAT`/1.5 and /2 on criterion c in the truth frame; the stick-held test.
- The part of the creep not explained by the velocity-estimate error and the lidar pitch (0.04 to 0.08 m
  at 5 m/s).
- A unit test for issue #148.
- The campaign repeated after leveling.
- Done: `CPB_T_LAT` re-measured in the truth frame (the campaign above); the P 8.5 / D 1.4 test (invalid, no
  separation of gain and damping).

## Not decided yet
Lower tilt limit (d), per-class tilt, feed-forward re-evaluation, inner-gain margins for hardware, `B_model`
and the number of repeats per cell, how a residual enters the choice of `CPB_T_LAT`.
