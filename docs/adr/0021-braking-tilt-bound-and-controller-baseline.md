# 0021. Braking tilt bound and the controller baseline

Status: Proposed. The owner agreed to the decisions below in chat on 2026-11-21; the approval is
recorded in a separate PR. Bench values only: no value here is approved for a vehicle. Issue #142.
The IDs it uses (SYS-CP-018, 027, OPEN-003, 004, 039) are defined in `docs/architecture.md`.

## Context
Measured in Gazebo SITL, X500, forward flight with the nose along the leg, hook on, a tall wall.
The values live outside the repository (bench values); runs are single unless stated.

**The vehicle runs above the velocity cap while braking.** The excess is 1.44 m/s at 12 m/s and
1.07 m/s at 5 m/s with the PX4 defaults (`MPC_XY_VEL_P_ACC` 1.8, I 0.4, D 0.2). The PX4 code
computes `acc_sp = ff + Kp*e + I - Kd*a_measured` (`PositionControl.cpp`; the derivative acts on the
measured, low-passed acceleration), and the hook clears the horizontal feed-forward. The predicted
ramp-following lag is about (1+Kd)/Kp: 0.67 s predicted and 0.61 s measured with the defaults, 0.45 s
predicted and 0.43 s measured with the chosen set below.

**Integrator.** The PX4 horizontal anti-windup runs only when the produced acceleration is below the
commanded one (tilt or thrust saturation). Without saturation the integrator integrates a real
tracking error and acts like the missing feed-forward during a ramp. With I = 0 the rollback after
the stop disappears, but the vehicle creeps to 1.60 m from the wall.

**Attitude gains.** `MC_ROLL_P` and `MC_PITCH_P` from 3.0 to 6.5 do not change the velocity response
(rise 1.08 to 1.12 s) and raise the pitch-rate power above 2 Hz from 0.7 to 5.4 percent, so the
default 4.0 stays. `MPC_XY_P` 0.6, 0.95 and 1.3 gave identical stop results in Mission mode with the
hook limiting.

**Velocity loop with set A** (P 3.0, D 0.35, I 0.4): rise 10 to 90 percent 0.95 s, overshoot 2.1
percent, no ringing, settling 3.5 s, ramp-following lag 0.43 to 0.44 s, pitch-rate power above 2 Hz
4.5 percent. Closed loop (hook on, forward, tall wall, `CPB_T_LAT` 0.95 s, two repeats each) against
the default (set A closest approach 2.44 and 2.29 to 2.30 m):

| | Default | Set A |
| --- | --- | --- |
| Cap excess at 12 m/s | 1.44 m/s | 0.98 m/s |
| Cap excess at 5 m/s | 1.07 m/s | 0.74 m/s |
| Reversal after the stop | one | none |

Gains x1.5 (P 4.5, D 0.525) and x2 (P 6.0, D 0.7) stayed clean: closest 2.28 to 2.43 m, no reversals.
The effective gain is about Kp/(1+Kd): P 6 with D 0.2 gives 5.0, P 8 with D 0.2 gives 6.7, P 6 with D
0.7 gives 3.5. The earlier finding "P 6 is worse and P 8 fails" is therefore explained by the
effective gain, not necessarily by damping. A test with P 8.5 and D 1.4 (also 3.5) separates the two;
it has not been run.

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
| 5 | `CPB_T_LAT` stays a measured value, below | Never tuned to make a run pass |

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
`CPB_T_LAT` stays a measured value (SYS-CP-027, the approved bench rule "measured and rounded up",
AGENTS.md). The step-based value grows with speed (0.56 to 0.90 s) because a step saturates the tilt,
which the cap law already models with `A_BRK`, so it mixes delay with saturated braking.
- Definition to use: the dead time that makes the model stopping distance match the measured stopping
  distance for hook ramps, reported with its residual per speed.
- The step value stays as an upper bound: 0.68 s rounded up to 0.70 s for set A, against 0.73 s
  rounded up to 0.75 s for the defaults.
- `T_LAT` is also an outer-loop gain: near the margin the cap is about (range - margin)/T_LAT, a
  proportional loop through the lidar at 10 Hz. Margin check: `T_LAT`/1.5 and `T_LAT`/2, judged on
  criterion c only (overshoot and sign changes), plus a test that holds the stick toward the wall for
  30 s after the stop and counts sign changes. The inner gains x1.5 and x2 are a separate robustness
  check.
- A linear loop does not limit-cycle; a sustained oscillation would need the nonlinear parts: the
  one-sided half-plane clamp, the 0.3 s dwell with the 2 m/s^2 rise, the 10 Hz sample and hold.

## Consequences
- The baseline is bench X500 and not for the M350 until repeated in HIL and flight.
- Any gain change makes earlier stopping results stale, and `CPB_T_LAT` is measured again.
- SYS-CP-018 is amended (decision 1); the hook and the integrators are untouched, and decision 1 needs no code change.
- Acceptance band for criterion a on the bench (owner decision 2026-11-21): margin minus 0.20 m, so
  1.94 m against 2.0 m is accepted, threshold +-20 cm; the upper side is a tuning target.

## To measure first (not decided yet)
- Time and count of events with nose-up above 17 deg or roll above 7 deg, by trigger (cap step cause).
- Roll in the sideways logs.
- `CPB_T_LAT` re-measured with the chosen gains; `T_LAT`/1.5 and /2 on criterion c; the stick-held test.
- The cause of the final creep (the 0.00 to 0.06 m slack at 3 to 5 m/s is in the creep).
- A unit test for issue #148.
- The P 8.5 / D 1.4 test.

## Not decided yet
Lower tilt limit (d), per-class tilt, feed-forward re-evaluation, inner-gain margins for hardware.
