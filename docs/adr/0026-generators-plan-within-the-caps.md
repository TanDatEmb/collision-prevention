# 0026. The trajectory generators plan within the caps; the hook stays as the backstop

Status: Proposed. In chat on 2026-11-25 the owner asked for the root cause rather than another fix, then chose
this direction for both Position and Mission ("A: cả hai chế độ"), issue #217. Their answers:

- The emergency case, where an obstacle appears too close for the generator to follow, will later go to a
  Brake mode with its own ADR. Until then the hook keeps limiting as now.
- When braking for an obstacle the generators use `CPB_A_BRK` and `CPB_JERK`.
- Fork PR #13 is closed.
- The mode-change checks of #209 are part of the acceptance.

A builder review raised three more questions, which the owner answered the same day:
- Stops made by the hook alone are left to the Brake mode. They are not part of the #209 acceptance here.
- When braking sideways or backward, the generators use `CPB_A_SIDE`, the same model the side caps use.
- A throwaway Mission spike measures the main risk, how the position correction meets the hook, before this
  record is approved.

This record amends ADR 0014 on where limiting first happens. It leaves the hook of ADR 0014, ADR 0020, ADR 0021
and ADR 0025 unchanged. Values marked TBD stay TBD.

## Context
**Symptom.** In the owner's Position flights in Gazebo SITL (2026-11-25, bench `CPB_*`, logs outside the
repository), the vehicle did not respond to the stick near obstacles, then jerked. After fork PR #13 it also
shook.

In the second flight:
- The hook switched between LIMITED and CLEAR 467 times in 9.5 min.
- Each switch cleared, then passed back, a feed-forward of 4 to 6 m/s^2.
- Body-rate jitter was 10.7 deg/s while the hook toggled, against 2.5 deg/s while it limited steadily.

**Cause.** The hook overwrites the output of a stateful trajectory generator (`StickAccelerationXY` in Position,
`PositionSmoothing` in Mission) after that generator ran. The generator never learns of the limit, so its
velocity, acceleration and position states keep describing an unconstrained motion. Every symptom so far is
this mismatch showing at a different place:

| Mechanism | Introduced because | Failed because |
| --- | --- | --- |
| Limit after the generator (ADR 0014) | Upstream collision prevention covers Position only; Mission must be covered; one authority | The generator does not know the limit |
| Clear the whole feed-forward when limiting (ADR 0020 row 1) | The feed-forward pushes through the cap; proportional removal had three defects in review | On/off: a step at every start and end of limiting |
| Keep the feed-forward not entering a cap (#194, abandoned) | Smaller steps | Inconsistent with the cut setpoint: the integrator wound up for 15 s and the vehicle hit the wall in SITL |
| (none: a consequence) #209 | | The Mission trajectory runs ahead of the limit and is released on a mode change |
| Velocity anti-windup in Position (fork PR #13, closed) | The stick integrator wound up to 10 m/s behind the limit (#216) | The task rides the cap edge, so the hook toggles and the vehicle shakes |
| Also reset the stored acceleration (proposed, rejected) | Stop the toggling | Same as the withdrawn option (b1) of #209: no turning while the side lock (SYS-CP-031) holds the setpoint |

Each fix at the output moves the mismatch:
- Clearing the feed-forward gives a step.
- Keeping it winds up the integrator.
- Syncing the velocity gives toggling.
- Syncing the acceleration as well stops turning.

ADR 0020 ("What would change this") already names the way out if the setpoint hook oscillates: limit at the
acceleration level instead.

**PX4 precedent.** PX4 has the needed parts:
- Upstream `CollisionPrevention` limits inside `StickAccelerationXY`, on the acceleration, before the drag, jerk
  limit, tilt limit and integration. The integrator therefore never holds a motion outside the limit.
- `PositionSmoothing` already plans the speed toward a stop point within its acceleration and jerk
  (`computeXYSpeedFromWaypoints`). Its velocity and position trajectory stay consistent while it slows.

**Spike (throwaway, local branch, never pushed).** The spike bounds the `PositionSmoothing` cruise speed in Mission
by what the limiter lets through along the leg.

Setup: Gazebo SITL with bench `CPB_*`, the Mid-360S-density model and the cloud 100 ms late. One run per case,
against the same runs on `a814a95391`:

| Case | Hook cut (median / max) | Hook still limiting | Feed-forward lost while limiting (median) | Max tilt | Body-rate jitter | Closest to the wall |
| --- | --- | --- | --- | --- | --- | --- |
| Forward 5 m/s, wall 40 m | 5.84 / 6.91 -> 0.29 / 0.50 m/s | 52 -> 19 % | 0.0 -> 1.5 m/s^2 | 19 -> 19 deg | 0.6 -> 1.1 deg/s | 2.15 -> 2.29 m |
| Forward 12 m/s (cap about 7), wall 90 m | 5.30 / 12.0 -> 0.09 / 0.52 | 88 -> 52 % | 1.3 -> 0.3 | 20 -> 19 | 1.9 -> 2.0 | 2.30 -> 2.47 |
| Sideways 5 m/s (lock 5), wall 60 m | 0.14 / 6.91 -> 0.08 / 0.21 | 64 -> 48 % | 0.0 -> 0.0 | 38 -> 19 | 2.3 -> 0.8 | 2.11 -> 2.21 |

- The large cuts are gone: the generator no longer runs ahead of the hook, the 38 deg tilt of the sideways run
  went down to 19 deg, and `CPB_MARGIN` (2 m) was kept in every run.
- The hook still limits, but by small amounts, for two reasons:
  - **At constant speed on the bound** (12 m/s, sideways), the cut is the position correction: a median of 0.08
    to 0.11 m/s, with the generator velocity at or below the output. The hook switches 170 to 180 times per run,
    as it already did before (117 in the sideways run). The feed-forward there is near zero, so nothing reaches
    the vehicle: jitter does not rise.
  - **While braking** (5 m/s run), the generator lags the falling bound by a median of 0.37 m/s. The hook then
    clears a braking feed-forward of 1.5 m/s^2. That is the remaining step.
- So "how often the hook limits" mostly measures tracking error. What reaches the vehicle is the feed-forward
  lost while limiting, the tilt and the jitter.

## Decision
1. **One rule, applied where the motion is planned.** Each generator bounds its own horizontal velocity by the
   speed that the `cp_brake` limiter would let through in the generator's own commanded direction. The bound
   uses the limiter function of `src/lib/cp_brake` (ADR 0025: direction kept, speed scaled, corridor
   `CPB_MARGIN`, speed lock `CPB_V_SIDE`), the same constraint message and the same parameters as the hook.
   - Position: `StickAccelerationXY`, before integration.
   - Mission: `PositionSmoothing`, as a speed limit along its path.
2. **The generator state stays inside the caps.** The bound acts before the generator integrates or smooths.
   Its velocity, acceleration and position therefore always describe one motion within the caps:
   - nothing winds up;
   - the Mission trajectory does not run ahead;
   - the feed-forward it sends matches its velocity.

   When the bound falls, the generator brakes with the stopping model of the binding sector's class (owner):
   - `CPB_A_BRK` in the forward class;
   - `CPB_A_SIDE` in the side and rear classes;
   - `CPB_JERK` in both.

   Only braking caused by the bound uses this model. Speeding up, and braking when the pilot releases the stick,
   keep the PX4 limits of each mode.

   A constraint that the limiter turns into a safe stop gives a bound of 0, so the generator brakes as well.
   Examples are a stale constraint, a missing heading, or SAFE_STOP. (Builder's call: fail closed, like the
   hook.)
3. **The hook is the backstop, unchanged.** The hook still limits every cycle with the rule of ADR 0025 and the
   feed-forward rule of ADR 0020. In normal flight its input is already within the caps, so it does not act. It
   acts when the generator cannot follow:
   - a cap step: an obstacle detected inside the stopping distance, or a sector turning unknown;
   - a stale or invalid constraint (SAFE_STOP).

   This role is interim, until a Brake mode (separate ADR) takes these cases.
4. **Removed.** The Position anti-windup of fork PR #13 (closed) is not needed. Nor is any per-mode handover patch
   for #209: after a stop the generator made itself, a mode change hands over a state within the caps. This must
   be shown in SITL (Verification).

   After a stop that only the hook made, the generator may still be braking behind and lie ahead of the vehicle.
   That part of #209 stays open for the Brake mode (owner).

**Safety argument.** The hook output stays within the caps every cycle, as before. The stopping proof (ADR 0021,
`CPB_T_LAT`) therefore depends on nothing this record adds: the generators only make the hook's input already
compliant. The stopping evidence is repeated as a regression, not derived again.

## What stays open for the specification of the fork work
- **The position correction (measured by the spike).**
  - The hook limits `v_ff + K_p (p_sp - p)`, constrained to `MPC_XY_VEL_MAX` (`PositionControl::_positionControl`).
    Any tracking lag therefore lifts the hook input 0.1 to 0.3 m/s above a generator that sits on the bound.
  - At constant speed the feed-forward is near zero, so the cut is harmless (spike). It is accepted, not
    patched: no margin and no new value.
  - Position has the same effect once the position lock engages. The Position fork PR measures it.
- **The edge while braking (measured by the spike).**
  - Bounding the cruise speed makes the generator lag the falling bound by a median of 0.37 m/s, and the hook
    clears a 1.5 m/s^2 braking feed-forward.
  - The fork work therefore plans to the stop point of the binding sector, the way `PositionSmoothing` already
    stops at a waypoint within its acceleration and jerk, instead of only bounding the speed.
  - Shown by measurement in the Mission fork PR.
- **Acceptance measures** (values TBD, owner), per approach:
  - the feed-forward lost while the hook limits;
  - the maximum tilt;
  - the body-rate jitter;
  - the closest approach (`CPB_MARGIN`).

  How often the hook limits is reported, but it is not a criterion: it mostly measures tracking error (spike).
- **Different constraint messages.** The task and the hook run in different modules, so a new map, about every
  50 ms, can lower a cap between them. This is covered by the same measures.
- **Position at the bound.** The stick acceleration beyond the bound is dropped along the commanded direction.
  Release and the brake boost stay as in PX4.
- **One limit for both directions in Mission.** `PositionSmoothing` uses one acceleration and one jerk for speeding
  up and slowing down (`MPC_ACC_HOR`, `MPC_JERK_AUTO`). The choice is between splitting them, which changes
  `VelocitySmoothing` and costs flash, and a configuration check against `CPB_A_BRK` / `CPB_JERK`. The spike
  informs it.
- **Inputs of the tasks.** The tasks need `cp_brake_constraints`, the heading, and these parameters:
  `CPB_A_BRK`, `CPB_A_SIDE`, `CPB_JERK`, `CPB_V_SIDE`, `CPB_REAR_ANG` and `CPB_CSTR_AGE`. The new names are counted in each fork PR.
- **Scope unchanged.** The hook enforces only in Position and Mission (ADR 0020 D6). Hold, Land and RTL are not
  limited, and the RTL gap noted in #209 stays.
- **Not addressed.** The hard stops from the coverage rule (forward-class sectors unknown while tilted) stay
  hard: they are cap steps and go to the backstop. Tuning the stick feel (`MPC_ACC_HOR`, bench velocity gains)
  waits until this work lands.

## Alternatives not chosen
- **Fix at the hook's output** (sync the velocity, sync the acceleration as well, or scale the feed-forward with
  the speed): the table above. Scaling the feed-forward keeps a feed-forward that disagrees with a held
  setpoint, which is the failure of #194.
- **Generators only, hook removed or reduced to SAFE_STOP:** deferred by the owner to the Brake mode. Until then
  a late detection would stop too late.
- **Position first, Mission later:** the owner chose both modes.
- **Smooth the output inside the hook** (rate limit, feed-forward equal to the cap rate): rejected by ADR 0021
  decisions 1 and 2. The hook is a stateless function and a cap tightens at once.

## Consequences
- Work order, one PR open at a time:
  1. This record, as an approval PR.
  2. A fork PR for Position, closing #216.
  3. A fork PR for Mission, with the Mission part of #209.
  4. A pointer PR.
  Then the docs sync: SYS-CP-012 and FEAT-15 text, and a FEAT row for the generator bound. ICD: no message
  change.
- The fmu-v6x flash margin is checked in each fork PR (96.4 % used before this work).
- **Verification in each fork PR:**
  - Unit tests of the bound: `StickAccelerationXY` already has a functional test; `PositionSmoothing` has unit
    tests.
  - Gazebo SITL at bench values, approaching a wall, a pole and a gate in Position and in Mission. Measured:
    - the acceptance measures above, against the same runs on the release branch;
    - how often the hook limits and by how much (reported, not a criterion);
    - the stick response when pulling away.
  - The #209 checks: stop in Mission and in Position, switch to Land, Hold, RTL and Position, trigger a
    failsafe, remove the obstacle; watch 60 s after each stop.
  - The stopping-evidence regression.
  - The owner's manual flight.

  Build, unit, SITL and flight evidence are reported apart.
