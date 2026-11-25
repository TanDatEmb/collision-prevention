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
- **The position correction (main risk; spike before approval).**
  - The hook limits `v_ff + K_p (p_sp - p)`, constrained to `MPC_XY_VEL_MAX` (`PositionControl::_positionControl`).
  - In Mission the trajectory always sets a position, so any tracking lag adds to a feed-forward that sits at
    the bound. Position has the same issue once the position lock engages.
  - If the hook then cuts that excess, it clears the feed-forward again.
  - A throwaway SITL spike, with Mission and the bound only, measures how often and by how much this happens.
    This record is revised from that data.
- **The edge between generator and hook.**
  - The two run in different modules and can use different constraint messages: a new map about every 50 ms
    lowers a cap between the task and the hook.
  - Following a falling bound from zero deceleration within `CPB_JERK` lags it unless the generator plans
    ahead, as `PositionSmoothing` does for a waypoint.
  - The option chosen, either planning to the stop point of the binding sector or bounding the speed, is decided
    by measurement: how often the hook still limits in a normal approach, and by how much. The acceptance value
    is TBD (owner).
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
    - how often the hook still limits, and by how much;
    - body-rate jitter and the closest approach (`CPB_MARGIN`);
    - the stick response when pulling away.
  - The #209 checks: stop in Mission and in Position, switch to Land, Hold, RTL and Position, trigger a
    failsafe, remove the obstacle; watch 60 s after each stop.
  - The stopping-evidence regression.
  - The owner's manual flight.

  Build, unit, SITL and flight evidence are reported apart.
