# 0025. The limiter keeps the commanded direction and limits only the speed

Status: Proposed. The owner chose this option of three in chat on 2026-11-24 ("Nếu vậy thì PA1 cái tôi muốn là
không trượt": option 1, no sliding), issue #195. It amends ADR 0008 decision 1 (each sector a half-plane, only the
entering component limited) and SYS-CP-012. ADR 0008 decisions 2 and 3 (tighten at once, dwell, rise) and the
feed-forward rule of ADR 0020 are unchanged. Values marked TBD stay TBD.

## Context
Evidence with bench values (records outside the repository):
- Gazebo SITL, Livox-equivalent density, certified range 40 m: a mission leg at 12 m/s parallel to a wall 5 m to
  the side. The hook limited the whole leg (sectors 61-65, about 45 degrees toward the wall), the speed swung
  3.7-6.5 m/s, and the hook turned the requested north velocity +0.52 m/s into -1.28 m/s: the vehicle was pushed
  3.9 m off the leg, the mission pulled it back, a cycle of about 10 s. The yaw stayed within 88.6-90.9 degrees.
- Cause: ADR 0008 decision 1 treats sector `i` as a wall normal to its bearing `u_i` at its distance `d_i`. An
  obstacle beside the path is seen by the oblique sectors, so `v cos(theta) <= cap(d)` limits motion along it, and
  the projection `v - e u_i` adds a component along `-u_i`: away from the obstacle and off the commanded
  direction. This is sliding, which SYS-CP-010 excludes, and it depends on which oblique sector binds.
- A kinematic comparison of the current rule and three options (point vehicle, velocity lag 0.4 s, |a| <= 4 m/s^2,
  map at 10 Hz and 0.1 s old, the cap rise of ADR 0008, stopping model with `CPB_T_LAT` 0.95 s, `CPB_A_BRK`
  3 m/s^2, `CPB_JERK` 4 m/s^3, `CPB_MARGIN` 2 m). "Off the command" is the largest angle between the hook output
  and the command:

| Case | Current rule (ADR 0008) | This decision |
| --- | --- | --- |
| Mission leg at 12 m/s beside a wall 5 m away | pushed 6.8 m off, 28 deg off the command | not limited, 0 deg |
| The same beside a wall 2.5 m away (VM-38 distance) | pushed 9.2 m off, 43 deg | 2.7 m/s, 0 deg |
| Corridor 5 / 6 / 10 m wide, along the centre | no motion / no motion / 3.5 m/s | 2.7 / 5.2 / 10.3 m/s |
| Oblique wall 30 deg across the leg | slides 8.8 m along it, 51 deg | stops 1.99 m from it |
| Pole 1.5 m off the path (VM-10 expects a stop) | goes round it, 40 deg | stops 1.97 m from it |
| Pole 3 m off the path | passes, 32 deg | passes at 2.8 m, 0 deg |
| Wall ahead, normal to the leg | stops at 1.94 m | stops at 1.94 m |
| Pilot command 45 or 10 deg into a wall, from 2.5 m | slides, pushed out to 2.7-5.3 m | stops at 1.98-2.00 m |
| Pilot command along a wall, from 2.05 m | pushed 4.4 m off | 0.4 m/s along it |
| Pilot command into an inside corner | 52 deg | stops at 1.99 m, 0 deg |

## Decision
1. Corridor. Sector `i` counts for a horizontal direction `w` when `w . u_i > 0` and
   `D_i sin(max(0, angle(w, u_i) - 2.5 deg)) < R`, with `R = CPB_MARGIN` and `D_i` the distance whose stopping speed
   is the held cap `c_i` (`D_i = stoppingDistance(c_i)`, so `c_i = 0` gives `D_i = CPB_MARGIN`). Sectors are
   centred on `5 i` degrees (ICD-CP-03), so 2.5 deg is the half-width: every point of the sector at a range of at
   least `D_i` lies at least `D_i sin(...)` from the ray along `w`, and a sector that does not count holds no
   obstacle within `R` of that ray. A sector whose cap is 0 or invalid counts for every direction that enters it
   (fail closed, as now).
2. Active set. `A` holds the sectors that count for the direction of the velocity setpoint or for the direction
   of the current horizontal velocity; a zero vector has no direction and adds nothing. The current velocity is
   there for the transient while the vehicle turns: with a 45 deg command step at 8 m/s toward a pole 10 m ahead
   and 3 m to the side, the setpoint direction alone gave 0.10 m clearance, both directions 1.20 m.
3. Speed only. `v_out = s v_sp`, `s = min(1, min over i in A with v_sp . u_i > c_i of c_i / (v_sp . u_i))`.
   The direction never changes, `0 <= s <= 1`, every sector in `A` is met, and the result is one pass over the 72
   sectors: the iteration of the current limiter (pass limit, zero output when it does not converge) is removed. When
   `s < 1` the horizontal feed-forward is cleared (ADR 0020, unchanged). The active sector reported is the one
   that sets `s`.
4. The rule for the operator: the vehicle keeps the commanded direction and only its speed drops; it stops
   `CPB_MARGIN` short of an obstacle that may lie within `CPB_MARGIN` of that direction; it is never pushed
   sideways and never slides. Motion along or away from an obstacle farther than `CPB_MARGIN` from the path is not
   limited, apart from the sector quantisation below.

## Alternatives not chosen
- Option 2, slide along the nearest surface and then scale: it went round poles (38 deg off the command) and,
  on a command 10-45 deg into a wall, moved 9-11 m and then stalled: neither a slide nor a stop.
- Option 3, the current projection restricted to the corridor: it slides along oblique walls (53 deg off) and
  pushes the vehicle 2.8 m off a leg beside a wall 2.5 m away; it keeps the iteration.
- The corridor of every direction between the current velocity and the setpoint (an arc instead of two
  directions): 1.37 m instead of 1.20 m in the step case above and no change with a ramped command; not needed.

## Consequences
- Names (3): `cp_brake_constraints.distances_m[72]` (`D_i` of the held caps, computed by `cp_brake` with the
  stopping model it already has) and `cp_brake_constraints.clearance_m` (`CPB_MARGIN`), carried into
  `HookConstraint`, and the current horizontal velocity as an input of the hook (the position controller already
  holds and records it). Removed: `kMaxPasses` and the non-convergence path. The message grows by 292 bytes; the
  flash margin of fmu-v6x is checked in the fork PR.
- Sector quantisation, for any rule on 72 nearest-point sectors: a sector near the commanded direction cannot
  tell an obstacle on the path from one beside it. Top speed beside a wall at lateral distance `L` (bench model,
  yaw on a sector centre / 2.5 deg off, 12 m/s commanded): 2.0 m 0 / 0.1, 2.5 m 2.8 / 3.1, 3.0 m 5.5 / 6.5,
  4.0 m 9.8 / 12, 5.0 m 11.4 / 12 m/s. A corridor narrower than `2 CPB_MARGIN` cannot be passed.
- A command with a component toward an obstacle within `CPB_MARGIN` of its path stops; the pilot steers along or
  away from it, and a mission leg that passes within `CPB_MARGIN` of an obstacle stops there.
- `R = CPB_MARGIN` ties the lateral clearance to the stopping margin: the value (TBD) must cover the vehicle
  half-span, the lateral tracking error and the lateral drift over the map age as well as the stopping margin.
- Transients: the hook limits the setpoint, not the path the vehicle flies while it follows a changed command.
  With an instantaneous 90 deg command step at 8 m/s, 10 m short of and 3 m beside a pole, the clearance fell to
  0.34 m with this decision and 0.91 m with the current rule. With the command ramped at 3 m/s^2, this decision
  kept at least 1.93 m in every case that started outside the stopping distance. The setpoint the hook receives in
  Position and Mission is expected to be smoothed by the PX4 trajectory generation; this is an open item to
  measure in SITL before the claim.
- The approval PR amends SYS-CP-012 (the direction is kept and only the speed is limited; a sector counts when
  its obstacle may lie within `CPB_MARGIN` of the path; a blocked direction gives zero horizontal output and a
  report), OS-02 (no sliding, direction kept), VM-08, VM-38 a (free only to the speed the quantisation allows,
  about 2.7 m/s at `CPB_MARGIN` + 0.5 m with the bench model) and marks ADR 0008 decision 1 as amended.
- Verification for the fork PR: unit tests of the properties (direction kept, output not longer than the input,
  every active sector met, a sector outside the corridor ignored, a 0 or invalid cap counts, retreat not limited);
  Gazebo SITL: walls beside the leg at 5 and 2.5 m, a corridor, VM-06, 07, 08, 10, 37, 38, 39 and 45, command
  swerves in Position mode, and 60 s of watch after every stop; the fmu-v6x build and flash margin.
