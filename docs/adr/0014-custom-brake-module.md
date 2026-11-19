# 0014. PX4 constraint is a new `cp_brake` module

## Context
The stock PX4 `CollisionPrevention` is called only from `StickAccelerationXY` (manual Position
mode) in the pinned PX4 source; Mission is not covered. V1 must constrain Mission (SYS-CP-001,
005) with directional half-plane limiting (SYS-CP-012) and a clear dwell before relaxing
(SYS-CP-013).

## Decision
Write a new PX4 module, `cp_brake`, instead of extending the stock collision prevention. It
validates `ObstacleDistance` input, computes per-sector speed caps, owns the operational state
machine, and applies the caps to the final horizontal setpoint through a hook between
`_positionControl()` and `_velocityControl()`, so position-only Mission setpoints are covered.
When `cp_brake` is enabled the stock collision prevention is disabled by configuration, so exactly
one authority acts (SYS-CP-003). Parameters use the `CPB_` prefix.

## Consequences
- The PX4 fork carries the module and the hook; updates follow ADR 0012.
- Disabled `cp_brake` must leave PX4 baseline behaviour unchanged (SYS-CP-006).
- Message definitions between Orin and `cp_brake`, and the module's internal design, are decided
  at G2 and are not fixed here.
- Closes OPEN-026.
