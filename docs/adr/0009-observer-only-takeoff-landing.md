# 0009. Observer-only before takeoff state flight

## Context
A horizontal limiter can remove the correction authority PX4 needs near the ground, and V1 has no
vertical or ground-clearance observation.

## Decision
Before `TAKEOFF_STATE_FLIGHT` the constraint is valid but inactive (`enforcement_active=false`).
Intentional landing or descent needs an explicit, logged CP disable with a residual-risk
indication.

## Consequences
- Baseline PX4 takeoff and landing are unchanged; V1 makes no clearance claim there.
- A later landing-phase discriminator needs a new approved baseline.
