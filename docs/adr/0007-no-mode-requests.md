# 0007. V1 ends at constrain, safe-stop and status

## Context
PX4 already owns flight modes and failsafes. A CP-owned Hold or RTL would duplicate and conflict
with them.

## Decision
V1 never publishes a vehicle command or requests Hold, RTL or another mode. Persistent blockage
keeps the constraint and notifies the operator. If estimator, heading or control authority is
invalid, the CP claim is `UNQUALIFIED` and existing PX4 failsafes act.

## Consequences
- No Hold timer exists; the 0.5 s stale threshold only triggers safe-stop.
- Mode management may be reconsidered only in a separately approved V2 baseline.
