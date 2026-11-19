# 0001. Orin observes, FMU owns final braking

## Context
Perception is complex and runs on a companion computer. Final motion authority must stay on the
flight controller, and concurrent collision-prevention controllers share failure sources.

## Decision
Orin builds the obstacle observation. PX4 applies the final velocity constraint. Exactly one
collision-prevention authority is active at a time. Whether that authority is the existing PX4
collision prevention or a new module is open (OPEN-026), as is the exact hook point.

## Consequences
- Orin never commands actuators or flight modes.
- V1 ends at constrain/safe-stop plus status; it requests no Hold, RTL or mode change.
