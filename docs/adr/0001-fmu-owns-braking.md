# 0001. Orin observes, FMU owns final braking

## Context
Perception is complex and runs on a companion computer. Final motion authority must stay on the
flight controller, and concurrent collision-prevention controllers share failure sources.

## Decision
Orin builds the obstacle observation. PX4 applies the final velocity constraint. Exactly one
collision-prevention authority is active at a time; it is the new `cp_brake` module (ADR 0014).

## Consequences
- Orin never commands actuators or flight modes.
- V1 ends at constrain/safe-stop plus status; it requests no Hold, RTL or mode change.
