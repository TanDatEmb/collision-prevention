# 0011. Layered verification instead of one simulator

## Context
One high-fidelity simulator would mix numerical, integration, perception, timing and flight claims.

## Decision
Use six lanes (unit and property, synthetic SITL, Gazebo, rosbag replay, HIL and bench, flight).
Each lane claims only what it can evidence, and results are reported as separate claims.

## Consequences
- Simulation cannot qualify detection range or real stopping distance.
- Simulators must preserve emitted-ray direction for returns and no-returns to test clear space.
