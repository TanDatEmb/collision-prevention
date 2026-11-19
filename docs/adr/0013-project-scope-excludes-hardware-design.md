# 0013. Electrical and environmental design are outside this project

## Context
This repository owns perception semantics, timing, PX4 constraint enforcement and evidence.

## Decision
Power, wiring, EMI, thermal and humidity design and qualification belong to vehicle integration.
Sensor geometry, self-mask and extrinsics stay in scope because they change collision sectors.

## Consequences
- Absence of those topics here is not evidence that the installed hardware is qualified.
- Orin thermal throttling is covered only as a latency fault (NFR-CP-004).
