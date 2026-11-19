# 0005. PERF-03 latency excludes attitude response

## Context
The original note put 100 ms of aircraft attitude response inside the 350 ms latency budget.
The baseline defines PERF-03 as sensor sample to PX4 enforcement, and sizes the stopping envelope
(36.5 m) from that latency plus a jerk-limited braking distance.

## Decision
PERF-03 measures the processing chain only: sensor sample to PX4 enforcement, p99 <= 350 ms, hard
freshness gate 500 ms. Attitude and actuator response belongs to the measured braking envelope
(CP-FLT-001), not to PERF-03. The stage split (frame gathering 100, driver 20, filtering 80,
limiter 20, ROS 2 bridge 30, reserve 100 ms) is provisional until measured (OPEN-004).

## Consequences
- Counting attitude lag twice is avoided.
- The braking model and CP-FLT-001 must include the setpoint-to-actual response.
- Stage budget values are not approved limits.
