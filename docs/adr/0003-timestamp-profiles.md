# 0003. Hardware PTPv2 and simulation clock timestamp profiles

## Context
Observation freshness needs one clock shared by sensor, Orin and PX4. Simulated time must never be
presented as a real sensor timestamp.

## Decision
- Hardware: Mid-360S PTPv2 packet timestamp (`time_type=1`), Orin `CLOCK_REALTIME` disciplined to
  the same PTP master, `UXRCE_DDS_SYNCT=1`, `UXRCE_DDS_SYNCC=0`.
- Simulation and rosbag replay: one Gazebo/rosbag `/clock`, `use_sim_time=true`,
  `UXRCE_DDS_SYNCT=0`, `UXRCE_DDS_SYNCC=0`.
- Both: the map timestamp is the earliest contributing acquisition time; a zero or unconvertible
  timestamp is rejected and yields unknown sectors.

## Consequences
- Only three runtime timestamps exist: sample, constraint and status.
- Stage timestamps are collected only in dedicated latency-profiling runs.
