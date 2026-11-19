# 0002. Use ObstacleDistance with 72 sectors as the Orin-FMU wire contract

## Context
PX4 already defines `ObstacleDistance` with 72 horizontal sectors, a body FRD frame and explicit
unknown, free and too-close values. A custom message adds version coupling.

## Decision
Publish the standard pinned `ObstacleDistance` on `/fmu/in/obstacle_distance`: 72 x 5 degrees,
`MAV_FRAME_BODY_FRD`, centimetres, 10 Hz nominal, best-effort / volatile / keep-last depth 2.
`0` = too close, `max_distance + 1` = observed clear, `UINT16_MAX` = unknown.

## Consequences
- Only the PX4 bridge converts FLU/counter-clockwise to FRD/clockwise.
- No sequence or health data on the wire; those go through the observability path.
- Message definitions are pinned and checked for parity between PX4 and `px4_msgs`.
