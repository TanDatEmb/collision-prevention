# 0006. Self-returns are removed by a static geometric mask

## Context
Airframe, propellers and antennas return rays to the sensor. A temporal filter (drop points that
repeat in direction and range) conflicts with SYS-CP-013 and can hide a real obstacle.

## Decision
The platform config declares a list of upright cylinders in `livox_frame`: axis position (x, y),
radius and an absolute z range (`z_min_m`, `z_max_m`). A return inside any cylinder is masked. There is no temporal filter.
- A masked ray is neither obstacle nor clear evidence; a direction covered only by masked rays
  stays unknown.
- Cylinders hug the aircraft structure plus the smallest approved margin. An obstacle fully inside
  a cylinder is invisible, so they are never enlarged to remove false brakes.
- Values come from CAD, are checked by CP-RPL-001 and are approved via OPEN-005.

## Consequences
- Each file in `config/platforms/` carries `self_mask` with unapproved `null` values.
- Changing a cylinder is a reviewed change, not tuning after an acceptance run.
- A residual false brake is evidence for a change request, not for a debounce.
