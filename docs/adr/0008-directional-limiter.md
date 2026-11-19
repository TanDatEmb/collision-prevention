# 0008. Directional constraints with asymmetric release

## Context
A global minimum range would stop forward motion in a corridor because of side walls, and
immediate release causes brake/release chatter on transient returns.

## Decision
- Each sector is a directional half-plane constraint on the horizontal setpoint; only the
  component entering an affected direction is limited.
- A newly credible obstacle tightens its cap immediately, with no multi-frame debounce.
- A cap may relax only after 0.3 s of continuously qualified clear data, and then
  rises at most at 2.0 m/s^2. Unknown or stale data resets the dwell.

## Consequences
- An empty feasible set gives zero horizontal output and a blockage report.
- The dwell and rise rate are design/test inputs, qualified by OPEN-024. Their parameter names
  depend on the implementation choice (OPEN-026).
