# 0010. Quantitative baseline is approved for design and test only

## Context
Implementation and test planning need numbers before flight evidence exists.

## Decision
The PERF, SAFE, DET, REL and ENV values in section 2 of `docs/architecture.md` (12 m/s ceiling, 3 m/s^2 and 4 m/s^3
reference, 350/500 ms latency, 0.5 s stale stop, 2.0 m margin, 40 m target class, 5% false-brake,
light-rain envelope) are the design and test baseline. They are not flight-qualified limits.

## Consequences
- Each value changes only through measured evidence and owner review (OPEN-002 to OPEN-008).
- Tests never relax a value to obtain a pass.
