# 0017. PX4 changes go through the fork first, then a pointer PR

## Context
`firmware/PX4-Autopilot` is a submodule of a separate repository, the fork named in `.gitmodules`
(branch `collision-prevention/release-1.17`). The `cp_brake` module and the limiter hook are PX4
code (ADR 0014), so they cannot be changed by a pull request in this repository alone.

## Decision
- The issue for a PX4 change lives in this repository.
- The change is a pull request in the fork, branched from the fork branch named in `.gitmodules`,
  reviewed and merged under the same rules as this repository (AGENTS.md).
- After it merges, a pull request in this repository moves the submodule pointer to a commit that
  is on that fork branch. It carries the commands and results of the PX4 build and tests, reported
  as separate build, unit, SITL, HIL, bench and flight claims.
- The fork needs a CI job that builds the targets of ADR 0012 and runs the PX4 gtests.

## Consequences
- One PX4 task is one issue, one fork PR and one pointer PR.
- A pointer never references a commit that is not on the fork branch.
- Until the fork CI runs, PX4 evidence is local only and says so (the GitHub Actions billing lock
  also applies to the fork unless the owner resolves it).
- Closes the process question of ADR 0012 for PX4 changes.
