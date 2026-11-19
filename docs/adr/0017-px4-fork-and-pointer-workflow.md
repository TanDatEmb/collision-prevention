# 0017. PX4 changes go through the fork first, then a pointer PR

## Context
`firmware/PX4-Autopilot` is a submodule of a separate repository, the fork named in `.gitmodules`
(branch `collision-prevention/release-1.17`). The `cp_brake` module and the limiter hook are PX4
code (ADR 0014), so they cannot be changed by a pull request in this repository alone.

## Decision
- A PX4 change is two tasks, so that AGENTS.md rule 1 (one issue, one branch, one PR) holds for
  each: the fork change, and the pointer update.
- The fork-change issue lives in this repository and its single PR is in the fork, branched from
  the fork branch named in `.gitmodules`, reviewed and merged under the same rules as this
  repository (AGENTS.md).
- The pointer update has its own issue and its own PR in this repository. It moves the submodule
  pointer to a commit that is on that fork branch after the fork PR merged, and carries the
  commands and results of the PX4 build and tests, reported as separate build, unit, SITL, HIL,
  bench and flight claims.
- The fork needs a CI job that builds the targets of ADR 0012 and runs the PX4 gtests.

## Consequences
- Each of the two tasks is one issue and one PR; several fork changes may share one pointer
  issue.
- A pointer never references a commit that is not on the fork branch.
- Until the fork CI runs, PX4 evidence is local only and says so (the GitHub Actions billing lock
  also applies to the fork unless the owner resolves it).
- Closes the process question of ADR 0012 for PX4 changes.
