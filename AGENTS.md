# AGENTS.md

LiDAR sector-based collision prevention for PX4 multicopters. Companion-computer code builds a
72-sector obstacle map and sends it to PX4 as `ObstacleDistance`. Platform values (first: X500 SITL) live in
`config/platforms/`; code stays platform-neutral.

## Before changing behaviour
- Read `docs/architecture.md` (requirements, ICD, hazards, open items).
- Never invent safety thresholds. Unapproved values stay `TBD`; stop and ask.

## Change workflow (hard rules)
1. One task = one issue = one branch = one PR. Branch: `<type>/<issue#>-<slug>`.
2. Before starting run `gh pr list --author "@me" --state open`. If a PR is open and this task
   is not that PR, stop and ask the owner.
3. Push the first commit and open the PR (draft is fine) immediately.
4. Review feedback or CI failure: add commits to the same branch. Never open a parallel or
   replacement branch/PR.
5. Never push to `main`, never force-push after review started, never approve a PR. Merge a PR
   only when the owner has said so in chat and the Codex review of the head commit has no
   unresolved findings. Merging is not design approval: approvals are recorded in a separate PR.
6. Never abandon a PR: finish it, or close it with a comment stating why.
Use the `commit` and `pr` skills.

## Naming
- Name packages, namespaces, files and types by function, never by airframe or customer project.
- New package, public type or top-level directory names must be listed in the PR for owner approval.
- Unit suffixes at interfaces and calculations: `_m`, `_rad`, `_ns`, `_m_s`.

## Documentation
- Do not create Markdown files unless the task asks for one; update the existing section instead.
- Never commit plans, progress reports or summaries.

## Build and evidence
- `source /opt/ros/jazzy/setup.bash`; build with at most 2 parallel jobs.
- Report build, unit, SITL, HIL, bench and flight evidence as separate claims. Never claim a test
  you did not run; never change a threshold to make a test pass.
