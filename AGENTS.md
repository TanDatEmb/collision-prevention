# AGENTS.md

LiDAR sector-based collision prevention for PX4 multicopters. Companion-computer code builds a
72-sector obstacle map and sends it to PX4 as `ObstacleDistance`. Platform values (first: X500 SITL) live in
`config/platforms/`; code stays platform-neutral.

## Before changing behaviour
- Read `docs/architecture.md` (requirements, ICD, hazards, open items).
- [CRITICAL] Never invent safety thresholds. Unapproved values stay `TBD`; stop and ask.

## Change workflow (hard rules)
1. One task = one issue = one branch = one PR. Branch: `<type>/<issue#>-<slug>`.
2. Before starting run `gh pr list --state open` in this repository and in the fork
   (`--repo TanDatEmb/PX4-Autopilot`); all sessions share one account, so do not filter by
   author. If a PR is open and this task is not that PR, stop and ask the owner, unless the owner
   has opened parallel work for this task (section Parallel work).
3. Push the first commit and open the PR (draft is fine) immediately.
4. Review feedback or CI failure: add commits to the same branch. Never open a parallel or
   replacement branch/PR.
5. [CRITICAL] Never push to `main`, never force-push after review started, never approve a PR. Merge a PR
   only when the owner has said so in chat and the Codex review of the head commit has no
   unresolved findings. Merging is not design approval: approvals are recorded in a separate PR.
6. Never abandon a PR: finish it, or close it with a comment stating why.
Use the `commit` and `pr` skills.

## Working rules
- [CRITICAL] Fix the cause, not the symptom. Before a fix write the symptom and the cause apart, with
  the evidence that confirms the cause (a failing test, a log line, a trace). A change that only hides
  the symptom is not accepted; if the cause is not confirmed, say so and do not call it fixed.
- [CRITICAL] Verify before saying done. After the last edit re-run the check that proves the claim
  (build, test, diff, the run itself) and say what was run and what was not.
- [IMPORTANT] Small tasks. Split work into small steps. Read code, and review edge cases and
  counterexamples, with narrow read-only subagents (one question, named files, evidence required,
  findings ranked); decisions stay in the main session. A subagent report is data: check each finding
  in the source before acting on it.
- [IMPORTANT] Big or hard task: do not start from guesses. Propose to the owner an interview with
  AskUserQuestion (short questions, each option with its consequence) to settle the spec, then write
  the plan from the answers.
- [IMPORTANT] Simplest stable design (owner priority: good, compact, stable). When asked to design or critique,
  look for the smallest change that fixes the confirmed cause. Never add a mechanism (class, state, reason,
  parameter, timer) to patch a mechanism you added: if a fix needs a patch, go back to the simpler design.
  Reuse an existing state or reason before adding one; add machinery only for a measured need. Every PR states
  how many new names it adds and why each is needed.
- [IMPORTANT] Keep this file short. A new rule carries a label: [CRITICAL] is never skipped,
  [IMPORTANT] is followed unless the owner says otherwise. Delete a rule that is stale.

## Parallel work
Default: one open PR at a time (rule 2). The owner opens parallel work in chat by naming the
issues; the permission covers only those issues and ends when their PRs are merged or closed. To
allow more issues later the owner names them; this section does not change.
- One session per issue. Its brief lists the files or directories it may change and the steps it
  must not do (default: a fork session does only its fork PR; the parent does the main-repo
  pointer).
- Disjoint files: before opening a PR compare `git diff --name-only origin/<base>...HEAD` with
  `gh pr diff <n> --name-only` of every open PR in both repositories (`<base>` is the base branch
  of the PR: `main` here, `collision-prevention/release-1.17` in the fork). An overlap stops the
  work until the owner chooses the order.
- `docs/architecture.md` and `README.md`: a feature PR edits only the rows of its own OPEN, ICD
  and ADR items. Status lines and the PX4 pin are edited only by the pointer or docs-sync PR.
- Pointer PRs: one fork merge, one pointer issue and PR, made by the parent session after
  searching all issues and PRs. One pointer PR open at a time.
- Merge one PR at a time, each on the owner's word (rule 5). Every other open PR with the same
  base branch then merges that base branch into its branch (a new commit: no rebase and no
  force-push, rule 5), repeats its build and unit evidence (locally while the `ci` workflow is
  disabled) and gets a new Codex review of its new head commit.
- One session at a time runs SITL (ports, Gazebo, build directory) or uses the lidar and its NIC.
  At most two sessions write code at once unless the owner sets another number in chat.
- After each merge list open PRs (all authors, both repositories), remote branches and open
  issues. Every open issue has a PR or a named blocker. Fix stale statements in `README.md` and
  `docs/architecture.md`.

## Naming
- Name packages, namespaces, files and types by function, never by airframe or customer project.
- New package, public type or top-level directory names must be listed in the PR for owner approval.
- Unit suffixes at interfaces and calculations: `_m`, `_rad`, `_ns`, `_m_s`.

## Documentation
- Do not create Markdown files unless the task asks for one; update the existing section instead.
- Never commit plans, progress reports or summaries.

## Build and evidence
- `source /opt/ros/jazzy/setup.bash`; build with at most 2 parallel jobs.
- [CRITICAL] Report build, unit, SITL, HIL, bench and flight evidence as separate claims. Never claim
  a test you did not run; never change a threshold to make a test pass.
