---
name: commit
description: Write a traceable Conventional Commit for this repository when the user asks to commit.
---

# Commit

Format: `type(scope): imperative summary`

- `type`: feat | fix | docs | refactor | perf | test | build | ci | chore | revert
- `scope`: package name (`obstacle_sector_map`, `px4_obstacle_bridge`) or `px4`, `docs`, `ci`.
- Summary: lowercase, imperative, at most 72 characters. No "update", "WIP" or "fix bug".
- One commit = one logical change.

Body (wrap at 72):
- Explain why, not what.
- Behaviour changes add `Refs: SYS-CP-xxx` and either `Tests: <command that was run>` or
  `NOT RUN: <reason>`.

Trailer: `Assisted-by: <tool>:<model-id>`. Never name AI in `Co-Authored-By` or `Signed-off-by`.

Never commit to `main`. Check `git status` and stage files explicitly.
