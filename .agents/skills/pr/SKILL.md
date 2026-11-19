---
name: pr
description: Create or update a pull request for this repository with truthful verification evidence.
---

# Pull request

The PR title becomes the squash commit on `main`, so it uses the commit format:
`type(scope): imperative summary` (see the `commit` skill).

Before creating:
- `gh pr list --author "@me" --state open` shows no other open PR for the agent.
- The branch was cut from the latest `origin/main`.

Body follows `.github/pull_request_template.md`:
- **Problem**: what is wrong or missing.
- **Change**: what the PR does and which names it introduces.
- **Refs**: issue, `SYS-CP-xxx`, `OPEN-xxx`.
- **Verification**: exact commands and real results; list what was NOT run.
- **Risk**: interface, timing and safety impact.

Open the PR (draft is fine) after the first push. Fix review or CI feedback with new commits on
the same branch. Never approve or merge.
