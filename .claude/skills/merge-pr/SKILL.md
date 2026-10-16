---
name: merge-pr
description: Merge a Datum pull request after CI passes, then sync local main. Use only when the user has explicitly allowed Claude to merge (in this conversation or durably); otherwise tell them how to merge it themselves.
---

Merging is outward-facing: do it only with the user's explicit permission.

1. `gh pr checks <n>`: every check must pass. If one fails or is pending, stop and report.
2. `gh pr view <n> --json mergeable,reviewDecision`: stop on conflicts.
3. `gh pr merge <n> --squash --delete-branch` (one milestone or task = one commit on `main`).
4. Sync: `git switch main && git pull && git fetch --prune`, then delete the local branch if it is still there.
5. Report in Russian, one or two lines: what was merged and the new `main` commit.
