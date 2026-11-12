---
name: pr
description: Open a Datum pull request for a finished task — branch, checks, HANDOFF, body, review. Use when the user asks to open/make a PR, or when a task is done and ready for review.
---

One task = one branch = one squash commit on `main`. The policy (who commits, who merges, when push is
allowed) is in `AGENTS.md`; this is only the order of steps.

0. Start from an up-to-date `main`: `git switch main && git pull --ff-only && git fetch --prune`, then
   `git status -sb`. A branch cut from a stale `main` costs a rebase after the first conflict.
1. `scripts/check.sh` (skill `/check`) green in this turn. CI repeats it on both OSes, but a red PR
   spends a review cycle on something the machine already knows. A change that touches only `**.md`
   skips CI (`paths-ignore`) and skips the run here too — say "изменений в коде нет" in the body, not
   "CI зелёный".
2. `HANDOFF.md` updated per `AGENTS.md`: state, next step, checks. The PR body is read in a month,
   `HANDOFF.md` in a day.
3. Branch from `main`, named after the task: `feat/<topic>`, `chore/<topic>`, `plan/<topic>`,
   `release/<version>`. Commit messages in English (`AGENTS.md`); the subject line becomes the PR title,
   so write it as the title you want.
4. Push only when the author asked for it — `git push -u origin <branch>`. A configured permission is not
   an instruction.
5. Body: fill `.github/pull_request_template.md`. Do not keep a second copy of the format here.
6. Title: English, a noun phrase, no `feat:`/`PR:` prefix. A decision in the journal goes in
   parentheses — `(D258)`; a release is `Release 0.5.5`.
7. `gh pr create --base main --title … --body-file …`. `--draft` only if the author wants an early look.
8. Review: the `reviewer` subagent on the diff (read-only, findings with `file:line`); `reviewer-pro` only
   on the author's direct request. Fix the findings on the branch and re-run `scripts/check.sh`.
9. Merge only with the author's explicit permission — skill `merge-pr` (squash, delete the branch).

Traps:
- Untracked prototypes (`crates/datum-ops/tests/stamp_spike.rs`, `examples/stamp-leaf.json`) never ride
  along in a commit. `git status` before `git add`, and name them in "Не входит".
- Changes to `**.md` only skip CI (`paths-ignore`). Say "CI не запускался — только документация", not
  "CI зелёный".
- No merge commits and no force-push to `main`; a stale branch is rebased on `main`, not merged.

Report in Russian, one or two lines: branch, PR number, what the checks say, what is still open.
