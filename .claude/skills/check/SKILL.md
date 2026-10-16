---
name: check
description: Run the mandatory Datum pre-completion checks (fmt, clippy, tests, determinism) and report results. Use before finishing any code task or when the user asks to check/verify the project.
---

Run `scripts/check.sh` from the repository root (it is the single source of the check list; do not inline its commands).

Report in Russian, briefly:
- passed: one line;
- failed: the failing step, the essential error lines and the suspected cause. Fix it if it is within the current task, then re-run.

Never claim success without a passing run in this turn.
