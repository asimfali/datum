---
name: bless-hashes
description: Re-pin Datum's reference model hashes after an intended geometry change. Use when `reference_models` fails with "a reference model changed", or when adding a model under tests/models.
---

Reference hashes live in `tests/model-hashes.json` (one entry per model and per level, `<file> [<level>]`),
pinned by `crates/datum-model/tests/reference_models.rs`. `DATUM_BLESS=1` rewrites the **whole file** from
what the current code produces, so it also pins whatever else is half-done in the tree.

1. `git status`: the tree holds only the intended change. Commit or stash the rest first — a stray edit
   elsewhere would be pinned and then pass every later check.
2. `scripts/check.sh`. It runs the pin test and `determinism-check` at 1, 4 and 16 threads; a mismatch there
   is a determinism bug, not a pin change: fix it, do not bless.
3. `DATUM_BLESS=1 cargo test -p datum-model --test reference_models`.
4. `git diff tests/model-hashes.json`: only the models and features you meant to change may differ. The
   failure output before the bless is two whole maps, so read the diff, not the assert. Anything unexpected:
   `git checkout -- tests/model-hashes.json`, find the cause, start over.
5. A new model: add `tests/models/<name>.json`, bless, and check that exactly its keys appear.
6. The unit cube hash in `crates/datum-brep/src/tests.rs` is not covered by the env var; edit it by hand when
   `Body::hash` itself changes.
7. `scripts/check.sh` again — green. Commit per `AGENTS.md`.

Done when the diff touches only the intended entries and `scripts/check.sh` passes. If the reason for the
change does not fit in one sentence, it is probably a bug.

Report in Russian: what changed and why, or what you rolled back and why.
