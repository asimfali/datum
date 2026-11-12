---
name: geometry-debug
description: Track down a wrong or failed geometry result in Datum — `Degenerate`, a failed build, a body that does not validate, or hashes that differ between thread counts.
---

Work from the smallest model that still fails. A wrong body is a claim about one operation, and the whole
model is noise around it.

1. Reproduce from the command line, not the window: `datum build <file>.json`, with `--level <name>` for a
   model with levels (a plain build is the root alone, while `determinism-check` covers every level). Keep a
   copy of the model before cutting it down.
2. Reduce: drop features from the end while it still fails, then inputs one by one. Ten lines of JSON that
   still fail are worth more than the original file. A model with `levels` or `parts` is reduced on a copy of
   one level: `parts` refer to features by name, and trailing cuts break them.
3. Read the error: it names the feature, the input and usually the id. `Degenerate` and `Unsupported` are
   promises, not bugs by themselves — check the operation's D-number and `ARCHITECTURE.md` §10 (its "never
   done" list is stale: STEP and sheet metal are in), and for `Unsupported` the journal entries that promise
   it (D185 п. 7, D186, D190), not §10.
4. Look at the result: `inspect_model` (per feature: volume, counts, hash, or why it failed),
   `query_topology` (faces with stable IDs, normals, plane offsets, areas), `mass_properties`, `render`.
   A missing, doubled or wrongly oriented face shows here before any code is read.
5. Split the blame: build the input alone and check its volume. If the input is right, the operation is
   wrong, and the reverse.
6. Determinism, when the result is "sometimes right": `datum determinism-check <file>` compares 1, 4 and 16
   threads. Any difference is a bug — unsorted `HashMap` iteration, an unordered float reduction, parallel
   results collected without sorting — even when the picture looks right.
7. Compare with the nearest reference model in `tests/models/` that exercises the same operation. If it broke
   too, the change is in the shared code, not in your model.
8. Decide and record: kernel bug (fix plus a test with these numbers), wrong pin (`bless-hashes`), or wrong
   expectation (fix the model). Add the reduced model to `tests/models/` only if it is a real scenario.

Never widen a tolerance to make a case pass (`AGENTS.md`): a case that cannot be decided exactly is
`Degenerate`.

Report in Russian: the cause, the evidence (the reduced model and what it shows), the fix, and what remains
unsupported.
