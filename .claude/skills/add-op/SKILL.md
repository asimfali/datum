---
name: add-op
description: Add a new kernel operation or model command to Datum end to end. Use when implementing a new `Op` variant, a new command, or wiring one into the window and MCP.
---

An operation is one fact in a chain, and the compiler catches only half of it. The steps below keep the
contract ahead of the code; the places marked **silent** do not fail to compile when missed, so check them
by hand.

0. The contract: parameters, defaults, what it refuses, and its D-number in `ARCHITECTURE.md` §12. Write it
   before the code — the doc comment and the schema are copies of it.
1. `crates/datum-ops/src/lib.rs`: a variant of `Op` (`#[serde(tag = "op", rename_all = "snake_case",
   deny_unknown_fields)]`). Its doc comment reaches the agent as the schema description, so state what the
   operation takes and what it refuses.
2. The same file: the `eval_named` dispatch and the `impl Op` block — `kind`, `arity`, `reads_blank`,
   `resolve`. **Silent:** `reads_blank` is a `matches!`, so a new sheet operation is simply `false` there.
3. The geometry in `crates/datum-ops/src/<name>.rs`, re-exported from `lib.rs`. Exact predicates, no
   epsilons, `Degenerate`/`Unsupported` instead of a near body, determinism — the rules are in `AGENTS.md`;
   follow them, do not restate them here.
4. `crates/datum-model`: usually nothing, since `add` and `edit` take the op from `schemars::schema_for!(Op)`.
   **Silent holes** when the operation names sketch curves or works on sheet metal: renaming curves skips
   unknown ops (`session.rs`, a `match` ending in `_ => {}`), the sheet rule is substituted only for the ops
   listed in `levels.rs` (`_ => None`), and `add` carries per-op requirements (`extrude` needs `mode`).
5. `crates/datum-mcp`: nothing for an edit; a query tool is a `SHORTCUTS` entry in `lib.rs`.
6. `crates/datum-view`: an action in `ACTIONS` (fixed-length array) only if the operation starts on its own —
   otherwise it enters through a dispatcher such as E (D185). **Silent:** `reedit` in `app.rs` matches the ops
   it knows and falls through `_ => {}`, so a new op cannot be reopened from the tree until it is added there.
   Every label and error needs a string in `locales/ru.ftl` (D172).
7. Tests: an integration file `crates/datum-ops/tests/<op>.rs` (this crate has no `#[cfg(test)]` modules),
   a property test, and a reference model in `tests/models/` pinned with the `bless-hashes` skill.
8. `scripts/check.sh` (it includes `determinism-check`). For a user-visible operation a line in `TESTERS.md`
   ("Что нового" and "Для агента (MCP)"); `HANDOFF.md` per `AGENTS.md`.

Done when the operation runs from a JSON model, `list_commands` shows its schema, the window offers it (or
its dispatcher does), a reference model covers it, and `scripts/check.sh` passes.

Report in Russian: what was added, which silent place you checked and how, what is still open.
