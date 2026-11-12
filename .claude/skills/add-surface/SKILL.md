---
name: add-surface
description: Add a new surface type to Datum's B-rep, from math to STEP. Use when the kernel lacks a surface an operation needs (torus, cone, a swept wall, NURBS), or when a decision names one.
---

A surface is one fact in a chain the compiler mostly guides: a variant of `Surface` in
`crates/datum-brep/src/curve.rs` breaks every `match` until each place is filled. What the compiler cannot
decide is below.

0. The contract in `ARCHITECTURE.md` §12: is it exact in STEP, does it tessellate, does it unfold, and what
   an operation does with it — a refusal is a valid answer (D186 п. 1). Without the entry the surface leaks
   into places nobody agreed on.
1. `crates/datum-brep/src/curve.rs`: the data and its evaluation — axis, radii, frame, the point and
   `normal_at(p: Vec3)` — as `Torus` and `Wall` do. `datum-math` takes a new case only when the surface needs
   an exact predicate or an intersection (`implicit.rs`, `meet.rs`). Numerical rules (exact predicates,
   `libm`, no epsilons) are in `AGENTS.md`.
2. The same file: the variant, then follow the compiler through `plane`, `revolution`, `moved` and the rest.
   `Torus` and `Wall` are worked examples of "not a plane, not a surface of revolution".
3. `crates/datum-brep/src/tess.rs`: the mesh — the same flatness deviation as the others, and the same number
   of segments as the wall it meets, so seams do not crack (D147).
4. `crates/datum-brep/src/step.rs`: an exact entity (`TOROIDAL_SURFACE`, `CONICAL_SURFACE`), or no entity and
   a B-spline under `--approximate` with the reason recorded (D147). A surface that is neither is a refusal,
   never a silent approximation.
5. `crates/datum-brep/src/validate.rs`: what a face on it must satisfy; debug builds run `Body::validate()`
   on every result.
6. Unfolding: if it develops exactly, `datum-ops/src/unfold.rs` and `Flat` — a cone alone does (`cone_blank`,
   D207), a cone joined to other faces does not, a torus does not. Otherwise the operation refuses and says
   why (D186 — layers of the solver).
7. Operations: which ones accept it — see `add-op`. A boolean against it stays `Unsupported` until curved
   faces are handled (`ROADMAP.md`, M10), unless it is the local case of D185 п. 7.
8. Tests: a body with the surface in `crates/datum-brep/src/tests.rs`, a reference model in `tests/models/`
   pinned with `bless-hashes`, and the STEP read-back in `scripts/check.sh` (OCP).

Done when a model with the surface builds, tessellates without cracks, exports exact STEP, `scripts/check.sh`
passes, and the journal entry says what is refused.

Report in Russian: what was added, what it refuses, which places the compiler found and which you had to
decide.
