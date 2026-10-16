# Datum

**Design Automation Toolkit for Universal Modeling**
<br><sub>under the hood: *Distributed Architecture for Topology-based Unified Modeling*</sub>

A parametric solid-modeling kernel built from scratch in Rust to use **every CPU core** while staying **bit-for-bit deterministic**.

> **Status:** early proof of concept, pre-M0. Nothing is usable yet.

## Why

Mainstream mechanical CAD systems rebuild models on a single thread. Their kernels and feature histories were designed decades ago around mutable shared state and a linear history. Engineers pay for that every day: slow rebuilds, references that break after an early edit, rebuild failures nobody can predict.

Datum is designed around a different set of assumptions:

- **Immutable bodies.** An operation never modifies its input; it produces a new body. Unchanged parts are shared.
- **A dependency graph, not a list.** Independent features rebuild in parallel. Only what actually changed is recomputed.
- **Stable topology IDs.** Every face, edge and vertex is named by its origin ("side face of extrusion X from sketch edge Y"), not by a counter. References survive edits, and results do not depend on thread count.
- **Symbolic intersections.** Boolean operations match intersection vertices by key, not by distance tolerance, so faces can be split independently on different cores and stitched back exactly.
- **No modeling box.** No absolute tolerances, so there is no "1 km cube": a site plan and an M6 bolt live in one model, with local frames and georeferencing.
- **Agent-native.** Every operation is a JSON command with a generated schema; the same commands drive the CLI, the UI and an MCP server. Models are stored as a JSON feature graph, and geometry is rebuilt deterministically.
- **Honest failure.** An unsupported degenerate case returns an error instead of a plausible-looking wrong body.

The long-term goal is an open CAD system for professional engineers, built around what they actually need.

## What the proof of concept must show

| # | Claim | Measured by |
|---|---|---|
| C1 | Feature-graph rebuild scales with cores on wide models | N independent cuts, 1–16 threads |
| C2 | Editing a parameter at the end of a long history is instant | 200-feature chain, < 10 ms overhead |
| C3 | A single Boolean operation speeds up internally | 10k–100k-face bodies, 1–16 threads |
| C4 | Results are bit-identical for any thread count | Canonical body hash at 1/4/16 threads |

All comparisons are within Datum itself: sequential vs parallel.

## Roadmap

| Stage | Scope |
|---|---|
| M0 | Workspace, math, half-edge B-rep, primitives, validation, volume, canonical hash, STL/OBJ export, CLI |
| M1 | Feature graph: memoization, early cutoff, parallel scheduler; JSON model format; MCP server; determinism tests (C1, C2, C4) |
| M2 | Parallel Boolean operations on polyhedra (C3) |
| M3 | 2D sketches with a constraint solver; extrude, cut, patterns |
| M4 | Viewer with live parameter editing |
| M5 | Cylindrical surfaces |

## Documentation

Design documents are currently in Russian:

- [`ARCHITECTURE.md`](ARCHITECTURE.md): goals, data structures, algorithms, decision log
- [`CLAUDE.md`](CLAUDE.md): project rules (determinism, geometry, code style)
- [`PAINS.md`](PAINS.md): real engineering pain points that drive priorities

## Development

```bash
scripts/check.sh   # fmt, clippy, tests, determinism check
```

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
