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

## Status

The proof of concept is complete: stages M0–M5 are done, and C1–C4 hold (Ryzen 5 3600 under WSL, 6 logical cores; 1 thread vs 6):

| # | Result |
|---|---|
| C1 | 1000 independent bodies: 230 → 61 ms (×3.8) |
| C2 | Editing the last of 200 features: 0.14 ms over the operation itself |
| C3 | 10 000-hole plate by one Boolean: 1.55 → 0.50 s (×3.1); terrain of 10⁶ triangles with a pit: 3.38 → 1.28 s (×2.6); 10 000 holes in a rolled sheet: 432 → 172 ms (×2.5) |
| C4 | Hashes identical on 1/4/16 threads and between Linux and Windows |

Coordinates 10⁷ m from the origin (in mm) keep every topology: no modeling box.

| Stage | Scope |
|---|---|
| M0 | Workspace, math, half-edge B-rep, primitives, validation, volume, canonical hash, STL/OBJ export, CLI |
| M1 | Feature graph: memoization, early cutoff, parallel scheduler; JSON model format; MCP server; determinism tests (C1, C2, C4) |
| M2 | Parallel Boolean operations on polyhedra (C3) |
| M3 | 2D sketches with a constraint solver; extrude, cut, patterns; parameters and expressions |
| M4 | Viewer with live parameter editing; large scenes on modest GPUs |
| M5 | Arcs and cylinders; cuts by sketch profiles; perforation of a rolled sheet in its development; flat blank as a graph node, DXF export |

## Documentation

Design documents are currently in Russian:

- [`ARCHITECTURE.md`](ARCHITECTURE.md): goals, data structures, algorithms, decision log
- [`CLAUDE.md`](CLAUDE.md): project rules (determinism, geometry, code style)
- [`PAINS.md`](PAINS.md): real engineering pain points that drive priorities

## Try it

```bash
cargo run --release -p datum-cli -- examples/rolled_perforation.json   # the viewer on a model
cargo run --release -p datum-cli -- dxf examples/rolled_perforation.json --out blank.dxf
```

The viewer opens any model dropped on its window; parameters are edited on the side and the model rebuilds live. [`examples/`](examples) and [`tests/models/`](tests/models) hold models to start from; the format is a JSON feature graph (see `ARCHITECTURE.md` §6).

## Development

```bash
scripts/check.sh   # fmt, clippy, tests, determinism check
cargo run --release -p datum-cli -- build tests/models/bracket.json --out bracket.stl
cargo run --release -p datum-cli -- mcp   # MCP server on stdio (.mcp.json registers it for Claude Code)
```

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
