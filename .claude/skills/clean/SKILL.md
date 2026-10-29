---
name: clean
description: Free disk space taken by Datum's build directory (target/) without a full rebuild. Use when the user asks to clean target, free space, or says the build takes too much disk.
---

Run from the repository root. Report sizes before and after.

1. `du -sh target` — size before.
2. Remove incremental caches: `rm -rf target/debug/incremental target/release/incremental`. They are the largest stale part and are rebuilt on demand at little cost.
3. Remove stale artifacts of old builds: `cargo sweep --time 3` (artifacts not used for 3 days). If `cargo sweep` is missing, say so and suggest `cargo install cargo-sweep`; do not install it without the user's word.
4. Keep `target/criterion`: it holds benchmark baselines for regression checks.
5. `du -sh target` — size after.

A full `cargo clean` only on the user's explicit request: the next `scripts/check.sh` then rebuilds everything from scratch.

Report in Russian, one or two lines: before → after, what was removed.
