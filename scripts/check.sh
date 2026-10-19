#!/usr/bin/env bash
# Single source of truth for pre-merge checks: used locally, by the /check skill and by CI.
# In CI (CI=true) formatting is verified; locally it is applied.
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ! -f Cargo.toml ]]; then
    echo "check: no Cargo workspace yet, nothing to check"
    exit 0
fi

cargo fmt --all ${CI:+-- --check}
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace

# The dev profile the tests just built (the kernel at opt-level 2): a release build of its own
# was a third compilation of the workspace, the longest step on Windows CI. Rust keeps IEEE
# float semantics at every opt level, so the hashes are the release ones.
if [[ -d apps/datum-cli && -d tests/models ]]; then
    cargo run -p datum-cli -- determinism-check tests/models
fi
