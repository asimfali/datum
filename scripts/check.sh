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

# STEP read back by OpenCASCADE (D130): valid solids, the validation properties met. Needs Python
# with OCP (`pip install cadquery-ocp`, or DATUM_OCP_PYTHON naming a Python that has it); CI on
# Linux installs it, elsewhere it is skipped. Files up to 1 MB: OCCT reads a large perforation
# for minutes.
python=${DATUM_OCP_PYTHON:-python3}
if [[ -d apps/datum-cli && -d tests/models ]] && "$python" -c "import OCP" 2>/dev/null; then
    out=$(mktemp -d)
    trap 'rm -rf "$out"' EXIT
    cargo build -q -p datum-cli
    for m in tests/models/*.json; do
        # B-splines where STEP has no exact type (D147): OCCT checks them too. A model with
        # levels (D152): each part and variant; its frame, if it holds no body, is not exported.
        levels=$("$python" -c 'import json, sys; print("\n".join(json.load(open(sys.argv[1], encoding="utf-8")).get("levels", {})))' "$m")
        if [[ -z $levels ]]; then
            target/debug/datum build "$m" --out "$out/$(basename "$m" .json).step" --approximate >/dev/null
        fi
        while IFS= read -r level; do
            [[ -n $level ]] && target/debug/datum build "$m" --level "$level" --out "$out/$(basename "$m" .json) [$level].step" --approximate >/dev/null
        done <<< "$levels"
    done
    find "$out" -name '*.step' -size -1024k -print0 | sort -z | xargs -0 "$python" scripts/step_check.py
elif [[ ${CI:-} == true && $(uname) == Linux ]]; then
    echo "check: CI on Linux needs OCP for the STEP read-back" >&2
    exit 1
else
    echo "check: no OCP (pip install cadquery-ocp), STEP read-back skipped"
fi
