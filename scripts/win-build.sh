#!/usr/bin/env bash
# Mirrors a committed branch from this WSL repo into the Windows checkout (C:\develop\datum,
# left detached at the built commit) and builds datum.exe there with the Windows toolchain.
# Usage: win-build.sh [branch] [--test]   (--test also runs the pinned-hash check there, C4).
set -euo pipefail

WSL=$(git rev-parse --show-toplevel)
WIN=${DATUM_WIN:-/mnt/c/develop/datum}
WIN_DOS=$(wslpath -w "$WIN")
branch=""
test=0
for arg in "$@"; do
  case "$arg" in
    --test) test=1 ;;
    *) branch=$arg ;;
  esac
done
[ -n "$branch" ] || branch=$(git -C "$WSL" branch --show-current)
[ -n "$branch" ] || { echo "detached HEAD in WSL: pass a branch name" >&2; exit 1; }

gw() { git -C "$WIN" -c safe.directory="$WIN" "$@"; }

# The Windows tree must be clean: nothing is discarded here.
dirty=$(gw status --porcelain)
[ -z "$dirty" ] || { echo "$WIN_DOS has uncommitted changes; commit or stash them there first" >&2; exit 1; }

# Uncommitted WSL edits are not built: warn loudly instead of building stale code silently.
wsl_dirty=$(git -C "$WSL" status --porcelain --untracked-files=no)
[ -z "$wsl_dirty" ] || echo "warning: the WSL tree has uncommitted changes; they are NOT built" >&2

# Separate statements: a failed fetch must stop the script, not skip the checkout (set -e only
# ignores a failure of a non-last command in an AND list).
gw fetch --quiet "$WSL" "$branch"
gw checkout --quiet --detach FETCH_HEAD
sha=$(gw rev-parse --short HEAD)

run_win() { /mnt/c/Windows/System32/cmd.exe /C "cd /d $WIN_DOS && $1"; }
log=$(mktemp)
if ! run_win "cargo build -q -p datum-cli" >"$log" 2>&1; then
  tail -n 40 "$log" >&2
  exit 1
fi
if [ "$test" = 1 ]; then
  if ! run_win "cargo run -q -p datum-cli -- determinism-check tests/models" >"$log" 2>&1; then
    tail -n 40 "$log" >&2
    echo "the pinned hashes differ on Windows (C4)" >&2
    exit 1
  fi
fi
echo "built $sha ($branch): $WIN_DOS\\target\\debug\\datum.exe"
