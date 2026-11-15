#!/usr/bin/env bash
# Syncs a branch from this WSL repo into the local Windows checkout (C:\develop\datum)
# and builds datum.exe there with the Windows toolchain. Usage: win-build.sh [branch].
set -euo pipefail
WSL=/home/simfali/develop/datum
WIN=/mnt/c/develop/datum
branch="${1:-$(git -C "$WSL" branch --show-current)}"

# The Windows tree must be clean: nothing is discarded here.
dirty=$(git -C "$WIN" -c safe.directory=/mnt/c/develop/datum status --porcelain | head -1)
if [ -n "$dirty" ]; then
  echo "C:\\develop\\datum has uncommitted changes; commit or stash them there first" >&2
  exit 1
fi

# Bring the branch over. When it is checked out in Windows, git refuses to move it by fetch:
# fetch to a temp ref and fast-forward instead.
current=$(git -C "$WIN" -c safe.directory=/mnt/c/develop/datum branch --show-current)
if [ "$current" = "$branch" ]; then
  git -C "$WIN" -c safe.directory=/mnt/c/develop/datum fetch "$WSL" "$branch" &&
    git -C "$WIN" -c safe.directory=/mnt/c/develop/datum merge --ff-only FETCH_HEAD
else
  git -C "$WIN" -c safe.directory=/mnt/c/develop/datum fetch "$WSL" "$branch:$branch"
  git -C "$WIN" -c safe.directory=/mnt/c/develop/datum switch "$branch"
fi

/mnt/c/Windows/System32/cmd.exe /C "cd /d C:\\develop\\datum && cargo build -p datum-cli"
echo "built: C:\\develop\\datum\\target\\debug\\datum.exe (branch $branch)"
