#!/usr/bin/env bash
# Builds datum.exe on the Windows side of WSL. Two environments, chosen by the
# Windows user name (no flag for that):
#   Simanov (work): mirror C:\Users\<user>\datum-src without .git — synced from a
#     committed branch via git archive + rsync --delete (target/ untouched), built
#     with --release.
#   simfali (home): git checkout C:\develop\datum, detached at the built commit,
#     built with --debug.
# Usage: win-build.sh [branch] [--test]   (--test also runs the pinned-hash check
# there, C4; branch defaults to the current one).
set -euo pipefail

WSL=$(git rev-parse --show-toplevel)
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

# Uncommitted WSL edits are not built: warn loudly instead of building stale code silently.
wsl_dirty=$(git -C "$WSL" status --porcelain --untracked-files=no)
[ -z "$wsl_dirty" ] || echo "warning: the WSL tree has uncommitted changes; they are NOT built" >&2

# cmd.exe refuses to start from a UNC cwd, hence the cd to /mnt/c first.
win_user=$( (cd /mnt/c && /mnt/c/Windows/System32/cmd.exe /C "echo %USERNAME%") 2>/dev/null | tr -d '\r' )
case "$win_user" in
  Simanov|simanov) mode=mirror ;;
  simfali)         mode=checkout ;;
  *) echo "unknown Windows user '$win_user': no build profile" >&2; exit 1 ;;
esac
echo "profile: $mode ($win_user)"

run_win() { /mnt/c/Windows/System32/cmd.exe /C "cd /d $1 && $2"; }

if [ "$mode" = mirror ]; then
  WIN=/mnt/c/Users/$win_user/datum-src
  WIN_DOS=$(wslpath -w "$WIN")
  [ -d "$WIN" ] || { echo "$WIN_DOS missing" >&2; exit 1; }
  [ -d "$WIN/target" ] || { echo "$WIN_DOS\\target missing: build once there first" >&2; exit 1; }

  # Mirror sync: extract the committed tree to a temp dir, rsync --delete into the
  # mirror (target/ excluded), drop the temp. Deleted-in-branch files must not linger.
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT
  git -C "$WSL" archive "$branch" | tar -x -C "$tmp"
  # git archive stamps files with the commit date; without this cargo may see every
  # source as older than its last build and silently skip the rebuild.
  find "$tmp" -exec touch {} +
  rsync -rtL --delete --exclude target/ "$tmp"/ "$WIN"/
  sha=$(git -C "$WSL" rev-parse --short "$branch")
  profile="cargo build -q -p datum-cli --release"
else
  WIN=${DATUM_WIN:-/mnt/c/develop/datum}
  WIN_DOS=$(wslpath -w "$WIN")
  gw() { git -C "$WIN" -c safe.directory="$WIN" "$@"; }

  # The Windows tree must be clean: nothing is discarded here.
  dirty=$(gw status --porcelain)
  [ -z "$dirty" ] || { echo "$WIN_DOS has uncommitted changes; commit or stash them there first" >&2; exit 1; }

  # Separate statements: a failed fetch must stop the script, not skip the checkout (set -e only
  # ignores a failure of a non-last command in an AND list).
  gw fetch --quiet "$WSL" "$branch"
  gw checkout --quiet --detach FETCH_HEAD
  sha=$(gw rev-parse --short HEAD)
  profile="cargo build -q -p datum-cli"
fi

log=$(mktemp)
if ! ( cd "$WIN" && /mnt/c/Windows/System32/cmd.exe /C "$profile" ) >"$log" 2>&1; then
  tail -n 40 "$log" >&2
  exit 1
fi

if [ "$test" = 1 ]; then
  if ! ( cd "$WIN" && /mnt/c/Windows/System32/cmd.exe /C "cargo run -q -p datum-cli -- determinism-check tests\models" ) >"$log" 2>&1; then
    tail -n 40 "$log" >&2
    echo "the pinned hashes differ on Windows (C4)" >&2
    exit 1
  fi
fi

echo "built $sha ($branch, $mode): $WIN_DOS\\target\\$( [ "$mode" = mirror ] && echo release || echo debug )\\datum.exe"
