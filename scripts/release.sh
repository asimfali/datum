#!/usr/bin/env bash
set -euo pipefail

die() { printf '%s\n' "$*" >&2; exit 1; }

[[ $# == 1 && $1 =~ ^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$ ]] || die 'Usage: scripts/release.sh <X.Y.Z>'
version=$1
tag="v$version"
branch="release/$version"
repo=asimfali/datum
root=$(git rev-parse --show-toplevel)
ref="refs/tags/$tag"

workspace_version() {
    git show "$1:Cargo.toml" | python3 -c 'import sys, tomllib; print(tomllib.loads(sys.stdin.read())["workspace"]["package"]["version"])'
}

git fetch --no-tags origin +refs/heads/main:refs/remotes/origin/main
main=$(git rev-parse --verify refs/remotes/origin/main)
remote_tag=$(git ls-remote --tags origin "$ref")
if [[ -n $remote_tag ]]; then
    git fetch --no-tags origin "$ref"
    commit=$(git rev-parse --verify 'FETCH_HEAD^{commit}')
    git merge-base --is-ancestor "$commit" "$main" || die "$tag does not point to a commit on origin/main."
else
    commit=$main
fi
current_version=$(workspace_version "$commit")

if [[ -z $remote_tag && $current_version != "$version" ]]; then
    latest=$(git ls-remote --tags --refs --sort=-version:refname origin 'refs/tags/v*' |
        awk '$2 ~ /^refs\/tags\/v[0-9]+\.[0-9]+\.[0-9]+$/ && !found++ { print substr($2, 12) }')
    python3 - "$version" "$current_version" "$latest" <<'PY'
import sys

requested, current, latest = sys.argv[1:]
key = lambda text: tuple(map(int, text.split(".")))
if key(requested) <= key(current) or (latest and key(requested) <= key(latest)):
    sys.exit("The new release version must be higher than main and the latest tag.")
PY
    existing=$(gh pr list --repo "$repo" --base main --head "$branch" --state open \
        --json url,isCrossRepository --jq 'map(select(.isCrossRepository | not))[0].url // empty')
    if [[ -z $existing ]]; then
        tree=$(mktemp -d)
        if git show-ref --verify --quiet "refs/heads/$branch"; then
            git worktree add "$tree" "$branch"
        else
            git worktree add -b "$branch" "$tree" "$main"
        fi
        trap 'git worktree remove --force "$tree"' EXIT
        target_dir=${CARGO_TARGET_DIR:-"$root/target"}
        [[ $target_dir == /* ]] || target_dir="$root/$target_dir"
        (
            cd "$tree"
            export CARGO_TARGET_DIR="$target_dir"
            python3 - "$version" <<'PY'
from pathlib import Path
import re
import sys

version = sys.argv[1]
manifest, notes = Path("Cargo.toml"), Path("TESTERS.md")
heading = f"## Что нового в {version}"
notes_text = notes.read_text()
development_heading = f"## В разработке для {version}"
if heading not in notes_text.splitlines():
    if development_heading not in notes_text.splitlines():
        sys.exit(f"Add the release notes for {version} to TESTERS.md on main.")
    notes_text = notes_text.replace(development_heading, heading, 1)

manifest_text, count = re.subn(
    r'(?m)(^\[workspace\.package\][^\[]*?^version\s*=\s*)"[^"]+"',
    lambda match: f'{match[1]}"{version}"', manifest.read_text(), count=1,
)
if count != 1:
    sys.exit("No workspace version found in Cargo.toml.")
manifest.write_text(manifest_text)
notes.write_text(notes_text)
PY
            cargo update --workspace --offline
            body=$(mktemp .release-pr.XXXXXX)
            printf 'Подготовка Datum %s к выпуску. Обновлены версия workspace, Cargo.lock и заметки для тестировщиков.\n' "$version" >"$body"
            bash "$root/scripts/pr.sh" "$branch" "Release $version" "$body" -- Cargo.toml Cargo.lock TESTERS.md
        )
    else
        printf '%s\n' "$existing"
    fi
    printf 'After the author merges the release PR, run scripts/release.sh %s again to publish.\n' "$version"
    exit 0
fi

[[ $current_version == "$version" ]] || die "$tag contains workspace version $current_version, expected $version."
if [[ -z $remote_tag ]]; then
    if git show-ref --verify --quiet "$ref"; then
        [[ $(git rev-parse "$ref^{commit}") == "$commit" ]] || die "Local $tag points to a different commit."
    else
        git tag -a "$tag" "$commit" -m "Datum $tag"
    fi
    git push origin "$ref:$ref"
fi

release_url() {
    gh release view "$tag" --repo "$repo" --json url,isDraft --jq 'select(.isDraft | not) | .url'
}

if url=$(release_url 2>/dev/null) && [[ -n $url ]]; then
    printf '%s\n' "$url"
    exit 0
fi

for ((attempt = 0; attempt < 24; attempt++)); do
    run=$(gh run list --repo "$repo" --workflow release.yml --branch "$tag" --commit "$commit" \
        --event push --limit 1 --json databaseId --jq '.[0].databaseId // empty')
    if [[ -n $run ]]; then
        gh run watch "$run" --repo "$repo" --interval 15 --exit-status
        url=$(release_url)
        [[ -n $url ]] || die "Workflow succeeded, but $tag has no published release."
        printf '%s\n' "$url"
        exit 0
    fi
    sleep 5
done
die "The release workflow for $tag is not visible yet. Run this command again to check publication."
