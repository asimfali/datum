#!/usr/bin/env bash
set -euo pipefail

die() { printf '%s\n' "$*" >&2; exit 1; }

[[ $# -ge 3 ]] || die 'Usage: scripts/pr.sh <branch> <title> <body-file> [-- <path>...]'
branch=$1
title=$2
body_file=$3
shift 3
if [[ $# -gt 0 ]]; then
    [[ $1 == -- && $# -gt 1 ]] || die 'Pass the paths to commit after --.'
    shift
fi
[[ -n $title && -s $body_file ]] || die 'Provide a title and a nonempty PR body file.'
[[ $branch != main && $branch != -* ]] || die 'Use a topic branch.'
git check-ref-format "refs/heads/$branch"
current_branch=$(git symbolic-ref --quiet --short HEAD) || die 'Check out main or the topic branch first.'
if [[ $current_branch != "$branch" ]]; then
    [[ $current_branch == main && $# -gt 0 ]] || die 'Use the checked-out topic branch or pass paths for a new branch from main.'
    if git show-ref --verify --quiet "refs/heads/$branch"; then
        die 'Check out the existing topic branch first.'
    fi
fi

repo=asimfali/datum
git fetch --no-tags origin +refs/heads/main:refs/remotes/origin/main
base=$(git rev-parse --verify refs/remotes/origin/main)
if [[ $current_branch == main ]]; then
    git merge-base --is-ancestor HEAD "$base" || die 'Local main contains commits outside origin/main.'
    git switch --no-track -c "$branch" "$base"
fi

if [[ $# -gt 0 ]]; then
    git --literal-pathspecs add -- "$@"
    if ! git --literal-pathspecs diff --cached --quiet -- "$@"; then
        git --literal-pathspecs commit --only -m "$title" -- "$@"
    fi
fi

head=$(git rev-parse --verify HEAD)
git merge-base --is-ancestor "$head" "$base" && die 'The topic branch has no commits ahead of origin/main.'
if git diff --quiet "$base...$head" --; then
    die 'The topic branch has no changes for the pull request.'
fi
if git diff --quiet "$base...$head" -- . ':(exclude,glob)**/*.md'; then
    echo 'Project checks skipped for a Markdown-only pull request.'
else
    # Check the exact commit independently of other work in the active tree.
    root=$(git rev-parse --show-toplevel)
    target_dir=${CARGO_TARGET_DIR:-"$root/target"}
    [[ $target_dir == /* ]] || target_dir="$root/$target_dir"
    check_tree=$(mktemp -d)
    git worktree add --detach "$check_tree" "$head"
    trap 'git worktree remove --force "$check_tree"' EXIT
    (cd "$check_tree" && CARGO_TARGET_DIR="$target_dir" bash scripts/check.sh)
    git -C "$check_tree" diff --quiet HEAD -- || die 'Checks modified tracked files. Run scripts/check.sh on the topic branch and commit the fixes.'
    git worktree remove --force "$check_tree"
    trap - EXIT
fi

[[ $(git rev-parse --verify "refs/heads/$branch") == "$head" ]] || die 'The topic branch changed during checks. Run again.'
git push -u origin "refs/heads/$branch:refs/heads/$branch"
existing=$(gh pr list --repo "$repo" --base main --head "$branch" --state open \
    --json url,isCrossRepository --jq 'map(select(.isCrossRepository | not))[0].url // empty')
if [[ -n $existing ]]; then
    printf '%s\n' "$existing"
else
    gh pr create --repo "$repo" --base main --head "$branch" --title "$title" --body-file "$body_file"
fi
