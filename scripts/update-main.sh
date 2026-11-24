#!/usr/bin/env bash
set -euo pipefail

git fetch --no-tags origin +refs/heads/main:refs/remotes/origin/main

local_main=$(git rev-parse --verify refs/heads/main)
remote_main=$(git rev-parse --verify refs/remotes/origin/main)
if [[ $local_main == "$remote_main" ]]; then
    printf 'main is up to date (%s).\n' "$(git rev-parse --short "$local_main")"
    exit 0
fi
if ! git merge-base --is-ancestor "$local_main" "$remote_main"; then
    echo 'main has local commits outside origin/main; cannot fast-forward.' >&2
    exit 1
fi

if [[ $(git symbolic-ref --quiet HEAD || :) == refs/heads/main ]]; then
    git merge --ff-only --no-autostash "$remote_main"
else
    # Fetch refuses to update a branch checked out in another worktree.
    git fetch --no-tags --no-write-fetch-head . "$remote_main:refs/heads/main"
fi

printf 'Updated main to %s.\n' "$(git rev-parse --short refs/heads/main)"
