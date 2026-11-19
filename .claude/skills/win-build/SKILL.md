---
name: win-build
description: Makes/refreshes the local Windows build of Datum (C:\develop\datum) — syncs the branch from this WSL repo and builds datum.exe with the Windows toolchain. Use when the author asks to make a build, refresh the Windows copy, or "see it" locally.
---

Run `scripts/win-build.sh [branch] [--test]` from `/home/simanov/develop/datum` (default branch: the WSL repo's current one). The script syncs the branch into `C:\develop\datum` (detached at the built commit) and builds `datum.exe` there; `--test` also runs the pinned-hash check (C4) on Windows.

Report: the branch synced, the build result, and the path `C:\develop\datum\target\debug\datum.exe` (the author runs it from Windows himself). If the script says the Windows tree is dirty, tell the author and stop — never discard his uncommitted changes. Do not push anywhere.
