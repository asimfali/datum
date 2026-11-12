---
name: wsl-image-paste
description: Make image paste from the Windows clipboard work in OpenCode's TUI on WSL. Use when Ctrl+V, Ctrl+Shift+V or Shift+Insert do nothing with a Win+Shift+S screenshot, or when setting up a new WSL machine.
---

# Image paste on WSL

OpenCode v2 reads the clipboard through OpenTUI's native backend, which talks to Wayland or X11 only. WSL has neither when WSLg is off (`guiApplications=false`), and Windows Terminal sends an empty bracketed paste for image-only clipboards. Text pastes fine because the terminal passes it as text; images silently do nothing.

Do not enable WSLg or install an X server without the user's word.

## Diagnose

```bash
echo "DISPLAY=[$DISPLAY] WAYLAND=[$WAYLAND_DISPLAY]"
ls /run/user/$UID/wayland-0 /tmp/.X11-unix 2>&1
timeout 5 wl-paste --no-newline 2>&1 | head -2
timeout 5 xclip -selection clipboard -o 2>&1 | head -2
grep -i guiApplications /mnt/c/Users/*/.wslconfig 2>/dev/null
```

If text pastes but images do not, and there is no Wayland/X11 socket, use the path below.

## Fix without a GUI: `shotd`

After `Win+Shift+S` the watcher saves the image to a file and puts the **path** on the clipboard (the image stays too). OpenCode turns a pasted path into an attachment: `[Image 1]`.

Install on a new machine (from this skill directory):

```bash
mkdir -p ~/.local/bin ~/.local/share/shotd
cp scripts/shotd ~/.local/bin/shotd
cp scripts/watch.ps1 ~/.local/share/shotd/watch.ps1
chmod +x ~/.local/bin/shotd
~/.local/bin/shotd start
```

Workflow: `Win+Shift+S` → wait about a second → `Ctrl+V` in the OpenCode prompt.

Maintenance:

```bash
shotd status   # is the watcher running
shotd stop     # stop it
shotd once     # convert the current clipboard image once
shotd start    # start again (needed after wsl --shutdown or reboot)
```

Files land in `%LOCALAPPDATA%\Temp\opencode-shots\` (about `/mnt/c/Users/<name>/AppData/Local/Temp/opencode-shots/` in WSL); shots older than 7 days are removed on start.

## Verify

1. Take a screenshot with `Win+Shift+S`.
2. `powershell.exe -NoProfile -Command "Get-Clipboard -Raw"` prints a `.../shot-*.png` path.
3. Paste it into the prompt with `Ctrl+V`; the prompt shows `[Image 1]`.

## Caveats

- The clipboard holds both the image and the path, so text-preferring apps (Word) may paste the path. Stop the watcher with `shotd stop` when that matters.
- The watcher is one background `powershell.exe` (about 50 MB). Restart it after `wsl --shutdown`; add `shotd start` to `~/.bashrc` to make that automatic.
- Upstream fix for the root cause is PR #57470 (Windows clipboard via `powershell.exe`); once it ships, this skill is unnecessary.
