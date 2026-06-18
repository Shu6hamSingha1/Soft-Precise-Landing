---
name: Dual-platform development (Ubuntu + Windows)
description: User works on this project in parallel from both Ubuntu and Windows — check the live environment before assuming paths/tools
type: user
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
User runs this project from **both Ubuntu and Windows**, switching between them. The CLAUDE.md Environment section documents the Windows side (Git Bash / MSYS2, `/c/Program Files/GitHub CLI`), but the live session may be on Ubuntu. User also uses **both VS Code and Cursor** as editors — they may switch mid-session.

**How to apply:**
- Check the runtime environment (system prompt's `Platform:` field, `uname`, `/etc/os-release`) before recommending paths, tools, or installation steps. Don't assume Windows just because CLAUDE.md describes Windows.
- When configuring an editor (keybindings/settings), ask or check which editor is currently in use — VS Code and Cursor have separate config dirs.
- On **Ubuntu**:
  - VS Code config at `~/.config/Code/User/`
  - Cursor config at `~/.config/Cursor/User/`
  - `gh` typically at `/usr/bin/gh`, native paths like `/home/shubham/...`
- On **Windows / Git Bash**:
  - VS Code config at `%APPDATA%\Code\User\` (i.e. `/c/Users/<user>/AppData/Roaming/Code/User/` in MSYS)
  - Cursor config at `%APPDATA%\Cursor\User\`
  - `gh` at `/c/Program Files/GitHub CLI/gh`
- Files in the project tree are shared (likely via git, possibly a synced folder), so MATLAB / PX4 / Gazebo code is the same on both — only the dev environment differs.
