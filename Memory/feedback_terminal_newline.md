---
name: Terminal newline shortcut is Alt+Enter (Cursor)
description: In Cursor's integrated terminal, Alt+Enter inserts a newline in Claude Code's chat input; do not try to remap Shift+Enter or Ctrl+J
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
In Cursor's integrated terminal on Ubuntu, **`Alt+Enter`** works out-of-the-box to insert a newline in Claude Code's chat input. The user prefers this and does not want custom keybinding overrides.

**Why:** We previously spent a long session trying to remap `Shift+Enter` and `Ctrl+J` via `~/.config/Cursor/User/keybindings.json` (and the Claude Code side `~/.claude/keybindings.json`); the overrides never fired in the terminal context. Alt+Enter already worked the whole time and is the user's preferred shortcut.

**How to apply:**
- If the user asks "how do I insert a newline" in Cursor's terminal, answer **`Alt+Enter`** directly. Don't suggest Shift+Enter, don't suggest Ctrl+J, don't suggest editing keybindings.json.
- The `~/.claude/keybindings.json`, `~/.config/Cursor/User/keybindings.json`, and `~/.config/Code/User/keybindings.json` are all intentionally empty/absent — don't repopulate them with newline bindings.
- `Ctrl+G` (default external-editor shortcut in Claude Code) is also a valid fallback for very long multi-line input.
- This is Cursor-specific; in plain VS Code or other terminals the situation may differ — re-check before assuming.
