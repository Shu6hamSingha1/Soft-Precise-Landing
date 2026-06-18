---
name: Trajectory naming convention
description: Use Case numbers (Case 1–5) instead of trajectory names; "static target" is OK but "linear target" is wrong — use "linear target trajectory"
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Refer to trajectories by case number (Case 1=Static, Case 2=Linear, Case 3=Sinusoidal, Case 4=Lissajous, Case 5=Circular) throughout manuscript and supplement. Trajectory names appear only in parenthetical clarification.

**Why:** User wants uniform referencing across all text, tables, figures, and captions. "Linear target" or "circular target" is logically incorrect — the target isn't linear, its trajectory is.

**How to apply:**
- Body text: "Case 2" or "Cases 2 and 5"; never bare "Linear" or "Circular" as trajectory labels
- Tables with space: "Case N: Name" (e.g., comparison table); tables without: "Case N" only
- Figure captions: "Case 1 (static target)" — target itself is static, so this is correct; "Case N (X target trajectory)" for all moving cases
- Python plot titles: same pattern; "Lissajous" stays capitalized (proper noun), others lowercase in parenthetical
- Natural language like "landed on a static target" is fine — don't capitalize as if it's a label
