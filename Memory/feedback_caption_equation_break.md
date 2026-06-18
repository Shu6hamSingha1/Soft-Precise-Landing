---
name: Caption equation breaks — close/reopen math, no \allowbreak
description: For long inline math in captions, allow line breaks at commas by closing and reopening math segments (`,$ $...`) instead of inserting `\allowbreak`. User rejected `\allowbreak` 2026-04-29.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** When an inline math expression in a `\caption{}` is too wide for a single column and TeX needs help finding break points, do NOT insert `\allowbreak` (or `\penalty 0`) inside `$...$`. Instead, segment the math at natural punctuation boundaries (commas inside vectors, between vector components) by closing and reopening math:

```latex
% wrong (per 2026-04-29 user pushback)
$\,^\mathcal{I}\boldsymbol{v}_\text{t}{=}[\lambda,\allowbreak\lambda,\allowbreak 0.1\cos(0.5t)]^\top$

% right
$\,^\mathcal{I}\boldsymbol{v}_\text{t}{=}[\lambda,$ $\lambda,$ $0.1\cos(0.5t)]^\top$
```

**Why:** The user wants automatic line breaks at the *end of each sentence* (between Cases joined by `;`) and at natural punctuation inside the equation, not arbitrary penalty insertions. Closing/reopening `$...$` lets TeX treat the inter-segment space as an ordinary breakable space, so it breaks only when needed. The math-mode comma stays inside math, preserving its thinmuskip spacing; visually identical to the un-segmented form when the equation fits on one line.

**How to apply:**
- Caption template for multi-component vectors: keep each comma attached to the math token to its left (`λ,$`), then a normal `~` *or* a regular space, then re-open math with the next token (`$\lambda,...`).
- Don't use `\,$ $\,$ $` (extra `\,` thinspaces) — a plain space is what allows TeX to break.
- Don't use `${}$` empty-group bridges either — those defeat the break point.

**Where this came up (2026-04-29):**
- `Soft_Precise_Landing/results.tex` Fig. 4 caption (`speed envelope: figure`): Cases 2–5 target-velocity profiles, with Cases 4 and 5 long enough that the original `[\lambda,\lambda,0.1\cos(0.5t)]^\top$` chain produced a ragged right edge. Fixed by segmenting at every comma.
- Companion supplement captions (`comparison_multi_speed_<traj>.pdf` for Cases 2–4) all carry the same single-equation per caption — they fit comfortably and do not need segmentation. Apply only when the caption visibly overflows or wastes margin.
