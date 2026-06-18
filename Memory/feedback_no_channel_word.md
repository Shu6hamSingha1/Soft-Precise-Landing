---
name: Avoid the word "channel" — use axis / loop / component instead
description: User dislikes "channel" (informal, borrowed from communication theory); replace with "axis" for per-coordinate subsystems, "loop" for closed-loop structures, "component" for vector-element indexing
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The rule (locked 2026-04-24):**

Do not use the word "channel" anywhere in `manuscript.tex`, `control_formulation.tex`, `results.tex`, `supplemental.tex`, or any figure/table caption. Replace context-specifically:

- **Per-coordinate subsystem** (x, y, z, lateral, vertical, horizontal) → **"axis"** / **"axes"**
  - "vertical channel follows" → "vertical axis follows"
  - "lateral channels" → "lateral axes"
  - "$y$-channel gains" → "$y$-axis gains"
  - "horizontal-channel closed loop" → "horizontal-axis closed loop"
  - "every channel is Hurwitz" → "every axis is Hurwitz"

- **Closed-loop control structure** (yaw, attitude, outer, inner) → **"loop"** / **"control loop"**
  - "yaw channel decouples" → "yaw control loop decouples"
  - For compounds where "yaw control loop" is too verbose (e.g. figure captions), use the most specific role-based term, e.g. "yaw adaptive gain" instead of "yaw-loop gain".

- **Vector-element / component indexing** ($\zeta_{2_k}$ with $k\in\{1,2,3\}$) → **"component"** / **"axis-wise"**
  - "bound each channel by ..." → "bound each component by ..."

**Why:** User flagged 2026-04-24 that "channel" is informal — it's borrowed from communication/signal-processing terminology and reads as imprecise in modern control-engineering writing. Reviewer-facing prose should use the more rigorous control-theory vocabulary.

**How to apply:**
- Before introducing "channel" in any new prose, pick the right replacement based on the categorisation above.
- For ambiguous cases (e.g., "the yaw subsystem"), consider also "loop" or "subsystem" — but never "channel".
- This rule applies to figure captions, theorem statements, proofs, and prose alike.
- The Drafts/ directory still contains old "channel" wording — leave it alone (those are not active manuscript files).

**Established convention now in active paper (2026-04-24):**
- Outer loop / inner loop / outer adaptive law (translational) — already uses "loop"
- Yaw control loop (for the closed-loop yaw ASMC subsystem)
- Lateral axes / vertical axis / $z$-axis / $y$-axis (for per-coordinate subsystems)
- Horizontal-axis dynamics / horizontal-axis closed loop (for compound structures)
- Each component (for vector-element indexing in sliding-surface anti-windup)
- Yaw adaptive gain $\kappa_\text{a}(t)$ (figure caption convention)
