---
name: Notation subscript convention for plant inputs
description: Use _u (not _d) for torque/thrust plant inputs to avoid conflict with disturbance subscript _d; body-frame prefix B is mandatory on T and tau
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Torque and thrust outputs from the inner loop must use subscript `_u` (control input), not `_d` (which means disturbance in the dynamics equation).

- `\,^\mathcal{B}\boldsymbol{\tau}_u` — body-frame torque (control input)
- `\,^\mathcal{B}T_u` — body-frame thrust (control input), always with `\,^\mathcal{B}` prefix
- `\,^\mathcal{B}\boldsymbol{F}_u` — body-frame force (control input), already correct in dynamics

Signals where `_d` = "desired" are fine (no disturbance counterpart exists):
- `\,^\mathcal{I}\boldsymbol{a}_\text{d}` — desired inertial acceleration
- `\psi_\text{d}` — desired heading

**Why:** The dynamics equation uses `_d` for disturbances (`\boldsymbol{d}_h`, `\,^\mathcal{V}\boldsymbol{F}_\text{d}`). Using `_d` on torque/thrust creates ambiguity with the disturbance torque. The baseline inner loop (Section III-C) already used `\tau_u` correctly — the SO(3) section was the inconsistent one.

**How to apply:** When writing or reviewing any equation involving torque or thrust commands, always use `_u` subscript and include the `\,^\mathcal{B}` frame prefix. Check block diagram signal labels match.
