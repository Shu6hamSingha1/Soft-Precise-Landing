---
name: Don't overclaim — write only what the theorems actually prove
description: When describing what MDF-ASMC accomplishes, never write "enforces transient specification", "finite-time bound on v_rel", or similar phrasing that would require a theorem we don't have. Theorems are: Theorem 1 (h_e funnel invariance), Theorem 2 (yaw UUB), Corollary 1 (kinematic visibility). Soft-touchdown is *empirical* (25/25 in simulation), not formally proven. Locked 2026-04-30.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** Before writing any sentence in the manuscript that claims MDF-ASMC enforces / proves / guarantees a property, check it against the actual theorem statements:

| Claim | Status | Where it lives |
|---|---|---|
| `\|h_e(t)\| ≤ \|p_2(t)\|` for all `t ≥ 0` (optic-flow funnel invariance) | **Theorem 1** | §III.C of main paper |
| Yaw error UUB: `σ_α` enters `{V_α ≤ Θ_α}` and stays | **Theorem 2** | §III.C |
| Target visibility: feature points stay inside FoV for all `t ≥ 0` | **Corollary 1** | §III.C |
| `\|v_rel(t_touch)\| ≤ 0.20 m/s` (soft-touchdown criterion) | **Empirical only** | 25/25 simulation in §IV |

**Phrases to avoid (not formally proven):**
- "enforces the *transient* specification required for soft touchdown"
- "guarantees a finite-time bound on the relative velocity"
- "proves soft touchdown"
- "ensures `\|v_rel\| → 0` at the touchdown instant"
- "drives the relative velocity to zero in finite time"
- Anything implying a closed-form bound on `\|v_rel\|` at touchdown.

**Phrases that ARE accurate:**
- "renders the optic-flow error globally uniformly ultimately bounded" (matches Theorem 1)
- "drives the image-orientation error to a uniform ultimate bound" (matches Theorem 2)
- "propagates optic-flow invariance to a closed-loop target-visibility guarantee" (matches Corollary 1)
- "demonstrates a 25/25 soft-precise landing rate across the simulated envelope" (matches §IV)

**Why this rule exists:** A reviewer reading "rarely enforce the transient specification required for soft touchdown" in §I, then looking for the corresponding theorem in §III, will fault the paper if no such theorem exists. The theorems we have are about `h_e`, `α`, and visibility — not about a finite-time bound on `\|v_rel\|`.

**The connection that *informally* exists** (worth understanding but **not** worth claiming as a theorem):
$$\|v_\text{rel}(t_\text{touch})\| = \|h(t_\text{touch})\| \cdot z_\text{f} \le (\|h_d\| + \|p_2\|) \cdot z_\text{f}$$
A bound on `\|v_rel\|` at touchdown could be derived from `\|h_d\|`, `\|p_{2_∞}\|`, and `z_f`, but the paper doesn't formalise this as a theorem. Treat it as a design rationale, not a guarantee.

**Where this came up (2026-04-30):**
- A draft of §I had "rarely enforce the *transient* specification required for soft touchdown", which implied MDF-ASMC enforces such a transient specification. The user pushed back; the line was reworded to "regulate image-position features without imposing a prescribed envelope on the optic-flow error that governs touchdown speed" (Option C′).
- I followed up suggesting "finite-time soft touchdown" wording; the user pushed back again because we don't use finite-time stability theory at all (we use UUB / funnel invariance).

**Related conventions:**
- `feedback_proof_placement_convention.md` — main-paper proof = Lyapunov candidate + `dV` bound + UUB conclusion; full algebra in §S2.
- `feedback_soft_precise_definition_location.md` — soft = 3-D `‖v_rel‖` ≤ 0.20 m/s, defined only in §I.
- `feedback_landing_marker_convention.md` — soft criterion is 3-D, not vertical-only.
