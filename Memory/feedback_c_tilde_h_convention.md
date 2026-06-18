---
name: $\tilde{\boldsymbol{c}}_h$ vs $\boldsymbol{c}_h$ — pre/post desired-flow-subtraction kinematic coupling
description: Locked 2026-05-06; updated 2026-05-07. The kinematic-coupling symbol is split into two: $\tilde{\boldsymbol{c}}_h$ (used in eq `h dot: equation` of §II.B.2) and $\boldsymbol{c}_h$ (used in eq `h_e_dot_1: equation` of §III.A.2), related by $\boldsymbol{c}_h \triangleq \tilde{\boldsymbol{c}}_h - \dot{\boldsymbol{h}}_\text{d}$. Supplement §S1-B gives explicit forms of both. NOTE: Earlier framing called eq h dot the "natural plant" but as of 2026-05-07 eq h dot is rewritten in closed-loop form using $\,^\mathcal{I}\boldsymbol{a}_\text{d}$ (Assumption 2 invoked), so the "natural-plant" label no longer applies; the $\tilde{c}_h$ vs $c_h$ split survives because it's just about whether $-\dot h_d$ has been folded in.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** The kinematic-coupling term in the optic-flow dynamics has TWO distinct forms with distinct symbols:

| Symbol | Eq label | Where it appears | Form |
|---|---|---|---|
| $\tilde{\boldsymbol{c}}_h$ | `h dot: equation` (eq 7, §II.B.2) | natural-plant $\dot{\boldsymbol{h}}$ | cross products of $\boldsymbol{w}, \dot{\boldsymbol{w}}, \boldsymbol{s}, \boldsymbol{h}$ — **NO** $-\dot{\boldsymbol{h}}_\text{d}$ term |
| $\boldsymbol{c}_h$ | `h_e_dot_1: equation` (eq 9, §III.A.2) | error dynamics $\dot{\boldsymbol{h}}_\text{e}$ | $\tilde{\boldsymbol{c}}_h - \dot{\boldsymbol{h}}_\text{d}$ — **INCLUDES** the desired-flow derivative |

The relation $\boldsymbol{c}_h \triangleq \tilde{\boldsymbol{c}}_h - \dot{\boldsymbol{h}}_\text{d}$ is stated in §III.A.2's lead paragraph after the perfect-attitude-tracking substitution.

## Why two forms

The natural plant
$$\dot{\boldsymbol{h}} = B_h\,^\mathcal{V}\boldsymbol{F}_u + \tilde{\boldsymbol{c}}_h + \boldsymbol{d}_h$$
is a kinematic fact about the camera; $\dot{\boldsymbol{h}}_\text{d}$ has no place in it (the controller-design $\boldsymbol{h}_\text{d}$ doesn't exist yet).

The error dynamics
$$\dot{\boldsymbol{h}}_\text{e} = \dot{\boldsymbol{h}} - \dot{\boldsymbol{h}}_\text{d} = \beta\boldsymbol{u}_h + \boldsymbol{c}_h + \boldsymbol{d}_h$$
naturally absorbs $-\dot{\boldsymbol{h}}_\text{d}$ into the kinematic-coupling cluster, giving the cleaner $\boldsymbol{c}_h$ that's used in all SMC derivations (regressor, sliding surface, Lyapunov).

## Origin / why this rule exists

**Before 2026-05-06:** the paper used a single symbol $\boldsymbol{c}_h$ in BOTH eq (7) and eq (9). The supplement's explicit form for $\boldsymbol{c}_h$ included $-\dot{\boldsymbol{h}}_\text{d}$ (the eq-9 form), but was claimed to be "the form for §II-B2" (eq 7). This was a **silent overload** — same symbol, two semantically different definitions, and one of them mathematically incorrect.

**User flag (2026-05-06):** *"$\boldsymbol{c}_h$ is mathematically the eq-(9) form, and its appearance in eq (7) is incorrect."*

**Resolution:** introduce $\tilde{\boldsymbol{c}}_h$ for eq (7), keep $\boldsymbol{c}_h$ for eq (9), make the algebraic relationship explicit. Forensic root cause: bug present since draft v1; never blew up downstream because controller analysis only uses eq (9), never eq (7).

## How to apply

1. **§II.B.2 (preliminaries / natural plant)**: use $\tilde{\boldsymbol{c}}_h$. Describe as "*kinematic coupling terms (cross products of $\boldsymbol{w}, \dot{\boldsymbol{w}}, \boldsymbol{s}, \boldsymbol{h}$) that are known and available for feedback linearization*."
2. **§III.A.2 (control design / error dynamics)**: use $\boldsymbol{c}_h$. Define inline as $\boldsymbol{c}_h \triangleq \tilde{\boldsymbol{c}}_h - \dot{\boldsymbol{h}}_\text{d}$, "*absorbing the desired-flow derivative into the kinematic coupling of \eqref{h dot: equation}*."
3. **§S1-B (supplement)**: state both. Currently:
   - $\tilde{\boldsymbol{c}}_h = -\dot{\boldsymbol{w}}\times\boldsymbol{s} - \boldsymbol{w}\times(\boldsymbol{w}\times\boldsymbol{s}) - 2(\boldsymbol{w}\times\boldsymbol{h}) - [(\boldsymbol{h}-\boldsymbol{w}\times\boldsymbol{s})\cdot\hat{\boldsymbol{e}}_3]\boldsymbol{h}$
   - $\boldsymbol{c}_h = \tilde{\boldsymbol{c}}_h - \dot{\boldsymbol{h}}_\text{d}$
4. **All SMC / Lyapunov derivations** (regressor $\boldsymbol{\theta}$, sliding-surface dynamics, control law, $V$ candidate, $\dot{V}$ bound): use $\boldsymbol{c}_h$. Never $\tilde{\boldsymbol{c}}_h$.

## Related conventions

- `feedback_kinematics_phrasing.md` — naming the equations themselves (image position/orientation kinematics, optic-flow dynamics).
- `feedback_target_image_parameters.md` — three image parameters (s, α, h) framework.
- `feedback_audit_anchoring_after_relocation.md` — when restructuring (e.g., when paragraph 4 was moved from §II.B.2 to §III.A.2), re-audit symbol overloads like this one.
