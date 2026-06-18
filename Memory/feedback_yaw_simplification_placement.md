---
name: Yaw simplification — controller-side, NOT structural
description: The identity $\,^\mathcal{B}\boldsymbol{\omega}_\text{b}=[0,0,\dot{\psi}]^\top$ is a *controller-architectural* simplification (only yaw is commanded) and must NOT be applied inside the general image-kinematic formulation ($\boldsymbol{c}_h$, $\boldsymbol{d}_h$, $\boldsymbol{\omega}_{\text{t/b}}$); introduce it only in the Yaw Control section. EXCEPTION: $\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}}$ (virtual-frame angular velocity) is structurally yaw-only by construction of $\mathcal{V}$ — that is a fact, not a simplification, and may appear in eq (5).
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** The yaw-only simplification $\,^\mathcal{B}\boldsymbol{\omega}_\text{b}=[0,0,\dot{\psi}]^\top$ is a *controller architectural* choice — it presupposes the inner loop commands only yaw via the SO(3) tracker. Inside general image-kinematic derivations (where the kinematics should hold for any rigid-body motion), keep $\boldsymbol{\omega}_\text{b}$ as a full 3-vector. Introduce the yaw-only specialisation **once**, in the Yaw Control section (currently §III.B.1), where it earns its keep by collapsing the orientation kinematics to a scalar law.

**Why.**
1. **Separation of concerns.** Image-kinematic derivations are facts about the camera; they should be valid for any rigid-body motion, not pre-suppose the controller architecture.
2. **Reviewer defence / generalisation.** A reviewer asking "does this work without yaw-only commands?" should not be forced to re-derive the kinematics.
3. **Where the simplification pays off.** The yaw-only specialisation buys a *scalar* yaw control law in §III.B.1; that is local payoff and the simplification belongs there.

## Where the rule applies (yes — keep general)

| Symbol / quantity | Where it appears | Keep general because |
|---|---|---|
| $\,^\mathcal{B}\boldsymbol{\omega}_\text{b}$ inside $\boldsymbol{d}_h$ | Centrifugal/Coriolis cross term $\,^\mathcal{V}\boldsymbol{\omega}_\text{b}\times\,^\mathcal{V}\boldsymbol{v}_\text{b}$ in supplement §S1-B | $\boldsymbol{d}_h$ is a disturbance term — must be valid for any body motion. |
| $\,^\mathcal{B}\boldsymbol{\omega}_\text{b}$ inside $\boldsymbol{c}_h$ | Cross-product coupling terms in optic-flow dynamics (supplement §S1-B) | Same — kinematic coupling, not controller-side. |
| $\boldsymbol{\omega}_{\text{t/b}}$ (if used) | Definition of relative target angular rate | Was the original symbol before the §II.B reorder; now superseded by $\boldsymbol{w}$ (target relative to $\mathcal{V}$, not $\mathcal{B}$). If reintroduced, keep general. |

## Where the rule does NOT apply (structural exception, OK to specialise)

**$\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}}$ — angular velocity of the virtual frame relative to inertial, expressed in $\mathcal{V}$.**

This quantity is *structurally* yaw-only by construction of $\mathcal{V}$:
- $\mathcal{V}$ is defined as $\mathcal{C}$ with roll and pitch rotated out (see §II.B intro).
- Therefore $\mathcal{V}$ has only yaw motion relative to inertial.
- Therefore $\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}} = \dot{\psi}\hat{\boldsymbol{e}}_3$ — *this is a definitional fact, not a simplification.*

This is the form that appears in `control_formulation.tex` eq (5):
$$\boldsymbol{w} \triangleq \,^\mathcal{V}\boldsymbol{\omega}_\text{t} - \dot{\psi}_\text{b}\hat{\boldsymbol{e}}_3$$
where the second term *is* $\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}}$ (rendered with subscript $_\text{b}$ because $\mathcal{V}$ shares yaw rate with $\mathcal{B}$ by construction). This usage is **not a violation** of the rule — keep it.

## Quick test for new prose

If a candidate edit substitutes $[0,0,\dot{\psi}]^\top$ for an angular velocity, ask:
1. **Is the substituted symbol the body angular velocity** ($\,^\mathcal{B}\boldsymbol{\omega}_\text{b}$ or anything derived by rotating it into another frame, like $\,^\mathcal{V}\boldsymbol{\omega}_\text{b}$)? → Rule applies. Keep general; introduce yaw-only form in §III.B.1.
2. **Is the substituted symbol the virtual frame's angular velocity** ($\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}}$)? → Structural exception. Yaw-only form is correct anywhere it appears.

**Common mistake:** confusing $\,^\mathcal{V}\boldsymbol{\omega}_\text{b}$ with $\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}}$. They are different:
- $\,^\mathcal{V}\boldsymbol{\omega}_\text{b}$ = body angular velocity expressed in $\mathcal{V}$. **Generally has 3 nonzero components** when roll/pitch rates are nonzero.
- $\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}}$ = $\mathcal{V}$'s own angular velocity. **Always yaw-only**, by construction.

## Origin

User flagged 2026-04-25 when an early draft of $\boldsymbol{c}_h$ pre-substituted $[0,0,\dot{\psi}]^\top$ for the body angular velocity inside the general kinematic derivation. Re-clarified 2026-05-05 after eq (5) review surfaced the structural exception — the original memory didn't distinguish $\,^\mathcal{V}\boldsymbol{\omega}_\text{b}$ from $\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}}$ and risked over-application.

## Related conventions

- `feedback_lateral_body_rates_no_d_alpha.md` — orthogonal observation about $d_\alpha$: lateral body rates $\boldsymbol{\omega}_{\text{b},xy}$ are absorbed by $\,^\mathcal{V}R_\mathcal{C}$ and don't reappear in $\dot{\alpha}$ / $d_\alpha$.
- `feedback_kinematics_phrasing.md` — naming for the equations themselves (image position kinematics / image orientation kinematics / optic-flow dynamics).
