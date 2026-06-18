---
name: Optic-flow dynamics attribution at eq (8) — singhal2025 not herisse2012
description: At eq `h dot: equation` (§II.B.2 of control_formulation.tex), the prefatory clause was originally "Following~\cite{herisse2012}". This was a misattribution flagged 2026-05-06; Hérissé 2012 derives only optic-flow kinematics on a spherical retina, not the force-input dynamics. The correct precedent is Singhal 2025, which derives the 1D vertical analogue of the same form. Eq (8) of this paper is the 3D vector extension. Locked 2026-05-06.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## What each cited work actually derives

**Hérissé 2012** — *Landing a VTOL UAV on a Moving Platform Using Optical Flow*, IEEE T-RO 28(1), 2012.
- Spherical-projection optic-flow kinematics: $\dot p = -\Omega\times p - \frac{\cos\theta}{d(t)}\pi_p(V - V_P)$ (their eq 7–8).
- Average optic flow over a solid angle: $w = (v - v_t)/d$ (their eq 12) — defines flow as scaled velocity.
- Vertical flow as inverse time-to-contact: $w_z = -\dot h / h$ (their eq 29).
- **Does NOT derive a force-input dynamics** $\dot{\boldsymbol{h}} = B_h\,^\mathcal{V}\boldsymbol{F}_u + \cdots$. Plant is differentiated only once (image point → flow), no Newton-law substitution to inject $\,^\mathcal{V}\boldsymbol{F}_u$.

**Singhal 2025** — *Performance-Constrained Adaptive Sliding Mode Control for Guaranteed Soft Landing Using Optic Flow*, IEEE T-AES 61(5), 2025.
- 1D vertical optic flow: $y_r(t) = v_z(t)/z(t)$ (their eq 3).
- 1D vertical optic-flow error dynamics: $\dot y_e(t) = -y_r^2(t) + \beta(t)\mu(t) + \varepsilon(t)$ (their eq 7).
- Has the $\beta(t)\mu(t)$ structure with time-varying $\beta$, lumped disturbance $\varepsilon$, and Newton-law substitution.
- **1D, vertical landing only**; stationary surface.

**This paper, eq (8) `h dot: equation`** — $\dot{\boldsymbol{h}} = B_h\,^\mathcal{V}\boldsymbol{F}_u + \tilde{\boldsymbol{c}}_h + \boldsymbol{d}_h$ with $B_h = -(m\,^\mathcal{V}z_\text{t})^{-1}I_{3\times 3}$.
- 3D vector translational optic flow.
- Mobile target, lateral + vertical + rotational coupling.
- Kinematic coupling $\tilde{\boldsymbol{c}}_h$ involves cross products of $\boldsymbol{w}, \dot{\boldsymbol{w}}, \boldsymbol{s}, \boldsymbol{h}$ — not present in either prior cite.
- **Genuine 3D extension of Singhal 2025**; full algebra in supplement §S1-B.

## What was wrong, and what was fixed

**Before 2026-05-06:** L93 read "Following~\cite{herisse2012} and expressing $\,^\mathcal{V}\boldsymbol{F}_u=\,^\mathcal{V}R_\mathcal{B}\,^\mathcal{B}\boldsymbol{F}_u$, the optic-flow dynamics reads…"

**Why wrong:** Hérissé 2012 does not derive a force-input dynamics. Attaching "Following~\cite{herisse2012}" to the eq (8) lead-in attributed a derivation to a paper that does not contain it.

**After 2026-05-06:** L93 reads "Extending the 1D vertical-flow dynamics of~\cite{singhal2025} and expressing $\,^\mathcal{V}\boldsymbol{F}_u=\,^\mathcal{V}R_\mathcal{B}\,^\mathcal{B}\boldsymbol{F}_u$, the optic-flow dynamics reads…"

## Where each cite still belongs

- **Hérissé 2012** — currently NOT cited in §II.B.2 directly. If a future edit wants to attribute the *definition* $\boldsymbol{h} = \,^\mathcal{V}\boldsymbol{v}_{\text{t/b}}/\,^\mathcal{V}z_\text{t}$ at eq (6), `\cite{herisse2012}` is the right reference for that definition (their eq 12 has the analogous $w = (v-v_t)/d$). Hérissé is also cited elsewhere in the paper for the optic-flow soft-landing principle — see `reference_optic_flow_soft_landing_principle.md`.
- **Singhal 2025** — now cited at eq (8) lead-in for the 1D precedent of the dynamics form, and at §II.C item 4 for actuator faults / saturation.

## How to apply

1. **Never restore "Following~\cite{herisse2012}" at the eq (8) lead-in.** It is mathematically incorrect — Hérissé does not derive a force-input dynamics.
2. **If editing eq (6)** (definition of $\boldsymbol{h}$): `\cite{herisse2012}` is the right reference for the scaled-velocity interpretation.
3. **If asked "where does eq (8) come from"**: answer is "extends Singhal 2025 (1D vertical) to 3D vector form; full algebra in supplement §S1-B".
4. **If a reviewer asks for the precedent**: Singhal 2025 has the closed scalar form; the 3D extension is original.

## Related conventions

- `feedback_citation_classification_audit.md` — broad rule: every cite must be PDF-verified for what it actually derives.
- `feedback_validate_critiques_against_cited_works.md` — process discipline: claims about what cited works do/don't do must be PDF-verified.
- `feedback_verify_before_claim.md` — same process discipline at finer grain (every cite-related claim).
- `reference_optic_flow_soft_landing_principle.md` — Hérissé 2012's role in the soft-landing principle (constant vertical flow → exponential descent); separate from the dynamics-derivation question above.
