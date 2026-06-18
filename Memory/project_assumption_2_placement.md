---
name: Assumption 2 placement — §III intro (section-level)
description: Locked 2026-05-07 (latest revision). Assumption 2 (perfect attitude tracking) is stated at the END of the §III intro paragraph, BEFORE §III.A. NOT in §II.C (rejected 2026-05-06 — forward-reference burden + framing mismatch); NOT in §III.A.2 (rejected 2026-05-07 — asymmetric back-reference, since §III.B.1 also uses Assumption 2). Section-level placement gives §III.A.2 and §III.B.1 symmetric back-references. The text deliberately drops specific references to $R_\text{d}$ and §III-B2 to keep the assumption phrased abstractly.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Final placement (locked 2026-05-07, latest revision)

`control_formulation.tex` §III intro paragraph (the section-opening paragraph just before §III.A) reads:

> MDF-ASMC couples an outer-loop image-parameter controller with an inner-loop attitude controller (Fig.~2). The *optic-flow funnel* (Section~III-A2) generates the commanded acceleration through a leakage-type ASMC law, and the *target image funnel* (Section~III-A3) sizes a state-dependent attitude cone that kinematically clips it. Together they certify funnel invariance plus kinematic visibility, defining the *dual-funnel architecture* studied in this paper. The cascaded analysis throughout this section rests on the following standard time-scale-separation hypothesis.
>
> *Assumption 2* (perfect attitude tracking): The body rotation tracks the desired rotation supplied by the inner-loop attitude controller, with the exponentially-decaying tracking residual absorbed into $\boldsymbol{d}_h$ of \eqref{h dot: equation}.

§III.A.2 lead paragraph then collapses to:

> Define the optic-flow error $\boldsymbol{h}_\text{e}=\boldsymbol{h}-\boldsymbol{h}_\text{d}$. Differentiating $\boldsymbol{h}_\text{e}$ and substituting \eqref{h dot: equation} under Assumption~2 yields …

## Where Assumption 2 is invoked

- **§III.A.2 L147** — eq h_e_dot_1 derivation: "under Assumption~2".
- **§III.B.1 L220** — yaw dynamics: "Assumption~2 forces $\dot{\psi}_\text{b}=\dot{\psi}_\text{d}$".
- **§III.C Theorem 1** — "under Assumptions~1--2 and Property~1".
- **§III.C Theorem 2** — "under Assumptions~1--2".

§II.C Problem statement reads "Under Assumption~1, the control problem is to design …" — Assumption 2 is NOT cited there, because at §II.C the controller architecture has not been introduced yet, so a controller-side hypothesis is premature.

## Why NOT §II.C

Initial draft (2026-05-06 morning) placed Assumption 2 in §II.C right after Assumption 1, mirroring Assumption 1's location. User flagged three reader-perspective issues:

1. **Forward-reference burden.** §II.C is before any §III content. Assumption 2 mentions $R_\text{d}$ (defined §III.B.2) and "the geometric SO(3) tracker of Section III-B2" — both unfamiliar at §II.C. Reader has no anchor.

2. **Conceptual mismatch.** §II.C is the *Problem Statement*: plant + perturbations (items 1–4) + plant-side bounds (Assumption 1). Assumption 2 is a *controller-design* hypothesis (cascaded time-scale separation). It is a different kind of statement, and mixing them in the Problem Statement breaks the framing.

3. **Circular reading.** "Under Assumptions 1 and 2, the control problem is to design a control law…" — i.e., design the controller assuming the controller's inner loop already works. Mildly circular for a reader who has not seen §III yet.

## Why the rephrasing (drop $R_\text{d}$ and §III-B2)

User chose Option X (relocate to §III.A.2) plus the abstract phrasing from Option W. Even at §III.A.2, naming $R_\text{d}$ explicitly would be a forward reference (§III.B.2 still ahead). The abstract phrasing — "the body rotation tracks the desired rotation supplied by the inner-loop attitude controller" — avoids the symbol and the section pointer, so the assumption reads cleanly without resolving forward refs. The specific construction of $R_\text{d}$ is then introduced at §III.B.2 where it belongs.

## Why Assumption 2 still appears in Theorem 1 / Theorem 2 hypotheses

Theorems are in §III.C, after §III.A.2 (where Assumption 2 is stated). The forward-reference direction is reversed: by the time the reader reaches Theorem 1, Assumption 2 has been introduced. Citing "Under Assumptions 1–2" in Theorem hypotheses is then a back-reference to §III.A.2, which is the standard convention for assumptions stated in the same controller-design section.

## How to apply

1. **Never restore Assumption 2 to §II.C.** The §II.C placement was rejected for reader-perspective reasons.
2. **Never re-introduce $R_\text{d}$ or §III-B2 reference into the Assumption 2 statement.** The abstract phrasing is deliberate.
3. **Theorem hypotheses cite Assumptions 1–2** (not just Assumption 1). Same for Property 1 in Theorem 1.
4. **§II.C Problem statement cites only Assumption 1** — Assumption 2 is a §III hypothesis.

## Related conventions

- `project_2026-05-06_section_ii_iii_lock.md` — layout snapshot; reflects this relocation.
- `feedback_assumption_redundancy_check.md` — when drafting "Under Assumption N, …" statements, do not repeat content already inside Assumption N. Same family of rule.
- `feedback_audit_anchoring_after_relocation.md` — when prose is moved, antecedents can break. The Assumption 2 relocation triggered this rule (forward refs to $R_\text{d}$ and §III-B2 broke after move; resolved by abstract rephrasing).
