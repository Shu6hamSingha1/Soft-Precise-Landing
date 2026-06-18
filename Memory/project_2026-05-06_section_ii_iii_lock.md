---
name: §II / §III layout snapshot — 2026-05-06 lock
description: Snapshot of §II (Preliminaries & Problem Formulation) and §III (MDF-ASMC Control Design) layout after the 2026-05-05 → 2026-05-06 restructure session. Captures the current subsection structure, where each symbol/equation lives, the natural-plant / error-dynamics split, and the locations of Property 1, the Remark, the block diagram, and Problem 1. Use as the canonical reference when editing §II / §III; if the layout has drifted by the next session, this snapshot identifies the baseline.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## §II PRELIMINARIES \& PROBLEM FORMULATION

```
§II intro (one paragraph, lists the three §II subsections)
[Fig. 1: frames_planes.pdf]
§II.A Quadrotor Dynamics
   eq (1) `quadrotor dynamics: equation` — 13-state Newton–Euler
   defines: m, J, B_v, B_ω, c_v, c_ω, F_u, τ_u, F_d, τ_d
   defines: combined input u = [^B T_u, ^B τ_u^⊤]^⊤ ∈ R^4
   sign: ^B F_u = [0, 0, -^B T_u]^⊤ with ^B T_u ≥ 0 (positive thrust magnitude)
§II.B Image Parameters (TWO subsubsections — merged from earlier three)
   intro paragraph (camera setup, virtual frame V, supplement §S1-A pointer)
   §II.B.1 Virtual Image Pose (Position + Orientation MERGED 2026-05-05)
      eq (2) `virtual image position: equation` — s = [^V r̂; 1]
      eq (3) `s dot: equation` — natural ŝ̇ kinematics (NOT error form)
      eq (4) `virtual image orientation: equation` — α via 2nd-order moments
      eq (5) `alpha dot: equation` — natural α̇ = -ψ̇_b + d_α (NOT error form)
   §II.B.2 Optic Flow
      eq (6) `optic flow def: equation` — h, w definitions
      eq (7) `optic flow inverse: equation` — pseudo-inverse recovery
      eq (8) `h dot: equation` — natural plant ḣ = B_h ^V F_u + c̃_h + d_h
              uses TILDE c̃_h (not c_h — see feedback_c_tilde_h_convention.md)
§II.C Problem Statement
   bullet items 1–4 (4 perturbation classes)
   Assumption 1 (uncertainty bounds, existence-only, need not be known)
   Problem statement (declarative, "Under Assumption 1, the control
                      problem is to design a control law for the
                      combined input u …")
   [Assumption 2 NOT here — moved to §III.A.2 lead 2026-05-06 for
    reader-perspective reasons; see project_assumption_2_placement.md]
[NO Property 1 — moved to §III.A.3 in 2026-05-06]
[NO block diagram float — moved to §III intro area in 2026-05-05]
```

## §III MDF-ASMC CONTROL DESIGN

```
§III intro (REDUCED 2026-05-06: 2 sentences; dual-funnel architecture only)
[Fig. 2: block_diagram_v3.pdf]
§III.A Outer Loop: Dual Funnel and Adaptive SMC
   intro paragraph (3 sub-blocks enumerated 2026-05-06)
   §III.A.1 Virtual Image Point Control Design
      defines s_d, s_e (controller-side error)
      eq `normalized error: equation` — bar-r_e
      eq `s_e_dot_d: equation` — PID law
      eq `h_d final: equation` — desired optic flow
      defines h_rd (positive descent flow scalar)
   §III.A.2 Optic Flow Control Design
      lead paragraph: defines h_e, STATES Assumption 2 (perfect
                       attitude tracking; relocated from §II.C
                       2026-05-06), derives error dynamics from eq (8)
                       under Assumption 2
                       defines c_h ≜ c̃_h - ḣ_d (NO TILDE — see convention memory)
      eq `h_e_dot_1: equation` — ḣ_e = β u_h + c_h + d_h
      defines: u_h = -^I R_V^⊤ ^I a_d, β = 1/^V z_t
      PPC paradigm + funnel + tanh transform
      eq `optical flow error transformation: equation`
      eq `unconstrained system: equation`
      eq `regressor: equation` — uses c_h (NOT c̃_h)
      eq `adaptive control law: equation`, `adaptive law: equation`
   §III.A.3 Acceleration Conditioning (Property 1 lives here, MOVED 2026-05-06)
      goal-first opening (target visibility)
      eq `rho fov: equation` — funnel envelope p_1(t)
      eq `d min fov: equation` — funnel margin
      eq `cone angle: equation` — θ_cone(t)
      eq `cone clamp: equation` — projection
      Property 1 (cone-clamped lateral acceleration) — STATED HERE
      eq `lpf: equation` — low-pass filter
      ^B T_u = m ‖^I ã_d‖ — feed to plant
§III.B Inner Loop: Virtual-Compass Yaw ASMC and SO(3) Tracker
   intro paragraph (yaw + SO(3) overview, ADDED 2026-05-06)
   §III.B.1 Yaw Control Design
      defines α_d = 0, α_e (controller-side)
      eq `alpha_e_dot: equation` — α̇_e = -ψ̇_d + d_α (under perfect yaw tracking)
      eq `yaw control law: equation` — leakage-type ASMC + adaptive gain
      eq `psi d integrator: equation` — virtual-compass integrator
   §III.B.2 Geometric SO(3) Attitude Tracker
      eq `R_d construction: equation` — Gram-Schmidt
      eq `so3 errors: equation`, `so3 torque: equation`
      cites Lee 2010 [Thm. 3.1]
   Remark — linearity admits Lyapunov + SO(3) decoupling (RELOCATED 2026-05-06
            from end of §III.A.3 to end of §III.B.2)
§III.C Lyapunov Stability Analysis (THIRD subsection in §III, renders III-C)
   intro: GUUB layered argument, cite [khalil2002], full algebra in §S2-B/C
   Theorem 1 — Adaptive Optic-Flow Funnel Invariance
      cites Assumptions 1-2 + Property 1 (Assumption 2 added 2026-05-06)
      eq `UUB bound: equation`
      Proof of Theorem 1 (sketch with §S2-B pointer for full algebra)
   Corollary 1 — Closed-loop target visibility
   Theorem 2 — Adaptive Yaw ASMC Ultimate Boundedness
      eq `yaw UUB bound: equation`
      eq `yaw sigma dynamics: equation`
      eq `V alpha candidate: equation`
      eq `V alpha dot bound: equation`
      Proof of Theorem 2 (sketch with §S2-C pointer for full algebra)
   Closing summary (FIXED 2026-05-06: no longer over-claims soft-precise
                    landing; says "optic-flow funnel invariance, yaw UUB,
                    target visibility; soft-precise landing validated
                    empirically in §IV")
```

## Key conventions locked in this session (2026-05-05 → 2026-05-06)

1. **§II.B has 2 subsubsections** (Pose + Optic Flow). Position and Orientation merged into Pose; both use `\emph{}` introductions of their definitional terms.

2. **§II.B uses NATURAL kinematics** ($\dot{\boldsymbol{s}}, \dot{\alpha}, \dot{\boldsymbol{h}}$), not error forms. The error definitions ($\boldsymbol{s}_\text{e}, \alpha_\text{e}, \boldsymbol{h}_\text{e}$) are absorbed where consumed: §III.A.1 (for $\boldsymbol{s}_\text{e}$), §III.A.2 (for $\boldsymbol{h}_\text{e}$), §III.B.1 (for $\alpha_\text{e}$).

3. **$\tilde{\boldsymbol{c}}_h$ vs $\boldsymbol{c}_h$ split**: see `feedback_c_tilde_h_convention.md`. Natural plant uses tilde; error dynamics uses bare. Related by $\boldsymbol{c}_h \triangleq \tilde{\boldsymbol{c}}_h - \dot{\boldsymbol{h}}_\text{d}$.

4. **Property 1** lives in §III.A.3 (where the cone clamp is constructed), NOT §II.C. Theorem 1 cites it by name.

5. **Combined input $\boldsymbol{u} = [\,^\mathcal{B}T_u, \,^\mathcal{B}\boldsymbol{\tau}_u^\top]^\top \in \mathbb{R}^4$** defined in §II.A. Sign convention: $\,^\mathcal{B}T_u \ge 0$ is positive thrust magnitude.

6. **§II.C Problem statement** is declarative ("the control problem is to design"), uses combined $\boldsymbol{u}$, no "Problem 1" prefix (only one problem).

7. **Block diagram float** placed in §III intro area (after §III.A.1's first paragraph or thereabouts; LaTeX float placement).

8. **Remark on linearity / SO(3) decoupling** placed at end of §III.B.2 (just before §III.C), summarising both outer- and inner-loop linearity that admits Lyapunov analysis.

9. **Closing summary at end of §III.C** does NOT claim "soft-precise landing certificate"; instead claims "optic-flow funnel invariance + yaw UUB + target visibility" with empirical validation in §IV.

10. **Supplement compiles separately** — main-paper labels referenced as text ("Section II-B2 of the main paper", "Fig. 1 of the main paper"), never `\eqref{}` / `\ref{}`. The supplement preamble now loads `mathtools` (added 2026-05-05) for `\splitfrac` in the wide $\boldsymbol{l}_\alpha$ closed-form.

11. **frames_planes.pdf** is now PDF (not TikZ); shows body frame $\mathcal{B}$ explicitly with dashed mount link from $O_b$ to $O_c \equiv O_v$. Caption mentions all four frames ($\mathcal{I}, \mathcal{B}, \mathcal{C}, \mathcal{V}$). Legacy TikZ in `Obsolete/Soft_Precise_Landing/`.

## Memories created or updated this session (2026-05-05 → 2026-05-06)

| Memory | Status |
|---|---|
| `feedback_c_tilde_h_convention.md` | NEW (2026-05-06) — see this file for the symbol split |
| `feedback_assumption_redundancy_check.md` | NEW (2026-05-05) |
| `feedback_audit_anchoring_after_relocation.md` | NEW (2026-05-05); updated 2026-05-05 with failure mode B |
| `feedback_goal_first_when_mechanism_follows.md` | NEW (earlier this session) |
| `feedback_target_image_parameters.md` | UPDATED — section-structure epoch-2 (3→2 subsubsections in §II.B) |
| `feedback_yaw_simplification_placement.md` | UPDATED — clarified structural exception for $\,^\mathcal{V}\boldsymbol{\omega}_{\mathcal{V}/\mathcal{I}}$ |
| `reference_frames_planes_figure.md` | NEW + multiple regen updates (2026-05-05 + 2026-05-06) |
| `project_2026-05-06_section_ii_iii_lock.md` | NEW (this file) — layout snapshot |

## Verifier state at session close

- Labels: 122
- Refs: 75 (all resolved)
- Bib keys: 55, cites: 21 (all resolved)
- Brace balance: balanced on all four files
- `manuscript.tex` 181/181, `control_formulation.tex` 1341/1341, `results.tex` 334/334, `supplemental.tex` 1687/1687

## Open items / known minor issues at session close

- §II.B.1 forward-references $\boldsymbol{h}, \boldsymbol{w}$ to §II.B.2 (eq 6) — accepted; user explicitly chose to keep current order.
- §III.A.3 has "<1% of control steps in the simulations" (L200) — flagged as simulation detail in §III; user chose NOT to move to §IV.
- §III.A.2 and §III.B.1 each have a "without *a priori* knowledge" clause that partially overlaps with Assumption 1 — user accepted these as classical-SMC contrast (not strict redundancy).
