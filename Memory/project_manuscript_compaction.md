---
name: Manuscript compaction and restructuring
description: Sections II/III/IV compacted, Section III restructured into Outer-Loop + Inner-Loop subsections, Deep-Sweep → supplement S4, citations locked at 20
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Compacted main manuscript toward IEEE TAES 10-page target. Key structural changes:

**Section III restructure (2026-04-16):**
- Split into `\subsection{Proposed Outer-Loop Controller}` with three subsubsections (dual-funnel PPC, leakage adaptive SMC, cone-clamp remark) and `\subsection{Proposed Inner-Loop Controller}` with two subsubsections (geometric SO(3), thrust extraction)
- Remark 2 (cone clamp) moved from after SO(3) to end of outer-loop subsection
- Label `\label{adaptive controller for inner loop : section}` at ~line 196 is misleading (points to outer-loop content) — flagged for rename

**Notation fix (2026-04-16):**
- T_d → ᴮT_u, τ_d → ᴮτ_u across control_formulation.tex, block_diagram.tex, supplemental.tex
- Convention: `_u` for plant control inputs, `_d` for disturbances and desired signals

**Earlier compaction:**
- Deep-Sweep Gain Analysis moved from results.tex to supplemental.tex Section S4
- Sections II and III tightened: merged Assumption 1+2, dropped intermediate equations
- Section IV: dropped enumerated robustness list, merged Fairness paragraph into comparison intro
- Citation count locked at 20 unique keys

**Page-reduction round 2 (2026-04-16):**
- Table III (gains) → supplement Table S2; speed envelope subsection + bar chart figure → supplement; Observation 2 condensed; Summary subsection → supplement
- Comparison_traj3d_circular.pdf and speed envelope subsection moved BACK to main paper after manuscript hit ~9 pages
- Final result: 10 pages

**Trajectory case-number convention (2026-04-16):**
- All trajectory references replaced with Case numbers: Case 1=Static, Case 2=Linear, Case 3=Sinusoidal, Case 4=Lissajous, Case 5=Circular
- Comparison table (Table VI) uses "Case N: Name" format (has space)
- Multi-init table (Table V) uses "Case N" only
- Figure captions: "Case N (static target)" for Case 1; "Case N (X target trajectory)" for Cases 2–5
- Python plot scripts updated with matching titles; all figures regenerated
- "Lissajous" stays capitalized (proper noun); other names lowercase in parenthetical context

**Block diagram rewrite (2026-04-17):**
- Added LP filter block (τ_a=0.08s) between cone clamp and geometric SO(3) — was missing from diagram
- Rerouted long-distance arrows to avoid crossings: α feed and state feedback along far-left edge, SO(3)→mix via so3.south, B_T_u bypass from lpf.west
- Tightened vertical gaps to eliminate blank space
- File: `Soft_Precise_Landing/block_diagram.tex` — full rewrite, not yet compiled/verified

**FoV observation added to results.tex (2026-04-17):**
- Body text (after Fig. 9 ref): IC5 physical camera pixels briefly exceed sensor FoV on aggressive initial tilt, but virtual-camera features (on which visibility funnel p_1(t) is defined) stay within bounds
- Assumption: camera with FoV ≥90° accommodates worst-case tilt; simulation assumes continuous feature availability (standard IBVS practice)
- MATLAB code does NOT model FoV loss — no pixel clipping/bounds check; decided not to add for this paper

**Image-plane plot redesign (2026-04-17):**
- Image-plane plot redesigned with single-axis overlaid layout in `scripts/make_multi_init_plots.py` (the prior 2×3-grid version is `Obsolete/scripts/make_multi_init_plots_v1.py`; promoted 2026-04-19)
- All 4 corners × 5 ICs on one axis; start quads (solid, faded), end quads (long-dash), desired quad (black solid)
- Corner trajectories colored by IC, linestyle by corner; legend lower-right, ncol=2

**Page reduction plan (drafted 2026-04-17, NOT YET EXECUTED):**
- Move Table III (gains) → supplement Table S2 (~1 pg)
- Move speed-envelope subsection + figure* → supplement Section S3-F (~1 pg)
- Plan file: `C:\Users\suhm_\.claude\plans\fizzy-stargazing-honey.md`

**Section III five-cut compression (2026-04-21):**
Applied 5 cuts to §III (MDF-ASMC CONTROL DESIGN) following "shift before delete":
- (A) Remark 1 (funnel-saturation guard) compressed to one sentence + pointer; full remark now in §S2-A.
- (B) Proof sketch of Theorem 1 deleted from main (full proof was already in §S2-D).
- (C) Intermediate θ/d̄ regressor algebra replaced with "definitions in Section~S2".
- (D) SO(3) inner-loop tracker details compacted — kept torque law + pointer to §S2 for R_d construction from $\,^\mathcal{I}\boldsymbol{a}_\text{d}$ and $\psi_\text{d}$.
- (E) Corollary 1 compacted to 3 sentences.
All cuts shifted content to supplement first (compliant with `feedback_shift_before_delete.md`).

**New §II figure (2026-04-21): `frames_planes.tex`**
Single-column TikZ figure inserted at §II preamble with `\label{frames planes: figure}`. Shows inertial (black) / body+camera shared-origin (blue, tilted 20°) / virtual (orange, spatially offset for readability with dashed connector to true shared origin) frames, UAV/target position vectors, physical and virtual image planes with FoV pixel box, and two projection rays from a target corner. Hit-point parameters 0.174 (HitC) and 0.227 (HitV) computed from ray/plane intersection geometry — NOT eyeballed — so the projection points lie exactly on the drawn planes. Required 3 screenshot-iteration rounds to clear label overlaps, axis collisions, and out-of-plane hit points.

**Comparison-summary figure relocated back to main (2026-04-21):**
`Figures/generated/comparison_summary.pdf` moved from supplement §S3-F to `results.tex` as a `figure*`, near the comparison narrative. Supplement subsubsection deleted; label `\label{comparison summary: figure}` is now in main. The three "moved back to main" figures after the initial 12→10 page cut are now: speed envelope, comparison_traj3d_circular, and comparison_summary.

**Polishing pass (2026-04-22):**
- *Funnel-saturation guard standardization:* Four locations in `manuscript.tex` (abstract, contributions (c), contributions (d)) plus `control_formulation.tex:196` unified on canonical form "funnel-saturation guard" — earlier drift between "funnel guard" / "saturation guard" resolved.
- *Compass claim corrected:* Abstract previously read "outer loop operates without altitude or compass measurements" — false, since the SO(3) inner loop consumes ψ from the compass. User manually edited to "proposed controller operates without altitude or compass measurements" (keeps reader-facing claim; compass is still used inside inner loop for attitude feedback).
- *`^I ã_d` explicitly defined in supplemental §S2:* New equation `sup:I a cd filt` establishes filtered desired inertial acceleration as first-order LP response of `^I a_d` with τ_a=0.08s, α_a=τ_a/(τ_a+Δt), initial condition `^I ã_d(t_0)=^I a_d(t_0)`. LP filter exists because L_s^{-1} amplifies pixel noise as `^V z_t → 0`.
- *Gap (iii) reworded:* "…from a single theorem" → "…from a single layered certificate"; trailing clause reworded to "as required to jointly certify soft and precise touchdown."
- *Three-tier image-plane error naming adopted (see `feedback_image_feature_naming.md`):* Seven locations edited — abstract + block-diagram caption + subsection title + defining paragraph + Corollary 1 + supplemental table header + block_diagram.tex node label. Short tag "Normalized Virtual Feature PID" used inside body/table/diagram; abstract keeps longer "normalized virtual image feature PID" for cold-start readability.
- *Supplemental §S3 internals intro refreshed:* "outer image-feature error" → "virtual image feature error" (line 341).
- *Funnel-symbol uniformization (late 2026-04-22):* Visibility funnel renamed from `\boldsymbol{\rho}_\text{fov}(t)` / `\ell_\text{fov}` / `\boldsymbol{\rho}_{\text{fov},0}` / `\boldsymbol{\rho}_{\text{fov},\infty}` to `\boldsymbol{p}_1(t)` / `\xi_1` / `\boldsymbol{p}_{1_0}` / `\boldsymbol{p}_{1_\infty}` (mirrors `\boldsymbol{p}_2(t)` on the optic-flow funnel). Scalar-k form: `\rho_{\text{fov},k}(t) → p_{1_k}(t)`. Sensor half-width `\mathcal{R}/(2f)` (formerly `\boldsymbol{p}_{1_0}`) renamed to `\boldsymbol{\varphi}_\text{max}` to free `\boldsymbol{p}_{1_0}` for the visibility-funnel initial value; supplement matrix form `P_{1_0} → \Phi_\text{max}`. Locations edited: `control_formulation.tex` (lines 40, 170, 172, 182, 220, 222, 224, 226, 230, 234), `supplemental.tex` (lines 181, 183, 196, 393–395), `frames_planes.tex:120`. S2 parameter-table header kept as "Visibility Funnel --- FoV-Adaptive Cone Clamp"; row labels updated. MATLAB code variables (`rho_fov_0`, `l_fov`) left unchanged for code compat. See `feedback_funnel_symbol_convention.md`.
- *15-px strict-inset justification added (2026-04-22):* `control_formulation.tex` line 222 previously said only `\boldsymbol{p}_{1_0}\preceq\mathcal{R}/2` without explaining the 15-px MATLAB inset ([145;105] vs `\mathcal{R}/2=[160;120]`). Strengthened to `\prec` strict inequality and appended the physical justification: the cone clamp operates on virtual-camera features while the visibility constraint is stated on physical corners, so the margin absorbs perspective spread from initial body tilt (IC5 transient v-axis breach, see `project_fov_observation.md` for the empirical retune history [160,120]→[120,90]→[150,110]→[145,105]). Validation: grep'd `Constants.m:14` (`res=[320;240]`) and three harnesses (`InitGains_Comparison.m:58-59`, `run_simulation.m:81-82`, `visualControl_IBVS_adaptive.m:82-83`) — all three carry identical `K_PLASMC.rho_fov_0=[145;105]` with "inset 15 px per side" comment.

**Why:** User explicitly asked to reduce toward 10 pages and cap refs at 30.

**How to apply:** `_citecount.py` at repo root counts unique `\cite{}` keys. Before claiming naming consistency, grep both `normalized.?[Ee]rror` and `image.?feature.?error` across the Soft_Precise_Landing tex set — occurrences should exist only inside the definition paragraph and the intended full-form abstract phrase.
