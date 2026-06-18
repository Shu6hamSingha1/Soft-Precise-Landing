---
name: Project custom skills
description: All 14 custom skills defined in .claude/skills/ for the Soft-Precise-Landing project; invoke via /skill-name. Includes /excalidraw-diagram and /comparison-multi-speed-plots added 2026-04-26.
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Custom skills in `.claude/skills/`:

| Skill | File | Purpose |
|---|---|---|
| `/manuscript-review` | `manuscript-review.md` | IEEE TAES hostile-reviewer pass: 9-step methodology — .mat data freshness, IC/cross-ref consistency, novelty claims, proofs, experimental validation, comparison fairness, notation uniformity, writing/figures, compiled-PDF audit. Covers manuscript.tex + control_formulation.tex + results.tex + supplemental.tex + block_diagram.tex |
| `/verify-tex` | `verify-tex.md` | Mechanical LaTeX check: unresolved refs, missing citations, label drift |
| `/analyze-controller` | `analyze-controller.md` | Analyze result_ctrl_{1-5}.mat comparison data |
| `/multi-init-plots` | `multi-init-plots.md` | Generate/analyze multi-init sweep plots |
| `/refresh-tables` | `refresh-tables.md` | Refresh results.tex tables from current .mat data |
| `/probe-crash` | `probe-crash.md` | Diagnose a crashed simulation run |
| `/sync-gains` | `sync-gains.md` | Sync MDF-ASMC gains across harnesses (bestParam.mat → InitVar/InitGains) |
| `/paper-gains` | `paper-gains.md` | Check baseline controller gains against their source papers |
| `/sweep-winners` | `sweep-winners.md` | Extract best candidates from parameter sweep logs |
| `/relock-gains` | `relock-gains.md` | Lock in new gain values after a tuning round |
| `/audit-harness-sync` | `audit-harness-sync.md` | Audit that all three harnesses (single-run, multi-init, comparison) share the same config |
| `/prose-audit` | `prose-audit.md` | Mechanical manuscript-prose style audit (16 rules: vague "this", controller name staleness, physical-vs-math visibility phrasing, funnel role-naming, legacy-design comparisons, image-feature-point naming + epoch-2 subscript conventions (rule 6, updated 2026-05-04), plant-input subscripts, trajectory naming, virtual-image-point error symbol naming + "Control Design" subsection pattern (rule 9, epoch-3 2026-05-04), funnel/FoV symbol spelling, overclaim of unproven properties (rule 11), optic-flow principle: bounded h, not constant/zero (rule 12), §I qualitative / §IV numerical anchoring + locked Para 1-7 wording (rule 13), citation classification (rule 14, includes chen2025 dual PBVS+adaptive), **Oxford/US spelling consistency** (rule 15, locked 2026-05-03), **hardcoded section-ref composition** (rule 16, locked 2026-05-03)) |
| `/excalidraw-diagram` | `excalidraw-diagram.md` | Author + embed Excalidraw diagrams in IEEE TAES tex (storage convention, SVG/PDF export workflow, when to pick Excalidraw vs TikZ) |
| `/comparison-multi-speed-plots` | `comparison-multi-speed-plots.md` | Regenerate per-trajectory baseline-controller multi-speed PDFs (4 PDFs from `<traj>_multi_speed_comparison.mat`); main paper Fig. 7 + supplement §S3-G figures |

Custom hooks in `.claude/hooks/`:
- `plasmc_gain_sync_reminder.py` — reminds about gain sync when editing InitVar/InitGains files (filename retained for compatibility; same function regardless of controller name epoch)

**Updated 2026-04-16:** `/manuscript-review` expanded from 8 to 9 steps — added §1 data freshness (verifies every table against live .mat files), §2 IC and cross-reference consistency (IC naming, supplement hardcoded refs, block diagram signal labels), extended file scope to include supplemental.tex and block_diagram.tex. Should now also check Case-number consistency (Case 1–5 convention, never bare trajectory names as labels).

**Updated 2026-04-17:** Block diagram rewritten with LP filter block between cone clamp and SO(3); arrows rerouted to avoid crossings. FoV observation + ≥90° camera assumption added to results.tex body text. Image-plane plot redesigned with single-axis overlaid layout (now `scripts/make_multi_init_plots.py`; the original 2×3-grid version was renamed to `Obsolete/scripts/make_multi_init_plots_v1.py` on 2026-04-19). Page-reduction plan drafted (Table III + speed envelope → supplement) but NOT yet executed.

**Updated 2026-04-18:** Approach 2 committed to Multi_init harness (d72ab4d). `/probe-crash` skill rewritten to cover the new hover-fail mode (in addition to crash-fail), drop `zeta_1`/`p_1` references (removed by Approach 2), and reference new logs (`V_s_e_n`, `d_min_log`, `theta_cone_log`, etc.). New diagnostic script `scripts/diag_ic4_z_channel.py` — pure-Python, runs on cached `.mat` datasets, identifies which z-channel saturation is load-bearing. `plotter_adaptive.m` refactor for new log fields still pending (currently commented out in both Multi_init files). Comparison harness Approach-2 propagation still pending.

**Updated 2026-04-19:** `/sweep-winners` skill fixed (sweep log path corrected `MATLAB/Multi_init_cond/sweeps/` → `MATLAB/Sweeps/`) and extended with **paired-gain constraint rules**: `zp` and `zd` must be retuned as a P+D pair (never solo); `Γ(2)` conflicts with paired zp/zd bumps (Combo E failure mode). New combo-probe scripts in `MATLAB/Multi_init_cond/`: `validate_combo_precision.m`, `validate_zp_solo.m`, `validate_combo_C.m`, `validate_combo_DE.m`. `sweep_speed.m` seed-clobber bug fixed — deterministic sweeps now pass seed via 6th arg (see `reference_run_simulation_seed_api.md`).

**Updated 2026-04-19 (later) — post-Combo-D + Approach-2 figure refresh session:**
- `MATLAB/Common/regen_bestParam.m` updated for Approach 2: now calls `Constants;` before `InitGains_Comparison;`; `locked_fields` drops `gamma_1`/`p_1inf`, adds `rho_fov_0`/`rho_fov_inf`/`l_fov`/`theta_cap`. Re-ran → `bestParam.mat` now reflects Combo D (`zp=9.0`, `zd=1.4375`) + FoV-adaptive cone clamp config.
- `scripts/make_plasmc_plots.py` Plot A refactored: was 2-subplot referencing removed `p_1`/`V_s_e`/`zeta_1` (broke after Approach 2). Replaced with **3-D visibility-funnel plot** showing the exponentially decreasing `rho_fov(t)` rectangular box with the four target-corner image trajectories inside, plus quadrilateral snapshots at t=0, every 3 s, and t_end. Axes: x=t, y=v[px], z=u[px]. Backup: `Obsolete/scripts/make_plasmc_plots_v1_preApproach2.py`.
- `scripts/make_multi_init_plots.py` (single-axis overlaid layout) promoted from the v2 file; the original 2×3-grid version moved to `Obsolete/scripts/make_multi_init_plots_v1.py`. `multi_Init_Var.m` line 108 fixed to point to the canonical script path.
- All 30 multi-init figures regenerated post-Combo D + Approach 2.
- **FoV no-longer-violated claim was WRONG** — earlier audit checked `P_DS[:,0:4]` (virtual features) instead of `P_DS[:,8:12]` (physical features); IC5 physical pixels still exceed v-axis FoV (~134 px vs ±120 limit) on every trajectory. The manuscript caveat must STAY. See `feedback_pds_column_convention.md` and `project_fov_observation.md`.
- `Soft_Precise_Landing/Drafts/control_formulation_v1.tex` saved as a pre-edit backup before the Approach-2 manuscript rewrite.
- `multi_speed_cond.m` re-run with Combo D + Approach 2: 20/20 realistic runs land precise+soft; max xy=5.0 cm (Lissajous λ=1.4), max v_rel=0.154 m/s (Circular λ=1.4).

**Updated 2026-04-20 — MDF-ASMC rename + manuscript rewrite pass:**
- **Controller renamed**: PLASMC → DF-ASMC → **MDF-ASMC** (Monocular Dual-Funnel Adaptive Sliding-Mode Control). MATLAB struct identifier `K_PLASMC` retained for code-compat; all reader-facing tex, skill descriptions, and skill `.md` files now use MDF-ASMC. Hook filename `plasmc_gain_sync_reminder.py` retained — same function.
- **Paper title locked (T2)**: "Dual-Funnel Adaptive Sliding-Mode Control for Vision-Only Soft Precise Landing with Guaranteed Target Visibility".
- **Theorem 1 renamed**: "Adaptive Optic-Flow Funnel Invariance" (was the incorrect "Inner-loop GUUB on $\boldsymbol{\zeta}_2$"). Inner-loop naming was wrong because: (a) aerospace "inner loop" means SO(3) attitude; (b) Approach-2 funnels are parallel, not nested.
- **Corollary 1 kept and renamed**: "Closed-loop target visibility" (Option B rewrite — physical language, honest about ISS cascade vs by-construction cone clamp).
- **Abstract + contribution (d) + conclusion** rewritten for the parallel dual-funnel framing — the two funnels are independent (visibility sizes the cone, optic-flow generates the acceleration), the ASMC-generated acceleration is clipped in closed form by the visibility-sized cone, and the entire outer loop runs on a single monocular camera. Agreed conclusion-paragraph-2 opener: "MDF-ASMC treats visibility and soft-touchdown as two independent funnels with two architectural innovations and drives the entire outer loop from a single monocular camera."
- **New skill `/prose-audit`** (this session): mechanical 8-rule style audit over manuscript tex. Enforces the feedback-memory writing rules that accumulated across this session and prior ones.
- **New feedback memories**:
  - `feedback_no_vague_this.md` — no bare "this"/"these"/"that"/"those" without an attached noun.
  - `feedback_visibility_phrasing.md` — physical "target remains visible" in prose; corner-point only in equations/proof proper.
- **Refined memory** `project_dual_funnel_parallel_architecture.md` — signal flow is *optic-flow funnel → ASMC → cone projection*; drop the misleading "two funnels meet at the cone projection" phrasing.
- **`/manuscript-review` integration**: recommend invoking `/prose-audit` after step 7 (uniformity pass) so the mechanical style sweep catches drift the content-focused reviewer pass doesn't target.

**Updated 2026-04-22 — polishing-pass session:**
- **`/prose-audit` Rule 9 added**: `image-feature` — enforces the three-tier naming convention for `\hat{r}_e` / `\bar{r}_e` / outer-loop PID locked this session (see `feedback_image_feature_naming.md`). Flags `Normalized-Error PID`, `image-feature error PID`, `virtual image-feature error` (hyphen variant), `feature-error PID`, `normalized image feature error` (missing "virtual"), and `outer image-feature error`.
- **`/prose-audit` Rule 10 added**: `funnel-symbol` — enforces visibility-funnel spelling `\boldsymbol{p}_1(t)` / `\xi_1` / `\boldsymbol{p}_{1_0}` / `\boldsymbol{p}_{1_\infty}` / `p_{1_k}` and camera-half-FoV `\boldsymbol{\varphi}_\text{max}=\mathcal{R}/(2f)`. Flags old `\rho_\text{fov}` / `\ell_\text{fov}` / `P_{1_0}` spellings and catches the `\oslash\boldsymbol{p}_{1_0}` collision (normalization must use `\boldsymbol{\varphi}_\text{max}`). See `feedback_funnel_symbol_convention.md`.
- **Manuscript polishing** applied: funnel-saturation guard unified, `^I ã_d` explicitly defined in supplement, gap (iii) reworded to "layered certificate" / "jointly certify soft and precise touchdown", compass claim corrected (compass is used in inner-loop SO(3), not outer loop), funnel symbols uniformized (`\rho_\text{fov}` → `\boldsymbol{p}_1(t)`; sensor half-width `\mathcal{R}/(2f)` → `\boldsymbol{\varphi}_\text{max}`). See `project_manuscript_compaction.md` 2026-04-22 section for the full edit manifest.

**Updated 2026-04-22 (late) — hostile-reviewer audit close-out + infra cleanup:**
- **Five Major Concerns closed** (MC1–MC5): deep-sweep classification rewritten to match harness (33 axes / 3325 runs), supplement cross-refs fixed, §III pixel-box residual fixed, Table V data drift reconciled, PX4 deployability softened. See `project_results_tex_staleness.md`.
- **`bestParam.mat` + `regen_bestParam.m` RETIRED** → `Obsolete/Common/`. `K.p_10` now sourced from `Constants.m` in all four consumers (run_simulation.m, visualControl_IBVS_adaptive.m, visualControl_comparison.m, sweep_deep.m). See `project_bestparam_location.md`.
- **Obsolete cleanup**: 24+ stale `.mat`/`.m` files moved to `Obsolete/` (validate_combo_*, sweep_Linear_inner/outer, sweep_deep_stale_base, validate_gamma1_speed, validate_kappa0_speed). `Datasets_v1_preApproach2/` flattened into `Obsolete/Multi_init_cond/Datasets/`.
- **Aggregator off-by-one FIXED** in `scripts/_analyze_results_for_tex.py`: `x_t[:, idx]` → `x_t[:, idx-1]` (MATLAB fills col-by-col so MATLAB col `idx` = Python col `idx-1`); `t_f = idx*dt` → `(idx-1)*dt` to match MATLAB-log convention. UAV state stays at Python col `idx` (MATLAB stores `X_DS(:,idx+1)=x_c`). Table V paste-direct-from-aggregator now works. See `project_indexing_convention.md`.
- **`/refresh-tables` skill updated**: indexing-convention section rewritten (both sides now aligned); obsolete Case 4/5-swap warning removed (resolved 2026-04-20).

**Updated 2026-05-04 — image-parameter naming epoch-3 + subscript convention + label refactor + spelling pass:**

- **Parameter naming epoch 3 (2026-05-03 evening, 2026-05-04 finalised)** — §II.B subsection title "Image Parameters" (was "Target Image Parameters"); §II.B.1 "Virtual Image Position" (was "Normalised Target Position" → "Normalized Target Position"); §II.B.2 "Virtual Image Orientation" (was "Target Virtual Orientation" → "Virtual Target Orientation"). Justification: under the pinhole model, $\boldsymbol{s}=[\,^\mathcal{V}\hat{\boldsymbol{r}};1]^\top$ IS the image position (homogeneous form), so "virtual image" framing is geometrically precise and parallels the §II.B.1/§II.B.2 titles.
- **Vocabulary epoch 2 (2026-05-04)** — feature-point and centroid terminology dropped "target" qualifier and unified subscript `_k` → `_i`:
  - $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ = "image feature point" (was "target image feature point")
  - $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$ = "virtual image feature point" (was "$k$-th feature point" with `_k` subscript) / short form "virtual feature point"
  - $\,^\mathcal{C}\boldsymbol{\hat{r}}$ = "image point" (centroid)
  - $\,^\mathcal{V}\boldsymbol{\hat{r}}$ = "virtual image point" (was "target image point in the virtual frame")
  - Subscript `_i` for per-feature index everywhere (camera + virtual frames); `_k` reserved for 3-D-vector component (`h_{e_k}`); moment exponents `p, q` (was `i, j` in `\mu_{ij}`).
- **Image-plane error variable naming epoch 3 (2026-05-04)** — variables now named for the centroid error (not per-feature):
  - $\hat{\boldsymbol{r}}_\text{e}$ = "virtual image point error" (was "virtual image feature error") / short form "image point error"
  - $\bar{\boldsymbol{r}}_\text{e}$ = "normalized virtual image point error" / short "normalized image point error"
  - $\hat{\boldsymbol{r}}_\text{d}$ = "desired image point" (long form only — no short form)
  - PID short tag = "Virtual Image Point PID" (was "Normalized Virtual Feature PID")
- **Subsection-title parallel "Control Design" pattern (2026-05-04)**:
  - §III.A.1: "Virtual Image Point Control Design" (was "Normalized Virtual Image Feature Control Law")
  - §III.A.2: "Optic Flow Control Design" (was "Optic Flow Control Law")
  - §III.B.1: "Yaw Control Design" (was "Yaw ASMC with Virtual-Compass Integrator") — now also has a label `\label{yaw control: section}` so it can be cross-referenced via composition.
- **Section-label refactor (2026-05-04)** — internal LaTeX labels renamed to match new section titles + composition pattern. All `\ref{}` calls updated paper-wide:
  - `target image parameters: section` → `image parameters: section`
  - `control strategy: section` → `control design: section` (matches "MDF-ASMC CONTROL DESIGN" heading; 11 active references updated)
  - `normalized pid: section` → `virtual image point control: section`
  - `ppc optic flow: section` → `optic flow control: section`
  - `fov cone: section` → `target image funnel: section`
  - New label: `yaw control: section` for §III.B.1.
- **Section-ref composition pattern (2026-05-03)** — IEEE TAES `\ref{}` to a subsection returns only the local letter ("C"); to render "III-C" use hardcoded composition: `Section~\ref{section_label}-\ref{subsection_label}\ref{subsubsection_label}`. Parent-collapse rule: when a sentence cites multiple sibling subsections of one parent, cite the parent. See `feedback_section_ref_composition.md`.
- **Paper-wide spelling pass to Oxford/US (2026-05-03)** — `-ised`/`-isation` → `-ized`/`-ization`; `-our` → `-or`; `-tre` → `-ter`; `analysed` → `analyzed`; `centimetres` → `centimeters`; preserve "Robert Bosch Centre" (proper noun). 25+ edits across active files; British forms remain only in `Drafts/` historical snapshots.
- **`/prose-audit` rules updated**:
  - **Rule 6 (`corner`)** rewritten for vocabulary epoch 2 — drops "target" qualifier, switches `_k` → `_i` per-feature, adds `\mu_{ij}` regression check.
  - **Rule 9 (`image-feature`)** rewritten for epoch-3 — locked names "virtual image point error" / "Virtual Image Point PID" / "Virtual Image Point Control Design"; epoch-1/-2 forms (`Normalized Virtual Feature PID`, `virtual image feature error`, `Optic Flow Control Law`, etc.) added as regression checks.
  - **Rule 13 §II.B section-title detection flipped** — locked epoch-3 forms ("Image Parameters", "Virtual Image Position", "Virtual Image Orientation") must hit; epoch-1/-2 forms are now regression checks.
  - **Rule 14 (`citation-classification`)** updated — `chen2025` is now in BOTH the IBVS aerial list (Para 2) AND the adaptive-control list (Para 5); intentional dual classification because its distinguishing feature is the adaptive observer.
  - **NEW Rule 15 (`spelling`)** — paper-wide US/Oxford convention; flags `-ised`/`-isation`/`behaviour`/`centre`/`analysed` etc. (preserves "Robert Bosch Centre").
  - **NEW Rule 16 (`section-ref`)** — flags bare subsection/subsubsection refs without composition, stale label names, sibling-range patterns that should collapse to parent.
- **Updated memories** — `feedback_target_image_parameters.md` (3 epochs naming history + epoch-2 vocabulary table), `feedback_no_landmark_term.md` (full vocabulary table refresh + subscript role lock), `feedback_image_feature_naming.md` (full rewrite for epoch-3 names + "Control Design" subsection pattern), `feedback_section_ref_composition.md` (new memory created), `MEMORY.md` (multiple index entries refreshed).

**Updated 2026-04-22 (earlier) — variable-based funnel naming locked:**
- **Funnel names changed**: "visibility funnel" → **target image funnel** ($\boldsymbol{p}_1(t)$, pixels); "soft-landing funnel" / "optic-flow performance funnel" never used → **optic-flow funnel** ($\boldsymbol{p}_2(t)$, rad/s). Chosen over purpose-based names because Theorem 1 proves invariance of the bound $\boldsymbol{p}_2(t)$ — variable-based naming matches what's proved. See `feedback_funnel_naming.md`.
- **Feature-point naming locked**: tracked image-plane projections are **feature points** (image plane) or **virtual feature points** (virtual image plane); "corner point", "physical corner", "physical feature point", "physical camera frame", "physical image plane" all forbidden. "Physical" qualifier is dropped globally — unqualified terms *are* the physical camera; only "virtual" marks the derived frame. See `feedback_corner_points_naming.md`.
- **Architecture clarification in `project_dual_funnel_parallel_architecture.md`**: the cone clamp is the *enforcement mechanism*, NOT the funnel. Subsection retitle: "Visibility Funnel: FoV-Adaptive Cone Clamp" → "Target Image Funnel and FoV-Adaptive Cone Clamp" (two architecturally distinct ideas).
- **`/prose-audit` rules updated**:
  - Rule 4 (`funnel-naming`) — canonical pair changed to *target image funnel* / *optic-flow funnel*; Rule 4 now flags `visibility funnel`, `soft-landing funnel`, `optic-flow performance funnel`, `inner funnel`, `outer funnel` as stale/forbidden.
  - Rule 6 (`corner-points`) — full rewrite: flags `corner point(s)`, `target image corner point(s)`, `physical corner(s)`, `physical feature point(s)`, `physical camera frame`, `physical image plane`. Canonical: "feature point" / "virtual feature point" / "3D point" (for real-world target).
  - Rule 10 (`funnel-symbol`) — descriptive text changed from "visibility-funnel spelling" to "target-image-funnel spelling"; symbol rules themselves unchanged.
  - Skill description updated: Rule 6 summary changed from "corner-point misuse" to "feature-point naming / drop 'physical' qualifier".
