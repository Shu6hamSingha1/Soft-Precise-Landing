---
name: baseline-swap-chen2025-to-lin2023
description: "2026-05-31 — Comparison baseline Chen 2025 (Controller 4, IBVS) is being replaced by Lin 2023 (robust circle-feature IBVS) on ethical grounds. In progress; MATLAB sims + tex rewrite still pending."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

**Decision (2026-05-31):** Replace comparison baseline **Chen 2025** with **Lin 2023**.

**Why:** Chen 2025 (*"Image-Based Visual Servoing of MAVs…"*, IEEE TCST) is an IBVS *regulation* paper with **zero mention of landing** — yet `results.tex` framed it as a *"representative landing controller."* Comparing a non-landing paper as a landing baseline (and reporting it "fails to land") misrepresents a claim Chen never made — ethically wrong. The other three baselines are landing papers by title (Lin2022, Zhang2026, Cho2022); Chen was the lone non-landing one.

**Replacement = Lin 2023** — Jie Lin, Yaonan Wang, Zhiqiang Miao, Hesheng Wang, Rafael Fierro, *"Robust Image-Based Landing Control of a Quadrotor on an Unpredictable Moving Vehicle Using Circle Features,"* IEEE Trans. Autom. Sci. Eng. 20(2):1429-1440, 2023, DOI 10.1109/TASE.2022.3180506. Bib key `lin2023` (added).

Why Lin 2023 fits:
- IBVS + landing both in title -> preserves the deliberate **2 PBVS (Lin2022, Zhang2026) + 2 IBVS (Lin2023, Cho2022)** split. See [[feedback_citation_classification_audit]].
- Already uses a **geometric SO(3)** attitude inner loop -> drops into the shared inner loop.
- Depth-free circle-moment features (Z*_V absorbed in gains); robust to unknown target velocity.
- Caveat: same Jie Lin / Fierro group as `lin2022` (PBVS). Defensible: benchmark one leading group's PBVS *and* IBVS landing designs.

**Lin 2023 profile differs from Chen's** — differentiators vs VDF-ASMC shift away from "needs metric depth" toward:
1. Performance funnel **fixed a priori** (no online identification).
2. Funnel is on **image-feature error**, not optic-flow / touchdown velocity -> **no soft-touchdown certificate**.
3. Uses **own metric velocity feedback** (Eq. e_v = v - vhat); VDF-ASMC uses optic flow, no metric velocity.

## State of the swap (updated 2026-05-31, integration done)
DONE:
- `MATLAB/Comparison/ctrl_Lin2023.m` rewritten to MIRROR proven `ctrl_Lin2022.m` (log-barrier BLF backstepping + NED force sign + SO(3) inner loop), with the OUTER loop on circle-moment image features. This is the IBVS-vs-PBVS distinction.
- `ctrl_Chen2025.m` -> `Obsolete/Comparison/MATLAB/ctrl_Chen2025_v0.m` (git mv). Active files backed up to `_v2` before edit.
- `bibliography.bib`: `lin2023` added. `CLAUDE.md` updated. Excel (Baselines+Landscape) updated.
- HARNESS INTEGRATION COMPLETE (`visualControl_comparison.m`): gain-select line 79 -> K_Lin2023; CTRL_SEL==4 state-init -> rho_t0_lin/rho_v0_lin=[]; case-4 block builds circle-moment features + adaptive funnels + calls ctrl_Lin2023. `InitGains_Comparison.m`: K_Chen2025 block replaced by K_Lin2023. `run_comparison.m` ctrl_names{4}='Lin 2023' (note: harness line 57-58 still cosmetically prints 'Chen 2025').
- VERIFIED RUNS + DESCENDS CORRECTLY (Static, seed 1002): vz~+0.52 m/s (soft, NED-correct), T~19.5N, lateral vel <=0.15 m/s, no NaN.

KEY INTEGRATION DESIGN CHOICES (made by me; user should review):
1. s_t built from simulated image points V_nP_i (NOT ground truth) -> stays image-based. Centroid normalized by f (px/f). Area ratio an from polyarea.
2. NED ADAPTATION: an = sqrt(a_img/a_des) (RECIPROCAL of Lin's ENU sqrt(a*/a)) so an<1 when high -> e_t(3)<0 when high -> descends, matching ctrl_Lin2022's NED sign. Using Lin's literal sqrt(a*/a) made the UAV CLIMB (sign-inverted); this was the fix.
3. Own-velocity feedback I_v_c with sigma_vel noise (Lin2023 uses velocity; treated like PBVS baselines).
4. Adaptive funnel rho(0)=|e(0)|+margin (mirrors Lin2022).

SIGN BUG FOUND + FIXED (2026-05-31): the smoke-test stall at 3.1m was NOT structural — it was a LATERAL SIGN BUG. I had mirrored ctrl_Lin2022's vhat = -k1*(...) sign, but image-feature dynamics (Lin Eq.8 ds_t=-(1/Z*)(v-v_t)) carry an extra inversion vs position dynamics, so the correct law is vhat = +k1*(...). The wrong sign drove the UAV laterally AWAY from target; the funnel pinned it at a steady offset that grew with gain (the tell: higher k1 made e_t1 WORSE, not better -> not a disturbance offset). FIX: (a) ctrl_Lin2023 vhat sign -> +k1 (.* per-axis); (b) feature reverted to Lin's literal an=sqrt(a*/a); (c) harness rho_v0-init line mirrored (+k1 .*); (d) k1 made per-axis [3x1] because literal an makes depth error large (~12) vs lateral (~0.03); (e) rho_t0_margin per-axis [1.5;1.5;5.0] so the large depth feature isn't born at the barrier.

FINAL BEST-EFFORT TUNED GAINS (Static, seed 1002): soft descent 5m -> 0.22m over full 40s, NO FoV loss, vz~0.010 m/s (very soft), target centered (lateral e_t~0.22 normalized). FoV loss RESOLVED.
  K_Lin2023.k1=[0.4;0.4;0.40]  (per-axis: lateral 0.4 centres; depth 0.40 is FoV-safe sweet spot, 0.60 reintroduces FoV loss)
  k2=4.0; rho_inf_t=[0.10;0.10;0.03]; rho_inf_v=[0.30;0.30;0.15];
  l_t=[0.05;0.05;0.10]; l_v=[0.03;0.03;0.10];
  rho_t0_margin=[1.5;1.5;5.0]; rho_v0_margin=1.5; kR/kOmega shared; psi_des=0.

TUNING TRAJECTORY (Static): bug-fix -> 0.45m stall (k1z=0.20); raised k1_xy 0.2->0.4 (centred, e_t1 -0.35->-0.22); moderate depth k1z=0.28 -> 0.25m; k1z=0.40 -> 0.22m (locked). k1z=0.60 OR fast l_t(3)=0.12 -> FoV loss returns (fast descent outruns centring). rho_inf_t(3) 0.10->0.03 had no effect (funnel not binding at t=40; stall is proportional-asymptote, not funnel).

RESIDUAL (honest baseline limitation, FAIR result): asymptotically stalls ~0.22m (2cm short of 0.2m termination) because the PPC proportional law vhat_z ~ e_t(3) -> 0 as an->1 (vanishing terminal velocity = good softness, slow touchdown). Plus residual LATERAL offset ~0.22 normalized from NO disturbance-rejection integrator -> soft approach but NOT precise. Pushing depth harder to force <0.2m reintroduces FoV loss. This cleanly distinguishes VDF-ASMC (adaptive SMC rejects disturbance + cone clamp preserves visibility -> soft-precise).

FULL 5-TRAJECTORY COMPARISON (run_comparison_all, seed 1002 = IC2; tuned gains above). Lin 2023 (slot 4) per-trajectory final horizontal error / steps / time:
  Static:     0.26 m / 4000 / 40.0s  -> soft descent, no break, BUT err>0.08m (no-integrator lateral offset) => soft NOT precise.
  Linear:     4.46 m / 303  / 3.0s   -> breaks early (FoV loss).
  Sinusoidal: 1.27 m / 1473 / 14.7s  -> breaks.
  Lissajous:  5.69 m / 54   / 0.5s   -> breaks almost immediately.
  Circular:   0.81 m / 1905 / 19.0s  -> breaks.
HEADLINE: Lin 2023 manages a soft-but-NOT-precise approach on the STATIC target only, and FAILS on ALL moving targets (early FoV loss, 0.8-5.7m error). Clean moving-target soft-precise gap vs VDF-ASMC. (Other baselines from same run: Lin2022 1.4-4.0m fails; Zhang2026 ~0.27m on moving but precise-not-soft per [[project_zhang2026_failure_mode]]; Cho2022 breaks immediately ~5m.)

LABEL FIX: harness/plotter/sweep/MC still printed stale 'Chen 2025' for slot 4 (numbers were correct Lin 2023 — Chen .m removed so no error possible). Fixed ctrl_names in visualControl_comparison.m L57-58 + comments L11/L52, plotter_comparison.m L44/L51, multi_speed_comparison.m L27, run_monte_carlo.m L18. Re-ran run_comparison_all for clean labels (numbers identical/deterministic).

FULL IC SWEEP (analyze_lin2023_ICs.m, 5 ICs x 5 trajs, IC_OVERRIDE + per-IC seed): Lin 2023 = 0/25 soft-precise, 0/25 landed (best gap 0.21m), 0/25 precise (best pxy 0.147m). Static soft-stalls (not precise) on IC1/IC2 only; IC3/IC4/IC5 break early; ALL 20 moving cells fail (FoV loss / divergence). IC4 [2,2,-7] breaks at step 1 on all trajs (IC2-tuned funnel margin doesn't generalize to 7m start -> conditioning artifact, not the structural ceiling).
TUNING-HEADROOM VERDICT (answers "can tuning improve soft-precise?"): NO. Two structural limits: (1) no disturbance-rejection integrator -> steady lateral offset under wind, pxy plateaus ~0.15m then destabilizes, NEVER reaches 0.08m precise bound; (2) no visibility-preservation mechanism -> moving targets lose FoV. Per-IC retuning can rescue conditioning breaks (IC3/4/5) up to soft-stall-not-precise, never past. Exactly the VDF-ASMC gap (adaptive SMC -> precise; cone clamp -> visibility). Gains were best-effort tuned at IC2/Static per the locked-gain protocol.

TEX UPDATED (2026-06-01): all chen2025 -> lin2023 across manuscript.tex, results.tex, control_formulation.tex, supplemental.tex (0 chen2025 remain). Lin 2023 Table V row (IC2, paper idx-1 convention, validated vs PLASMC/Lin2022): Static 39.99/0.147/0.216, Linear 3.02/2.204/3.872, Sin 14.72/0.913/0.874, Liss 0.53/2.748/4.979, Circ 19.04/0.701/0.400 (all dagger=no land; 0/5 precise). Per-baseline failure (results L208) + Table VI row + §S3-E subsubsection rewritten to Lin 2023's STRUCTURAL story (no integrator -> steady lateral offset, never precise; no visibility mechanism -> moving-target FoV loss; 0/25 IC sweep). Speed sweep (80-run, re-run 2026-06-01): Lin 2023 0/20, visibility-break envelope 2.4-4.7m at 1.2-5.1s; 0/80 across baselines holds (L379/L467 updated). §I drawback (L112) + 3-gap analysis (L120, gap ii moved lin2023 to "fixes funnel a priori" group, NOT adaptive-observer) + Table I row + Remarks 4/8 updated. Gain-table + S3-E reworded honestly: lin2023 implemented from control law (paper gains NOT tested for crash), structural limitation established across gain/IC sweep. Backups: Drafts/{manuscript_v4,results_v3,control_formulation_v4/v5,supplemental_v4}.tex.

PENDING (user's domain):
- PYTHON plots: make_comparison_plots.py + make_comparison_multi_speed_plots.py baseline labels + citation numbers ('Chen 2025'/[10] -> 'Lin 2023'/new bib number); regenerate comparison PDFs. NOTE lin2023 added to bib shifts citation numbers (see [[project_baseline_citation_numbers]]).
- _analyze_results_for_tex.py has stale hardcoded 'Chen 2025' ctrl name (cosmetic; numbers read fine from datasets).
- Scratch files to remove before commit: Comparison/diagnose_stall.m, Comparison/analyze_lin2023_ICs.m.
- Confirm clean re-run numbers, then PYTHON plots (citation numbers) + TEX rewrite (results.tex per-baseline failure analysis must describe Lin 2023's ACTUAL failure: soft-not-precise on Static, FoV-loss on moving; manuscript.tex Table I row + §I drawback + gap analysis i/ii/iii).
- 80-run multi-speed sweep (multi_speed_comparison.m) not yet re-run with Lin 2023.
- BACKUPS this session in Obsolete/Comparison/MATLAB/: ctrl_Lin2023_v0/_v1; visualControl_comparison_v3/_v4/_v5; InitGains_Comparison_v3..v15; plotter_comparison_v2; multi_speed_comparison_v1; run_monte_carlo_v1; run_comparison_v2. diagnose_stall.m scratch file in Comparison/ — REMOVE before any commit. InitVar.m:57 has a stale 'Chen2025' comment (cosmetic, a* still used).
- After tuning: re-run single-run + 80-run multi-speed sweep -> regenerate Controller-4 `.mat`.
- Python: baseline labels + citation numbers (4 dicts / 2 scripts), regenerate combined PDFs. See [[project_baseline_citation_numbers]].
- tex (needs real sim data — do NOT invent failure mode): `results.tex` L136/L208/L222/L227 + Table IV; `manuscript.tex` L112/L120/L127/L152 (Table I row re-score); supplement S3-D/E + Table S2.

## Supersedes / update when complete
These memories still describe Chen as a live baseline — stale once swap completes: [[project_comparison_study]], [[project_baseline_final_results]], [[project_chen2025_limitation]], [[project_baseline_citation_numbers]], [[project_comparison_reconciliation_history]].
