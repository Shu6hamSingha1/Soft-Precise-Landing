---
name: Approach 2 — Funnel-margin cone clamp (implemented, committed)
description: DF-ASMC revision with state-dependent cone driven by shrinking target image funnel; propagated across Multi_init + Comparison harnesses; term "funnel-margin cone clamp" locked 2026-04-22 (prior "FoV-adaptive" label superseded)
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Approach 2 replaces the fixed 35° cone clamp + visibility funnel with a state-dependent cone angle driven by the physical-corner distance from a shrinking FoV box.

**Architecture (implemented in Multi_init_cond only):**
- REMOVED: Visibility Funnel (symmetric transformation on virtual features) — `p_1`, `dp_1`, `S_1`, `G_1`, `zeta_1`, `izeta_1`, `raw_dzeta_1` arrays all deleted
- REMOVED: `K_ctrl.gamma_1`, `K_ctrl.p_1inf` fields
- REPLACED: Barrier PID on transformed `ζ_1` → raw-error PID on normalized `V_s_e_n = V_s_e ./ K.p_10` (sensor-half normalization)
- REPLACED: Fixed 35° cone → `θ_cone = min(θ_current + atan(d_min/f), θ_cap)` where `θ_current = acos(R(3,3))`, `θ_cap = 60°`
- ADDED: Shrinking FoV box `ρ_fov(t) = (ρ_fov_0 − ρ_fov_∞)·exp(−l_fov·t) + ρ_fov_∞` on physical corners `C_nP`; `d_min` is min axis-wise distance of any of 4 corners to box edge
- NEW logs: `V_s_e_n`, `iV_s_e_n`, `raw_dV_s_e_n`, `rho_fov_log`, `d_min_log`, `theta_cur_log`, `theta_cone_log`
- REMOVED logs: `zeta_1`, `p_1`, `dp_1`, `S_1`, `G_1`, `izeta_1`

**Committed parameters (as of d72ab4d, 2026-04-18):**
- `K_ctrl.rho_fov_0 = res/2 = [160; 120]` px (sensor half)
- `K_ctrl.rho_fov_inf = [40; 40]` px — **widened from initial 15 px** because 15 collapsed d_min to 0 and killed cone authority
- `K_ctrl.l_fov = 0.1` (1/s)
- `K_ctrl.theta_cap = deg2rad(60)`

**Current realistic-mode result (2026-04-18 retune):** 25/25 landed + 25/25 precise + 25/25 soft across all five trajectories — **recovered the OLD 25/25/25** via three-knob retune (zd=1.15, E(3,3)=1.0, h_rd=-0.42) plus Omega(3,3) 4× bump. See project_multi_init_final_lock.md. Previous regression (23/22/21) was driven by IC4 hover-fail; now resolved.

**Why:** Old formulation guaranteed *virtual* feature boundedness only; physical pixels overshot FoV on aggressive ICs (IC5). Approach 2 provides a proved *physical* FoV guarantee — IC5 stress test confirmed max physical pixel 113 (under 120 limit) vs prior >120.

**How to apply:**
- Multi_init harness: DONE (commit 6f8e4e5 architecture + d72ab4d rho_fov_inf tune)
- Comparison harness (`visualControl_comparison.m`, `InitGains_Comparison.m`): NOT YET propagated — user deferred until single-harness validation stable
- Stability proof (new Theorem 1): still deferred
- Paper narrative rewrite (Section III + block diagram): DONE 2026-04-22 — §III-B.4 titled "Target Image Funnel and Funnel-Margin Cone Clamp"; block-diagram node relabelled "Funnel-Margin\\Cone Clamp"; all prose instances of "FoV-adaptive cone clamp" / "FoV-adaptive projection" renamed across `control_formulation.tex`, `results.tex`, `supplemental.tex`, `block_diagram.tex`, Python/MATLAB comments
- `plotter_adaptive.m` refactor for new log fields: still deferred (currently commented-out in both files)

**Pending issues:**
- IC4 hover-fail RESOLVED (Omega(3,3) 0.006 → 0.025, 2026-04-18). See project_ic4_z_hover_fail.md
- OLD dataset backup preserved at `Obsolete/Multi_init_cond/Datasets_v1_preApproach2/`
- Comparison harness + plotter refactor + stability proof + paper narrative: still pending
