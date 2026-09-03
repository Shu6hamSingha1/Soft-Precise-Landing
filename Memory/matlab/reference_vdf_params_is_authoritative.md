---
name: reference-vdf-params-is-authoritative
description: MATLAB/VDF_ASMC/vdf_params.m holds the manuscript's locked gains — the inline K_ctrl assignments in visualControl_IBVS_adaptive.m are PRE-RE-BAKE and reading them produces phantom manuscript errors
metadata:
  type: reference
---

**The paper's Table `sup:control params` is sourced from `MATLAB/VDF_ASMC/vdf_params.m`.**
Verified 2026-09-03 — every row matches: `P.p_r0=[1.2;1.2]`, `P.p_rinf=[0.85;0.85]`,
`P.chi_r=[2.0;2.0]`, `P.chi_z=0.1`, `P.Gamma=diag([0.4375,0.5,0.75])`,
`P.Pleak=diag([0.5,0.5,1.5])`, `P.N=diag([0.10,0.10,0.10])`, `P.E=diag([0.5,0.5,0.5])`,
`P.h_rd=-0.42`, `P.Omega_a=P.Gamma_a=0.25`, `P.theta_cap=deg2rad(60)`.

⛔ **Do NOT read the inline `K_ctrl.*` assignments in
`MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m:59-212` as "MATLAB's locked values."**
They are PRE-RE-BAKE and disagree with the paper on ~9 rows (`E`, `N`, `Pleak`, `chi_r`,
`p_rinf`, `Omega_a`/`Gamma_a`, `xi_r`, `p_2inf` …). Comparing them against the manuscript
produces a full page of phantom errors — done on 2026-09-03 and retracted in full. It also
generates two seductive false patterns worth naming so they are not "rediscovered":
- *"`𝒩` tracks PX4 while `Γ`/`θ_cap` track MATLAB, so the table was patched from two sources"* —
  no; that is simply what a pre-re-bake comparison looks like.
- *"`𝒳`/`Omega` is a gain in the code that is missing from the table"* — no; `vdf_params.m`
  has **no `P.Omega`** at all. The surface uses `chi_r`/`chi_z`, exactly as the manuscript states.

**Also verified against MATLAB (same session), so don't re-open these either:**
- `θ_cap = 60°` is correct AND its "2× hover-thrust margin" justification holds:
  `Constants.m:28 T_max = 60.0` N, m = 2.114 → **2.896 g** available; 60° demands 2.00 g
  (41.4 N); the paper's own condition `T_max ≥ mg/cos θ_cap` is satisfied with 1.45× headroom.
  (PX4's `A_CAP` = 1.389 g cannot deliver it — that is why PX4 bakes the derived 43.94°. A
  platform difference, not a manuscript error. See [[project_two_output_cals_aruco_vs_cross]]
  for the same shape of ArUco/cross confusion.)
- The formulation section does NOT describe a controller the results didn't run: funnel-ref
  `h_d` exists in `flow_surface.m:25`, and the theta-QP the paper describes is
  `cbf_visibility.m`. Only the joint QP, `a_z` relief and `k_r` are PX4-only, and the
  manuscript never claimed them.
- `flow_surface.m` has NO `-k_r·G_r^-1·ζ_r` term, so the manuscript's three "no back-mapped
  rate" statements are accurate. The `h_e = G_r^-1(ζ̇_r + k_r ζ_r)` identity is a property of
  the **PX4** law only.

**⚠ MATLAB↔PX4 yaw-gain divergences are DELIBERATE — do not "sync" them.**
- `Ω_a`: PX4 0.1 vs MATLAB 0.25 — derived, `controller.py` ~531-543: `u_a` is a yaw-RATE command
  so the rate structure already integrates `e_a`; `Ω_a·ie_a` added a SECOND integrator with no
  phase margin against PX4 inner-loop lag (K_R_YAW + rate loop + `tau_ua` LPF) that MATLAB does
  not have. 0.1 removed the yaw limit cycle (ncross 5-6→2).
- `Γ_a`: PX4 0.5 vs MATLAB 0.25 — **no derivation was ever recorded.** Flagged as unjustified in
  the 2026-09-03 audit; **user resolved it by declaring the PX4 SITL value the BASELINE** rather
  than reconciling to MATLAB. Do not change it toward 0.25 and do not treat the gap as a defect.
  It has never been swept, so it stays a legitimate tuning target on its own merits — and it sits
  in the loop the Q8 turning-rover yaw work touches, so any change there A/Bs against **0.5**.

**Root cause of the 2026-09-03 error, worth generalising:** `CLAUDE.md`'s repository map did
not list `MATLAB/VDF_ASMC/`, so the parameter file was never found and the nearest-looking
file was used instead. Map now fixed. **Treat CLAUDE.md's directory listing as a starting
point, not an inventory — `find`/`grep` for the file that actually defines a value before
concluding anything from a lookalike.** Same lesson as
[[feedback_verify_injected_docs_before_trusting]], one level up: there it was a stale value in
a doc, here a missing entry in a map.
