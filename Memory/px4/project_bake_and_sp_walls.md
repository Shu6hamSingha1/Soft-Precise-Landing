---
name: project_bake_and_sp_walls
description: "2026-06-29 BAKE (787cf2d) of the stability config to controller.py defaults + two fresh dead-ends (h_rd=-0.3, HD_KR=1.0) + the SP-two-walls synthesis"
metadata: 
  node_type: memory
  type: project
  originSessionId: 13b15a0c-9903-4e62-95a5-6161b516a73e
---

**2026-06-29 session handoff.**

## ⭐ BAKED the stability config to controller.py defaults (commit 787cf2d, pushed)
The whole env-gated stability stack is now **default-ON** (no env needed). 11 edits:
- **Structural flags → "1":** `TERMINAL_COMMIT` (s_e_n→0 ramp), `TOUCHDOWN_LOOM` (loom-inversion touchdown detect + landing_test.py disarm), `SAVGOL_PREDICT` (PREDICT_MODE=model), `HD_FUNNEL_REF`.
- **Gains:** `CHI_R` 0.5→1.5, `PRINF` 1.0→0.8, `HD_KR` 0→0.5, `P20` 25→15 (x/y); combined-barrier auto-align (line ~273-283) `KAPPA0` 0.125→0.5, `XI2` 0.2→0.7/0.7/1.0, `P2INF` 0.5→1.0 (x/y).
- Unchanged: `GAMMA` 0.4375/0.5/0.75 (VDF), `E` 1.0/1.0/0.5, `combined_barrier=1`. `GT_FEEDBACK` stays TEST-ONLY (default uses perception).
- **State:** GT-FB **12/12 land, 0 fly-aways** across IC1-5; **0/12 SP at IC2-5** (SP is the open work). Live no-env SITL confirm was BLOCKED by a persistent `/clock` infra flake ("Unable to get simulation time") — bake verified by syntax+import+value-equivalence; **user should do one no-env run to confirm the 4 feature prints when SITL is healthy.**
- ⚠ SITL gotcha: never put `pkill -9 -f 'gz|px4|MicroXRCE'` INSIDE a backgrounded Bash tool call — it kills the task's own runner (no log, exit 1). Clean in a SEPARATE foreground call first.

## Two fresh DEAD-ENDS (don't re-try)
- **`h_rd=-0.3` (LANDING_REF_RAD_OPT_FLOW): REGRESSES** → fly-aways (IC2 13m, IC3 **137m**, IC3 no-land). The fly-away is a CLIMB-away (drone ascends Z 2.25→58m). ROOT: `h_rd` is NOT an independent descent knob — it DRIVES lateral convergence via the FF `~h_rd·s` and `k_lat=|h_rd|+k_r`. Lowering it STARVES lateral convergence (rate 0.66→**-0.05** m/s, 5→2m band) → off-center at deck → 1/Z divergence. Keep baked **-0.42**.
  > ⛔ **2026-06-30 SUPERSEDED — this was a BACK-MAPPED-config property; the COMBINED SURFACE dissolved it.** That `k_lat=|h_rd|+k_r` / `ds_d`+`h_rd·s` coupling was the back-mapped surface — now REPLACED by the baked combined surface `σ=ζ_h+χ_r·ζ_r`, where the lateral is driven by the position barrier `ζ_r` DIRECTLY (h_rd-INDEPENDENT; `h_rd·s` survives only as a small descent FF, negligible when centered). So lowering `h_rd` is now (mostly) a pure descent change and does NOT starve lateral convergence. EMPIRICAL (q=1+W_U_MAX=2.0+combined base): **h_rd=-0.35 & -0.30 → IC5 SP (xy 0.05, rel 0.01-0.09, NO fly)** — the slower descent gives IC5 the runway to converge, opposite of the old starvation. So `h_rd` is now usable as a slow-descent knob (constant-h_rd-per-rep still holds — it's a fixed value, not time-varying). **✅ CONFIRMED (IC1-5 n=1, 2026-06-30): NO fly-aways at h_rd=-0.30 OR -0.35** — IC2 (0.117) and IC3 (0.130) LAND CLEAN (the old "IC2 13m/IC3 137m climb-away" is REFUTED; combined surface dissolved the coupling). BUT global h_rd reduction is NOT a net SP win: -0.30 helps IC5 (SP) but makes the higher ICs (IC2/IC3) imprecise (0.12-0.13 soft-not-precise) — they don't need the runway; -0.35 worse (2/5 SP, IC1/IC3/IC4 neither). Baseline -0.42 (6/9) still best overall; -0.30/-0.35 just REDISTRIBUTE (fix IC5, cost IC1-4). So h_rd is a SAFE knob but a single global value can't optimize all ICs (IC5 wants slow, IC1-4 want -0.42); n=1 noisy, net effect needs n>=5. Keep baked -0.42; reach for slower h_rd only if IC5-class (short-runway) cases dominate.
- **`HD_KR=1.0` (raise k_r): WORSE** → peak_tilt 32→36°, v_z_td 3.16→4.15, s_e_n WORSE both axes (x 0.17→0.31). Higher k_r amplifies the TERMINAL `-k_r·ζ_r/g_r` 1/Z command. Keep baked **0.5**. (Confirms old "higher HD_KR worse".)
- **velocity-gate touchdown detector: REJECTED** (band-aid; if `|h_e_xy|<v_thr` never met it never fires → controller pumps the bounce → fly-away returns).

## SP = TWO INDEPENDENT WALLS (this session's synthesis)
1. **PRECISION wall = off-center `s_e_n` never converges.** Restoring authority `ζ_r/g_r` PEAKS at S_r=0.649 then VANISHES at funnel edge ([[feedback_sen_authority_analysis]]). IC5 spawns at S_r≈0.66 (lat2.8/Z3) and descent drives it to S_r=**18** (54% frames past peak, anti-restoring). `lat` DOES converge (2.73→1.27) but slower than Z → s_e_n=lat/Z diverges. The vertical "tilt→thrust→fall" (7/12 IC2-5, peak tilt>30° → B_T·cos(tilt) below hover → near-free-fall v_z 5-8) is this SAME 1/Z, expressed as a vertical↔lateral coupling gain `∂s_e_n/∂Z=-lat/Z²` (1/Z², sub-critical at altitude, super-critical <0.3m). Fix = MONOTONE restoring authority = **stacked-barrier backstepping** (move ζ_r into the switching surface, which doesn't vanish at the edge) — see [[project_stacked_barrier_backstepping]]; caveat IC5 may be under-actuated (3m for 2.8m offset, inner tilt/thrust also limited — SEN_RECOVER_ST was inert for this reason).
2. **SOFTNESS wall = `h_e` boundary-layer floor.** `h_e` plateaus at ~0.5-0.6 on ALL axes because `σ_med ≈ E` (SMC holds the surface AT the boundary layer E=1/1/0.5, not 0). Structural (E + funnel p). The terminal LATERAL touchdown velocity is CONTACT-generated (gear strike az ±20-66 at Z~0.2m), NOT control (a_u small) nor impact-coupled (corr(v_z,v_lat)=0.11). Lever = graded E(σ) OR the lag (DDS).

## Corrections logged this session
- B_T is NOT thrust-saturated — `B_T=m·(I_a_z+g)/cos(roll)/cos(pitch)` HAS tilt comp; ~18N has headroom. The descent is COMMANDED (I_a_z≈-3), not thrust-limited.
- Per-axis: **X is the hot lateral axis** (s_e_n_x/kap_x/sig_x > y). Keep per-axis gains.

NEXT: the only lever not yet dead is the **stacked-barrier surface** (monotone s_e_n authority). See [[project_stacked_barrier_backstepping]], [[feedback_backstep_tried_clamps_are_lever]] (backstep h_d over-demands off-center — the surface route avoids the unbounded inward rate).
