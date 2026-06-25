# HANDOFF — Velocity-damping / ζ_h thread (2026-06-25)

**For the next chat.** This continues the GT-FB terminal-fly-away thread. The binding failure is now fully understood and the fix is identified and partially validated. Read the linked memory files for detail; this is the operational map.

## TL;DR
- **The terminal fly-away = a 1/Z positive-feedback loop:** `h_xy = v_lat/Z` → control over-reacts → tilt → lateral accel → `v_lat` grows → bigger `h_xy`. Loop gain ∝ 1/Z, so it always wins as `Z→0` unless `v_lat` is already ~0.
- **The fix = arrest `v_lat` BEFORE the deck (velocity damping):** engage `ζ_h` (the dormant flow/velocity barrier in `σ_xy = ζ_h + χ_r·ζ_r`) by tightening the lateral flow funnel. **CONFIRMED + EXTENDED (2026-06-26):** XI2_xy 0.2→0.5 (6/2/2) → **0.7 + P20_xy=15 → GT-FB IC1-5 n=2 = 10/10 BOUNDED, ZERO fly-aways**; XI2_xy=0.9/P20=10 fixes the high-start IC4 (xy 2.84→1.04).
- **`h_xy` cannot be made to *not* spike** (geometric 1/Z at `Z→0`). The goal is to push the "safe floor" (altitude where the loop triggers) below touchdown. `ζ_h` moved it ~1 m → **~0.08 m** at XI2_xy=0.7. **Metric = safe-floor altitude, NOT `hxy_deck` magnitude.**
- **⭐ REFRAME (2026-06-26): the reported `xy_err` is NOT a precision failure.** The controller touches down DEAD-CENTERED (5/10 reps lat 0.005–0.09 m at min-Z, incl. all the "flys"); the 1/Z kick then balloons the landed drone 1–4 m. Score min-Z, not the post-kick endpoint. The genuine residual is the terminal VELOCITY (vlat 0.5–1.5, vz 0–1.3 m/s) ≈ the 38 ms lag. The lateral position problem is SOLVED.
- **`h_rd` is CONSTANT (−0.42), always.** The `vz = h_rd·Z` taper IS the soft-landing mechanism. Never reference-govern / proximity-gate it.

## What's baked (current default config)
- `PLASMC_COMBINED_BARRIER=1` (σ_xy = ζ_h + χ_r·ζ_r; auto-aligns Γ/E/κ₀/XI2/P2INF to MATLAB `vdf_params`).
- `chi_r=0.5` (PX4-specific; do NOT raise toward MATLAB's 2.0 — see below).
- **`N_xy=0.1` BAKED this session (commit 826e655)** — woke κ_xy (τ 33s→7s). ⚠ Regresses *alone* (woken κ amplifies the terminal); safe only WITH the velocity damping.
- `h_rd=−0.42`, `E=[1,1,0.5]`, `P2INF=[0.5,0.5,1.5]`, `XI2=[0.2,0.2,0.2]` (baked), CH_CLEAN, FLOW_LAT_REDUCED, the diagonal recal, the GT-FB w_z sign fix.

## Key results this session
1. **`ζ_h` velocity-damping lever WORKS** → [[feedback_flow_funnel_zetah_works]]. `XI2_xy=0.5` → ζ_h 7%→18%, v_lat arrested, **6 sub/2 marg/2 fly**, no limit cycle (loom cycle *suppressed*), κ healthy 4 axes. Bundle `20260625-202813`.
2. **MATLAB gains DON'T transfer to PX4** → [[feedback_matlab_gains_not_portable]]. Porting `vdf_params` (χ_r=2.0, N=0.02, P2INF_xy=1.0, p20_z=4) → **1/2/7 fly** (IC1 centered 18 m, a_u_xy=2M). The 38 ms **lag** leaves a terminal residual χ_r=2.0 over-reacts to. PX4 χ_r=0.5 is the ceiling. Bundle `20260625-191808`.
3. **s_e_n & h_e boundedness = the SP itself.** Both converge+bounded in descent, both diverge at the deck. `s_e_n=lat/Z` bounded ⟺ `lat→0` (precise); `h_e=v_rel/Z` bounded ⟺ `v_rel→0` (soft). The lag breaks both → they fail *together* (coupled via tilt). → [[feedback_sp_task2_terminal_limit_cycle]], [[feedback_terminal_smc_actuator_wall]].

## DONE this round (2026-06-26) — see [[feedback_flow_funnel_zetah_works]] + `project_current_state.md`
1. ✅ **`XI2_xy=0.7` + `P20_xy=15`** → GT-FB IC1-5 n=2 = **10/10 bounded, 0 fly-aways** (bundle `20260625-213624`). Safe-floor ~0.08 m.
2. ✅ **`XI2_xy=0.9` + `P20_xy=10`** → fixes the high-start IC4 (mean xy 2.84→1.04; bundle ~`20260625-21xxxx`). rel_vel still ~1.4 (position lever can't touch terminal velocity).
3. ✅ **IC2_rep1 traced:** NOT a v_lat-arrest failure (v_lat arrested to Z=0.11) — a terminal tilt→GT-flow positive-feedback at Z<0.1 m (κ_x asleep through the clean descent → woke too late; θ 2°→396°). Hypothesis "woken-κ N=0.1 transient" partly confirmed (κ wakes late, not over-woken).
4. ✅ **Z-funnel A/B (XI2_z=0.6, P2INF_z=0.5; bundle `20260625-235528`):** the combined auto-align HAD loosened p_z (P2INF_z 0.5→1.5, XI2_z 0.6→0.2). Tightening scales `|h_e_z|` down 4× but `h_e_z` rides the funnel at a fixed ~0.6 ratio and does NOT systematically reduce terminal vz + adds mild bounded z-chatter → NOT the soft lever. Terminal vz is lag-set.

## NEXT STEPS (open)
1. **The terminal lateral kick magnitude** (θ detonates at Z<0.1 m, a_u→1000s) is the irreducible Z→0 residual — the controller is already dead-centered at min-Z, so the lever is likely a **touchdown-disarm / commit gate** (stop commanding through ground contact), NOT more funnel. Check `landing_test.py` scores min-Z vs the post-kick balloon.
2. **n≥5 confirm of `XI2_xy=0.7`**, then **perception-ON** (drop `PLASMC_GT_FEEDBACK`) — the actual deployment condition — THEN consider baking `XI2_xy`.
3. Terminal velocity (soft gap) ≈ 38 ms lag → the genuine remaining lever is the uXRCE-DDS rate-path (see [[feedback_dds_lag_fix_blocker]]), not control gains.

## Reproduce
```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo
# the confirmed ζ_h gate (GT-FB):
HEADLESS=1 PLASMC_GT_FEEDBACK=1 \
  PLASMC_XI2_X=0.5 PLASMC_XI2_Y=0.5 PLASMC_XI2_Z=0.2 \
  N_REPS=2 bash scripts/run_ic_validation.sh > run_logs/ic_funnel_zetah.log 2>&1
# next: change XI2_X/Y to 0.7
```
- ⚠ **Launch gotcha:** never `pkill -f "gz sim"` in the same shell that launches — it matches its own command line and self-kills. Clean stragglers in a *separate* command.
- ⚠ **Auto-align gotcha:** set ALL THREE axes of any VDF-aligned param (XI2/Gamma/E/kappa0/P2INF). A single-axis env BYPASSES the combined-barrier auto-align (controller.py ~L265-277) and reverts the other axes to the hot pa-defaults.
- ⚠ **GT-FB self-eval:** `PLASMC_GT_FEEDBACK=1` feeds V-frame GT s/h to the controller (isolates control from perception). Analysis can compare consumed (GT) vs true GT.

## Analysis snippets (the metrics that matter)
- **ζ_h share** of σ_xy in the descent (engaged? want >30%); **p2_xy(t)** contraction (want ~1-2).
- **v_lat vs altitude** (SP condition: <0.5 m/s @ 1 m = arrested = clean).
- **safe-floor altitude** = where the terminal `v_lat`/`h_xy` loop triggers (NOT hxy_deck magnitude — that always spikes at Z→0).
- **limit cycle:** `sat(σ/E)` saturation fraction + velocity sign-flips per axis (cycle = 10+ flips sustained; clean = 1-3).
- **κ health 4 axes:** `kappa(t)` (x/y/z) + `kappa_a(t)` (yaw); descent std (adapting?) + terminal max (exploding = fly-only loop).

## Dead-ends (do NOT re-try)
- κ caps / Γ / θ_cap / clamps on the terminal SMC reaching side (actuator-bounded wall — they fail/backfire).
- Raising χ_r toward MATLAB's 2.0 (lag → catastrophic).
- Reference-governing / proximity-gating `h_rd` (breaks the soft-landing taper).
- Single-sub-term c-term caps (whack-a-mole; all fed by the same flow).
