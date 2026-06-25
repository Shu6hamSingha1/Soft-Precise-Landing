---
name: project_descent_terminal_failure
description: ⭐⭐ HANDOFF (2026-06-24) — the binding IC1/IC4 failure is a TERMINAL DESCENT-CONTROL failure (loom collapse → κ_z over-brake → balloon → 1/Z lateral runaway), NOT h_rd, NOT the funnel, NOT perception, NOT the (fixed) lateral cycle. State + dead-ends + the real lever + env traps for a fresh chat.
metadata:
  type: project
  originSessionId: e265057f-9381-4119-bd0b-0fb5e3c7498f
---

**Frontier as of 2026-06-24 (GT-feedback control tuning).** All tuning this session was on GT feedback (PLASMC_GT_FEEDBACK=1) to isolate CONTROL from perception. Best config = baseline (h_rd=-0.42, VDF funnel/gains Γ_z=0.75, K_R rp=2.5): **IC2/IC3/IC5 sub-meter (3/5), IC1+IC4 fail terminally.** The IC1/IC4 failure is the binding issue.

**THE BINDING MECHANISM — terminal descent-control failure (full chain, traced):**
1. The loom reference is loom-proportional: `h_d_z = h_rd` ⇒ `vz = h_rd·Z`, which **tapers vz→0 as Z→0**. So near the deck the measured loom `h_z=vz/Z` **COLLAPSES toward 0**.
2. The collapse grows the loom error `h_e_z = h_z − h_rd` → σ_z grows → **κ_z PUNCHES to its cap (3.0)** → the switching term `−θ·sat(σ_z/E_z)·G·κ_z` over-drives (G also ∝1/Z near deck) → **OVER-BRAKES: vz flips POSITIVE (balloon).**
3. The balloon (vz↑ near the deck) perturbs the otherwise-CONVERGED lateral; `s_e_n=lat/Z` then **1/Z-amplifies into a lateral RUNAWAY** (a_u explodes 10²–10⁴, marker leaves FoV) → off-target / fly-away.
KEY: the drone CONVERGES laterally fine (lat→0.08–0.11 by Z≈1m); it's the **terminal vertical transient** (collapse→over-brake→balloon) that throws it. Common to ALL h_rd — the trigger just differs: gentle h_rd → descent STALL→collapse; aggressive h_rd → OVERSHOOT→over-brake→balloon.

**h_rd IS NOT THE LEVER (full analysis, both runs compared):** feasible **h_rd=-0.42 TRACKS the loom cleanly** (h_e_z≈+0.01 from Z=2 down to Z=0.7, Γ_z=0.75). Aggressive **h_rd=-1.0 is INFEASIBLE at altitude** (demands vz=-1.0·Z=-6 m/s @6m → loom can't track, overshoots to -2.0, then over-brakes). Both still balloon at the terminal (κ_z punch). So the descent loop is sound mid-descent; the failure is purely terminal (the κ_z over-brake), independent of h_rd.

**DEAD-ENDS this session (do NOT re-try):**
- **Aggressive constant h_rd=-1.0:** infeasible at altitude → weak-capped descent (a_u_z stuck ~1.4, barrier-transform `a_u=−G⁻¹·a_v` absorbs the surface, κ_z slow to punch). STOCHASTIC: descend-but-off-target (4–8m) OR hover-deadlock. 0/5 sub. Bundles 185049 (deadlock), 191928 (off-target).
- **Funnel tightening (P2INF_z=0.5, XI2_z=0.8):** feeds the **Singhal containment** GENUINE (non-glitch) large loom errors. Containment clamps h_e_z to the funnel boundary and fabricates a "descending" loom → fools the controller into hover → DEADLOCK (h_rd=-1.0, where hover h_e_z=+1.0 > floor 0.5) or STRUGGLE/off-target (h_rd=-0.42, 1/5 sub, bundle 201120). **Constraint: funnel floor must exceed |h_rd|** (hover h_e_z=|h_rd|). **Containment is HARMFUL in GT-FB** (no glitches to reject — every large loom error is genuine). Don't tighten the loom funnel; keep VDF P2INF_z=1.5.
- **Slow Γ_z=0.3:** weakens descent authority → deadlock-prone. Keep VDF Γ_z=0.75.
- **Two-phase / proximity-gated h_rd:** user rejected (wants constant h_rd).

**ALREADY-FIXED / not the issue:**
- **Mid-descent lateral+yaw limit cycle:** FIXED by **K_R rp 1.5→2.5** (eliminated, 10–100× amplitude cut; the coupled yaw cycle collapsed with it). The yaw cycle is **PX4 rate-loop-lag-bound (287ms)**, not yaw-SMC-tunable (E_a↓ and K_R_YAW↑ both ruled out; Ω_a=0.1 baked removed the double-integrator). K_R rp=2.5 is baked in controller.py (uncommitted).
- **Perception sign-flip:** DOWNSTREAM of the fly-away (vanishes on clean trajectories, 0% wrong-sign). Near-deck loom degradation = single ~1m marker OVERFLOWS the FoV at Z≈0.4m (corners off-frame → decode dies → ring under-reports) → loom UNOBSERVABLE <0.4m; affects only the failing reps. FLOW_LOOM_SIGN_GUARD added default-on (clamps consumed loom ≤0) but moot for the control issue. V-frame-ring-only baked (real-image ring removed). All uncommitted.

**κ_z CAP IS NOT THE LEVER (RESOLVED 2026-06-24, no-κ-cap test bundle 203757).** Uncapping κ_z (PLASMC_KAPPA_MAX_Z=1e6, baseline h_rd=-0.42): IC1 **0.06 (RECOVERED** from terminal-fail — the 3.0 cap WAS over-braking it), IC2 0.41, IC5 0.80 (3 clean sub, κ_z stayed 0–10) BUT IC3 **κ_z RAN AWAY to 287** (59.6s hover-struggle, metric-glitch 0.00) and IC4 off-target 11.7m (κ_z→88). So uncapping is NOT a clean fix: it recovers reps where the cap over-braked (IC1) but lets κ_z run away on the reps where the loom collapses (IC3/IC4) → struggle/off. Net ≈3/5, just SHIFTED (IC1↔IC3). **CONFIRMS the loom collapse is the ROOT** — it drives κ_z to misbehave either way (capped→over-brake@3.0; uncapped→runaway 88–287). The cap value only modulates the symptom.
**THE REAL LEVER (next): fix the terminal LOOM COLLAPSE directly, NOT the κ_z cap.** Untried: a reference governor that doesn't taper vz→0 near the deck (the loom-proportional vz=h_rd·Z is the collapse source); or terminal loom/brake-authority handling that prevents the collapse→κ_z-misbehavior. Keep the default κ_z cap 3.0 (PLASMC_KAPPA_MAX_Z=0.03 is GT-FB-only per [[feedback_gtfb_kappa_z_bounce]], regresses production).

**ENV-NAME TRAPS (cost hours this session):**
- Per-axis params use the **PLASMC_ prefix via pa()**: it's `PLASMC_KAPPA_MAX_Z` (NOT KAPPA_MAX_Z — the latter is silently ignored; the κ_z cap ran at default 3.0 ALL session), `PLASMC_GAMMA_Z`, `PLASMC_P2INF_Z`, `PLASMC_XI2_Z`. (PLASMC_GT_FEEDBACK has the prefix built in → it worked.)
- Setting `PLASMC_GAMMA_Z` (or XI2_Z) ALONE bypasses the VDF auto-align `if not any(PLASMC_GAMMA_* in env)` (controller.py ~247) → X/Y revert to the HOT pa() defaults (2.0). **Must pin all three** (X/Y at VDF 0.4375/0.5). P2INF/E use per-axis checks (no bypass). VDF gains: GAMMA(0.4375,0.5,0.75), KAPPA0(0.125,0.125,0.25), E(1.0,1.0,0.5), P2INF(0.5,0.5,1.5), XI2 0.2 all, KAPPA_MAX_Z 3.0.

**INFRA:** gate = `scripts/run_ic_validation.sh` (IC1-5 added: IC1 0,0,5 / IC2 2,2,5 / IC3 -2,2,5 / IC4 2,2,7 / IC5 2,2,3). ~50% SITL flake. The setsid'd children SURVIVE a harness "task failed" → gate keeps running detached → leftover SITL → must kill the `run_ic_validation` root + `run_aruco_landing*` + `parameter_bridge`/`ros_gz_bridge` explicitly (pkill -f misses some; use exact PIDs). Use run_in_background (harness-tracked), NEVER shell `&`.

**Analysis gotchas:** align controller `t` (absolute perf_counter) with GT via `gt["Start Time"]+gt["Time"]` (absolute) — do NOT zero `t` then interp against absolute GT (clamps to constant). GT loom = clean_vz (uniform resample+savgol(7,2)+gradient) / max(Z,0.1). In GT-FB, Control_Data `h(t)`=GT loom (consumed), Img_Data = image perception (logged parallel). flight_s>35 + Zmin>1 = hover-stall; xy=0.000 = metric glitch (check Control_Data length).
