---
name: project_tuning_campaign_newcal_reset
description: Control-tuning campaign restarts under the 8-run honest cal (2026-06-06); 3 cal regimes; perception-freeze gate; prioritized 6-knob sweep order; generic run_knob_sweep.sh harness
metadata: 
  node_type: memory
  type: project
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

The PLASMC control-parameter tuning campaign **restarts from zero** under the honest sensor cal (8-run 2026-06-06 `d60973a`+`34917ab`, REFRESHED to all-13 on 2026-06-07). Nothing has been tuned under this cal yet.

**THREE cal regimes (not two) — this dates every historical conclusion:**
1. **May-12 broken cal** (2–13× under-report; controller at 0.08–0.53× design gains). Era of: Big Sensitivity Sweep, strict coord descent, precision-softness frontier, Phases 1–5, [[feedback-precision-tuning-lessons]]. All gain conclusions here are GAIN-STARVED artifacts ([[feedback-historical-cal-confound]]).
2. **Multisine 4-run cal** (~06-01→06-05): parameter_record **trials 1–59** ran here. Better scale BUT still carried the V-frame g-sign bug ([[feedback-getvirtualpts-g-sign]]) + yaw-RHS bug → flow scale still off. Includes the "28% SP / trial 46" result and the IC2-5 funnel collapse.
3. **8-run honest cal (2026-06-06)** — V-frame leveling fixed, yaw-only RHS, ring+corner transfer. **Untuned.**

Regime-independent facts SURVIVE: lag timing (pitch 38ms / roll-pitch 52–61ms / yaw 287ms), MATLAB delay-robustness, IC-start compass drift (fix the rig not a gain — [[feedback-yaw-compass-drift-ic-start]]), moment-alpha canonical, MC_RATE_P-dead, marker-FoV-match constraint, KLT bridging.

**PERCEPTION NOW FROZEN (2026-06-07) — baseline-tune against THIS.** The perception layer is committed: (a) sensor cal = all-13 corner M + re-keyed transfer ring; (b) centroid `IMG_FEATURE_FILTER='kf'` (q/r decouple `5ae0f29`, `IMG_FEAT_KF_R`=0.004); (c) **`FLOW_FUSE_RING=1` DEFAULT ON** — the controller's flow is now the corner+ring EKF fused `[h_tr;w]` via `getOptFlowAngVel`, with the reliability-gated ring handoff. **CRITICAL CAVEAT — `FLOW_FUSE_RING=1` is UNVERIFIED closed-loop:** the only fused landing hovered (1-rep, unattributed; fused h_z==corner h_z so probably not EKF-specific, but unconfirmed — [[ekf-default-breaks-descent]]). So: establish the IC1 n≥5 baseline AT these defaults FIRST; **if landings regress vs the corner-only history, set `FLOW_FUSE_RING=0` to isolate the EKF flip from your gains BEFORE chasing gain changes.** Two un-run gates the campaign now owns: the EKF-default descent verify + the centroid-KF IC2-5 (cal IC2-5 was waived by the user). Then `analyze_saturation_audit.py` → pick first knob from evidence.

**CURRENT BAKED CODE DEFAULTS (as of 2026-06-08, commits f812895/a62c81e/917cf57/21d27b0/2e125a9):**
- `KP=12`, `KI=1`, `P=5/5/5`, `E=1.5/1.5/1.0`, `N_Z=0.02`, `GAMMA_Y=1.0`
- `SEN_FUNNEL=1` (was silently OFF before — all pre-2026-06-08 PX4_NewCal_Record runs used env var)
- `FUNNEL_MODE=cbf2`, `THETA_FLOOR=60`
- `IC_YAW_TOL=5°` (widened from 2° to accept EKF drift), `IC_YAW_SERVO_DMAX=0.3`
- `LANDING_AUTOSAVE=1` in `run_aruco_landing.sh`

**CAMPAIGN STATE AS OF 2026-06-09 — 42 rows in PX4_NewCal_Record:**

**Best confirmed stack:** `KP=9, E=[2.5,2.5,0.5], P=[5,5,5], KI=1.0` → median **3.80m**, min 1.95m (n=5). Performance ceiling under gain tuning.

**KP×E coupling (see [[feedback_kp_e_coupling]]):** KP=9 with E_XY=2.5; KP=12 only with E_XY≤1.5.

**ALL PARAMETERS SWEPT and confirmed optimal at defaults:** Gamma_Z=0.75, KR_YAW=2.0, P2INF_Z=1.5, TERMINAL_HOLD_EXTENT=70, YAW_OMEGA=0.5, PSID_RATE=1.0, tau_ua=0.1, E_Z=0.5 (marginal improvement over 1.0).

**Dead-ends confirmed:** KI≤0.35, E_Z≥1.5, KP=12+E=2.5, N_Z=0.05 alone, tau_ua=0.3, P_XY=3, KP≥13.

**SITL RELIABILITY:** `renice 15 $(pgrep -x code)` BEFORE launching. Always use `run_aruco_landing_retry.sh`. taskset -c 6-15 on outer shell.

**BINDING FAILURE (2026-06-09):** Stochastic LK/ArUco collapses — 1-2 TL/5 reps even at perfect IC. LK dynamic range ceiling ~2 m/s. Not gain-tunable. Gap between xy_min (~2m) and xy_med (~4-6m) is entirely stochastic perception.

**ALL GAIN-SIDE LEVERS EXHAUSTED (2026-06-09, row 43):**
- `N_XY=0.05`: 5.87m vs 3.80m baseline — DEAD-END. Faster κ adaptation without σ signal (E=2.5 keeps σ<E) = noisy.
- `KP_Y < KP_X` is the only untried gain-side lever but low priority (asymmetry is secondary vs perception fix).

**NEXT PRIORITY: code-level levers (perception fix):**
1. **Pyramidal LK levels 2→3** in `img_data.py` — directly extends dynamic range ceiling (~2 m/s → ~4 m/s). This is the binding failure fix. High impact.
2. Anti-windup on outer PID: freeze `is_e_n` when `h_e/p > 0.7` — prevents h_d integral windup when inner loop near-saturated.

**Preliminary failure-mode read** (existing pre-final-perception reps under ~correct cal, 0 SP): dominant near-miss = terminal touchdown spike + funnel breach (sigma/ℰ ~21–29% + zeta on the impact axis, ds_d→25–32) — matches [[feedback-dsd-touchdown-spike]]; plus IC-edge TARGET_LOST at t≈0 (marker near FoV edge); plus κ-runaway hover-timeout. Cone-clamp deadlock isolated to THETA_FLOOR=0.

**Harness:** `scripts/run_knob_sweep.sh` — generic single-knob IC1 sweeper (`SWEEP_KNOB`/`SWEEP_VALS`/`EXTRA_ENV`/`IC`/`N_REPS`), per-value sub-bundles + aggregate. IC2-5 gate is `bash scripts/run_ic_validation.sh` — **never pass it `LANDING_OUT_BASE`** (it self-bundles by copying the latest default Landing_Test dir; a custom base breaks its before/latest detection → all ICs logged "NO land").
