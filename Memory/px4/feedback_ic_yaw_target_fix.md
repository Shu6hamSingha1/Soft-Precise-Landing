---
name: feedback-ic-yaw-target-fix
description: IC3-5 failures root cause (NOT initial yaw); IC_YAW_TARGET=alpha HURT IC4 (90-deg alias); real fix pending. NC153-160.
metadata:
  type: feedback
  originSessionId: e265057f-9381-4119-bd0b-0fb5e3c7498f
---

IC3-5 consistently fail when `INITIAL_YAW_DEG=0` (drone faces North/East) because the marker is at 90-180° yaw error from the starting orientation, causing anti-restoring lateral correction from the first descent frame.

**Why:** run_ic_validation.sh sets no `INITIAL_YAW_DEG`, defaulting to 0.0 (facing North in NED). With `IC_YAW_TARGET=gt` (default), the yaw servo drives GT ENU yaw → 0 (East in Gazebo). For IC3 (ENU -2,2,5), the marker bearing is 135° NED = marker is SE of the drone. Starting at North (0°), the yaw error is 135° → large initial alpha error → opposite sign from IC2 → anti-restoring lateral commands in the first frames → fly-away before yaw can correct.

**IC3 failure evidence (all gates combined, June 2026):**
- June 13 gate: 0/5 (xy 1.1-2.1m, non-fly-away failures — old config)
- June 14 gate: 0/3 fly-aways (9.9m, 23.7m, 3.6m)
- June 23 gate: 1/3 (0.707m success), 2 fly-aways
- June 24 gate v1: 0/3 fly-aways (4.5m, 6.4m, **60m**) — NC154
- Total: 1/14 = 7% success rate

**IC4 (ENU 2,2,7) failure pattern:** same xy-offset as IC2 but higher altitude → more EKF compass drift accumulation during longer hover → worse yaw at descent start. All gates: 0-1/3 per gate.

**IC5 (ENU 2,2,3) failure pattern:** soft landings (rel_vel ≈ 0.05 m/s) but xy ≈ 2.9m TL. The 3m altitude is insufficient for lateral convergence from 2.83m offset. Same initial yaw problem + geometry.

**IC2 improvement note (NC153):** with combined barrier, IC2 is now 2/3 sub-meter (0.52m, 0.61m) — best ever. Previous gates: 0-1/5 (June 13-14), 1/3 (June 23).

**Gate v2 results (IC_YAW_TARGET=alpha, 20260624-135230) — REVERTED:**
- IC2: 2/3 sub-meter (NEUTRAL — same as gate v1, alpha fix neither helped nor hurt)
- IC3: 0/3 fly-aways at 6-13m (NEUTRAL — alpha fix didn't help IC3)
- IC4: 0/3 fly-aways at 18-71m (**REGRESSION** — gate v1 was 1.2-2.6m; alpha HURT IC4 catastrophically)
- IC5: 0/3 at ~2.8m (NEUTRAL — alpha fix didn't help IC5)

**IC3 REAL root cause — TERMINAL LOOM SIGN-FLIP from marker overflowing FoV (traced from Control_Data + Img_Data, IC3_rep1 bundle 20260624-132414):**

⛔ TWO earlier claims this session were WRONG and are RETRACTED:
- "IC3 descends 70% faster than IC2" — FALSE. Altitude-binned GT velocity: descent rates are nearly identical (IC2 1.13 vs IC3 1.18 m/s over Z>1m; 4m→1m in 2.05s vs 2.22s — IC3 marginally SLOWER). The "70%" was a time-alignment artifact (compared at mismatched t/Z). Both null lateral to ~0.15-0.25m by Z=2-3m.
- "moment loom under-reports by a D-dependent geometric factor" — FABRICATED formula, no derivation. (Moment loom geometry M∝1/Z² is right so ≈vz/Z mid-flight, but the windowed-derivative estimator under-reads when loom changes fast AND glitches at the deck.)

✅ ACTUAL mechanism (both ICs descend the same; IC3 dies in the LAST ~0.4m):
1. At **Z≈0.42m** the single ~1m ArUco marker nearly fills + OVERFLOWS the 640×480 FoV (corners project to ±321px vs ±240px half-height; corner v reaches 517>480). Marker centroid is still near-centered (|s|=0.19) — it is NOT drifting to the edge, it is TOO BIG/TOO CLOSE. ArUco decode FAILS (N flow corners 89→0 at the loss frame; stays lost 40/40 subsequent frames). This is the documented single-marker limit ([[feedback_single_marker_rank_deficiency]]: no single marker spans 10m→0.4m, ~4.5 octaves).
2. With aruco gone, the raw corner-moment loom = 0 (no corners). But the controller consumes the corner+ring **fusion EKF** (FLOW_FUSE_RING=1 default) → it falls back to the **ring loom** (N ring corners 25-75 still present). Near the deck the ring moment is noisy and swings POSITIVE (+0.26); the EKF tracks it (post-loss corr(fused,ring)=+0.48) and its velocity state amplifies → fused loom flips **−0.31 → +0.44 → +0.99 → +1.83**.
3. A positive loom = "marker shrinking/receding" — physically impossible during descent. The Z-SMC reads it as the drone climbing away, commands **upward thrust** (I_a_z −10.6→−6.2, thrust ~3.6) → balloon. Simultaneously the centroid glitches (lat 0.26→0.48, s_e_n breach 0.89) → marker further out of frame → fly-away. IC2 (success) never lost the marker (h_z stayed monotonic −0.6→−1.4) and landed at 0.52m.

So IC3's fly-away is **a PERCEPTION failure (terminal decode loss + wrong-signed ring-fallback loom), NOT a control gain / anti-restoring / fast-descent issue.** This is exactly the interference GT-feedback removes (GT loom=vz/Z is always valid).

**Natural perception fix lever (for AFTER control tuning, NOT yet implemented):** a descending drone can never have a positive loom — clamp h_z ≤ 0, or HOLD last-good loom / gate the ring loom when aruco corner count = 0 (the "LOOM_CLAMP" idea). Or root availability: nested/inner-cluster marker so a smaller marker stays decodable when the big one overflows ([[project_landing_target_design]]).

**IC4 alpha regression cause (gate v2, separate issue):** At 7m the marker subtends smaller angle → 4-fold ArUco symmetry more degenerate → alpha=0 has multiple near-equal minima → servo converges to WRONG 90° alias → fly-away. DO NOT use IC_YAW_TARGET=alpha at ≥7m.

**GT-FB gate RESULTS (bundle 20260624-150335, NC161-164) — THE KEY FINDING:** Running the controller on CLEAN GT loom/feedback (PLASMC_GT_FEEDBACK=1 + KAPPA_MAX_Z=0.03, TL=0 everywhere since GT never loses the marker): IC2 **3/3 sub-meter** (0.61/0.76/0.43), IC5 **2/3** (0.91/0.94/1.43) — but **IC3 and IC4 STILL DIVERGE on perfect feedback**: IC3 0.53/3.06/**47.8m**, IC4 1.5/3.1/**11.6m**. So the perception sign-flip is REAL but **NOT the sole IC3/IC4 cause** — there is a genuine off-center **CONTROL-LAW instability** that GT-FB exposed. The perception sign guard is NECESSARY but NOT SUFFICIENT; IC3/IC4 need control tuning. (IC2/IC5 were perception-limited → fixed by clean loom; IC3/IC4 are control-limited.)

**EKF assessment (GT-grounded, image EKF loom vs GT loom on bundle 150335):** (1) wrong-sign loom frames are **86/93 (IC3) / 63/63 (IC4) at aruco=0** = the ring-takeover sign-flip (with aruco present it almost never flips, 0-9 frames) → addressed by the SIGN GUARD. (2) Separately the EKF UNDER-reports loom magnitude (~0.5×, ratio 0.05-0.82) and is positive-biased (+0.3 to +1.9) + anti-correlated (corr −0.1 to −0.66) even with aruco present = documented terminal sensor SATURATION — but the GT-FB gate PROVES this is NOT the IC3/IC4 blocker (control diverges on perfect loom) → **no EKF rework beyond the sign guard is warranted** (chasing the under-report won't move the gate).

**SIGN GUARD IMPLEMENTED (FLOW_LOOM_SIGN_GUARD, default-on, code change UNCOMMITTED):** clamps the consumed ego-loom h_z ≤ 0 at the EKF state (img_data.py `_ekf_fuse_step`, prevents windup) + the non-fuse output paths (`getOptFlowAngVel`). A landing's ego-loom is always ≤0 (vz<0/Z>0), so clamping ≤0 prevents a wrong-signed ring loom from commanding ascent; catches the actively-updated wrong sign the existing anti-stale loom-decay MISSES (decay only handles a FROZEN loom). IMPORTANT (corrects an earlier wrong proposal): the EKF does NOT gate the ring on aruco count — it deliberately LEANS on the ring through aruco dropout (kills the corner loom when n_corn≤3 so the ring carries the vertical; ring admitted on its OWN corner count ≥6). So the right fix is a sign-CLAMP, not a ring-gate.

**CONTROL-SIDE ROOT CAUSE of IC3/IC4 divergence (fork analysis on the GT-FB runs, 2026-06-24):** NOT a lateral-control failure, NOT worse-off-center convergence — all 12 reps converge laterally first (minLAT 0.01-0.17m at ~3m, INCLUDING the fly-aways). The failure is a **TERMINAL VERTICAL BALLOON from tilt-thrust coupling**, discriminated by **terminal descent-COMMIT speed, not lateral error**:
- LAND (IC2, IC3_r3): carry HIGH descent velocity into the terminal (vz≈-1.6 m/s @0.22m, κ_z→3.0, I_a[2]→-44) → punch to the deck BEFORE lateral ramps. (They reach s_e_n≈1.45 ≫ 0.648 and STILL land → the 0.648 anti-restoring line is NOT the cliff, REFUTED.)
- FLY (IC3_r1 48m, IC4_r1 12m): descend SLOWLY (vz≈-0.45 @0.29m, κ_z only 0.23) → descent STALLS @~0.27m → as it hovers, lateral a_u ramps 7→27 (1/Z; chi_r·zeta_r dominates sigma, zeta_r pinned at its 5.18 cap) → demands ~68° tilt → inner-loop lag (e_R=0.8rad/46°) → tilt STEALS vertical thrust → vz→+0.03 while I_a[2] still commands -14.9 down (ascent is NOT vertical-channel-commanded — it's the tilt) → upward impulse → balloon → loom genuinely positive → runaway.
Hypotheses: chi_r over-drive SUPPORTED; |s_e_n|>0.648 REFUTED as discriminator; e_R-lag present but DOWNSTREAM (68° tilt demand is unphysical, stiffer kR/kΩ can't help); lateral→vertical coupling SUPPORTED (=the balloon); yaw ±90° alias REFUTED (|e_a|<0.3 all reps).

**CONTROL LEVERS (prioritized; validate n≥5 by fly-away RATE — commit-vs-balloon is STOCHASTIC: IC2 3/3, IC3 1/3, IC4 0/3):** (1) **Cap terminal lateral command / commanded tilt** (clamp |a_u_lat| or PLASMC_THETA_CAP once close, gate on marker extent, scale-free) — lateral is already ~0.25m so capping costs nothing and kills the thrust-stealing tilt. Highest value. (2) **Terminal commit** (freeze lateral, commit descent when centered+close) — make IC3/IC4 commit like IC2; COMMIT_EXTENT machinery exists (default-off), needs the centered-terminal gate. (3) Reduce chi_r 0.5→~0.35 / soften p_r near terminal — cuts the a_u ramp; secondary, risks slowing convergence.

**⚠ METHODOLOGY FLAG:** the GT-FB gate ran at PLASMC_KAPPA_MAX_Z=**3.0 (default)** — `KAPPA_MAX_Z=0.03` was IGNORED (per-axis params use the PLASMC_ prefix via pa(): the var is **PLASMC_KAPPA_MAX_Z**). FORTUNATE: κ_z→3.0 is LOAD-BEARING for IC2's commit (free-fall punch needs I_a[2]→-44); capping at 0.03 would likely REGRESS IC2. So the default-cap result is the right baseline — do NOT re-run with 0.03.

**PERCEPTION DEFERRED** (user 2026-06-24): sign guard / EKF outlier gate parked until new test results logging BOTH GT and image values. (Sign-guard code in, default-on, uncommitted; outlier gate designed not implemented; the V-frame-ring-only change STANDS — geometric correctness fix.) Do NOT add IC_YAW_TARGET=alpha (reverted).
