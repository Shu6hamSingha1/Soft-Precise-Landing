---
name: project_moment_loom
description: "MATLAB descent failures root-caused to the pinv(L_s) loom extraction (vz/z = sigma_min~2 weak singular mode; tol=4 truncates->sign-flip, low tol->limit cycle). Fix = scale-free moment area-rate loom (-1/2 d(lnM)/dt) -> 92->95/100, better-calibrated. Lateral stays pinv (replacements regress)."
metadata: 
  node_type: memory
  type: project
  originSessionId: a655e27a-1e09-403a-bdce-9a60aec3aca1
---

**Descent (loom) root cause + fix (2026-06-19).** The MATLAB single-run recovers the optical flow `V_h = [vx/z; vy/z; vz/z]` via `V_v_i = pinv(L_s, 4)*dPdt` (ACTUAL=1). The loom `V_h(3)=vz/z` is the **flow divergence** and sits on the **weakest singular mode of L_s** (vz column = `[-x;-y]` -> sigma_min ~ 2.09, in (0.01,4] for 93% of steps; sigma_max~600, cond~290).

**Why it failed:** `pinv(L_s, TOL)` 2nd arg = SVD truncation tolerance.
- tol=4 (default) TRUNCATES the loom mode -> `V_h(3)` under-reports then **sign-flips** near ground (marker fills/overflows FoV, featRad>1.2) -> Static IC5 seed4 descent runaway (thrust collapse 20->3, 30deg tilt robs vertical thrust). NOT 1/z noise; it's SVD truncation.
- tol<=1 RETAINS the loom but **un-regularizes** -> divides finite-diff/quantization error in dPdt by the tiny SV -> amplified oscillatory loom -> descent **limit cycle** (Lis-IC4 noiseless: vz pp & freq DOUBLE) -> suite 44/100 (15/25 even noiseless).
- Regularization-vs-resolution (truncated-SVD) tradeoff; no single tol wins. Smoothing dPdt (SG-derivative) to cut noise = fatal LAG (0/25).

**Fix = SCALE-FREE moment area-rate loom** (gated `USE_MOMENT_LOOM`, default off): `V_h(3) = -1/2 d(ln M)/dt`, M = 2nd-moment corner spread `sum((V_nP_i-centroid)^2)`. Uses only the FRACTIONAL rate `Mdot/M` (M ~ (size/Z)^2 -> `d(lnM)/dt = -2 Zdot/Z`) -> **independent of marker size, units, depth**; fixed factor 1/2, NO scale calibration. **Rotation-immune**: only divergence changes area (curl/shear area-preserving; foreshortening 2nd-order near fronto-parallel) -> no gyro, robust to target rotation.
- VERIFIED vs analytical truth: slope 0.90-0.99 CONSISTENT across cells (=scale-free), corr 0.86-0.98. Better-calibrated than pinv loom esp NOISY (corr 0.918 vs 0.865, RMS 0.049 vs 0.062).
- CLOSED-LOOP (keep-s_ddot 92-config, chi_r=0.65 kR=[4;4;0.5] p2inf_z=1.0): noiseless 25/25 (no regression) -> **NOISY 92->95/100**, breach 1.91->1.49.
- Lag-comp gain (MOMENT_LOOM_GAIN to push slope->1) HURTS: 1.0->95, 1.1->91, 1.2->88, 1.3->84. The 0.82 under-read is benign (gains tuned around filter attenuation). KEEP gain=1.0.

**Do NOT replace the pinv LATERAL.** In C_SIMPLE+combined-barrier ONLY V_h uses pinv (V_w/V_dw dead; angular = gyro psi_dot_b). Lateral replacements REGRESS: centroid (1st-moment) lateral POORLY calibrated (slope 0.71 corr 0.72 noisy) -> 86/100; reduced-pinv lateral (loom col removed) WELL-calibrated (slope 1.02 = pinv) but closed-loop 90 + doesn't fix pathological seeds. The lateral is the pinv GOOD part (well-conditioned sigma~600, 4-corner-fused, savgol-filtered). alpha-rate de-rotation: zero effect. **Architecture: moment loom + pinv lateral (hybrid) = 95/100.**

**Pathological seeds (Static IC5 s4, Liss IC3 s4) fail under EVERY estimator** (pinv, moment, reduced) -> raw terminal-noise / FoV-overflow corruption + pinv CROSS-CONTAMINATION (joint 6-DOF lsq spreads loom-noise into lateral: over-reads S5, under-reads L3). A perception FRONT-END limit, not an estimator-choice one.

Theory for full pinv removal (image moments, Chaumette 2004/Tahri 2005 in refs): div(area)->loom, curl(alpha)->yaw, deformation(2nd-moment shear)->tilt, centroid->lateral; gyro can't de-rotate a rotating target but area divergence is rotation-invariant. All test hooks gated default-off; baked default unchanged. See [[project_sddot_limit_cycle]], [[project_combined_barrier_matlab]].
