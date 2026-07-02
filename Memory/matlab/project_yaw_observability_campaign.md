---
name: project_yaw_observability_campaign
description: "Yaw campaign 2026-06-27..07-02: 2pi weighted-corner orientation ported PX4->MATLAB (removes the ±90° second-moment fold; gate 25/25), every yaw gain lever mapped/exhausted, yaw-rate ceiling ~0.5 rad/s is ARCHITECTURAL (s_e orbits in yaw frame -> cone-sat + funnel breach), heading-yaw on Sin/Liss explored and REVERTED"
metadata:
  type: project
---

**2pi orientation (KEPT, uncommitted):** `image_feature.m` s_alpha was `1/2*atan2(2mu11, mu20-mu02)`
— second moments are even-order => invariant under 180° => orientation observable only mod 180°,
so `yaw_asmc` folded e_a to (−90°,90°] and fast swings PINNED e_a at the 90° ceiling (no gain can
act on error the sensor can't represent). Ported PX4 `img_data._marker_principal_angle`: corner
weights [4,3,2,1] -> weighted-centroid displacement (1st moment, rotates 1:1) disambiguates the
pi-axis to full 2pi. Verified clean 360° sweep (15° steps -> 15° feature steps, no fold).
`yaw_asmc.m` unwrap ×2…/2 REMOVED (full ±180° e_a) — the unwrap was REQUIRED for the old mod-180°
alpha (prevents chasing phantom >90° errors); only removable after the 2pi estimator. alpha_d
auto-derives via image_feature(V_nP_d) — Static terminal e_a = −0.0° confirms no branch/bias.
Gate with baked gains: 25/25 realistic + 25/25 noiseless. Backups:
`Obsolete/image_feature_pre_weighted2pi.m`, `Obsolete/yaw_asmc_pre_unwrapdrop.m`.

**Yaw gain map (all levers tested, Sinusoidal heading-yaw + 2x-Circular):** e_a stays 88–90°
across EVERY yaw param pre-2pi. Neutral: E_a↑, n_a↑, Gamma_a↓(0.15), kOmega_yaw↑. Harmful (lateral
pump): Gamma_a↑, Omega_a↑, kappa_a0↑, p_a↓, kR_yaw↑(1.0–2.5 breaks Sin), kOmega_yaw↓(reopens the
limit cycle — pinned at 0.2 from both sides). Yaw-rate FF `Omega_d=[0;0;u_a]` in so3_tracker:
helps constant yaw (Circ e_a 53°) but Sin 5/5->3/5 — u_a is the LAGGED output, feeding it back is
circular; REVERTED (`Obsolete/so3_tracker_pre_yawff.m`). Omega_d=0 rationale: lean-FF would
differentiate the noisy CBF lean; yaw-FF alone is clean but only valid for slow references.
NO limit cycle in yaw (sat(sigma_a/E_a) frac 0.00; body-rate flips 4–6 = forced tracking).
Post-2pi terminal lag: integral Omega_a=0.4 cuts Circular termEa 17°->9° (constant rate) but
WORSENS oscillating Sin (41°->60°, windup) — do not bake.

**Yaw-rate ceiling (Circular, CIRC_YAWMULT hook removed after test):** 1x(0.48) 5/5 ->
2x(0.96) 1/5 -> 3x 0/5. Failure is LATERAL not yaw (termEa only 14–18° at 2x): s_e expressed in
the drone-yaw-aligned V frame ORBITS at the yaw rate (s_e angle sweeps at ~1.08 rad/s = wz);
transport term correctly follows (measured psi_dot_b) but radial convergence must come on top of
orbit-tracking. Caps that bind: CBF tilt-cone sat frac 0.66/0.71/0.86 at 1x/2x/3x + position
funnel breach engR 1.16 at 2x (vs 0.59 at 1x). chi_r 2->4 no help; kR_yaw=1.0 marginal 2/5.
=> ceiling is ARCHITECTURAL; levers = widen accel budget or yaw-fixed lateral frame (research,
out of manuscript scope — manuscript range 0.29–0.67 rad/s sits inside).

**Heading-yaw on Sin/Liss (REVERTED — traj_Gen restored to original):** tangent heading
atan2(v_y,v_x): Sin works 5/5 realistic + speed with raw heading + gentle Gamma_a=0.15 post-2pi
(its v_y=v0>0 bounds the swing 51–129°); noiseless 4/5. Lissajous INTRACTABLE: velocity ~vanishes
at reversals (A=0.4: max|psi_dot|=32 rad/s, min|v|=0.018) -> ~180° snaps; rate-limited stateful
heading (w_max 0.05–0.4) does NOT fix (Sin 4/5 regressed, Liss 1/5); larger A softens
(worstXY 39->5) but 0/5 at all A — the PATH demands untrackable turns; leave Liss non-rotating
(physically honest: no vehicle turns instantaneously). Circular remains the yaw-exercising case
(kappa_a adapts to sigma_a~0.95 -> equilibrium |sigma_a|/p_a~0.5; Sin kappa_a leak-decays to 0).
`sigma_a`/`e_a` logging added to yaw_asmc/run_simulation (kept).
