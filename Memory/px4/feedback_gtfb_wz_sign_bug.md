---
name: feedback_gtfb_wz_sign_bug
description: "⭐⭐ ROOT CAUSE of the GT-FB lateral divergence: gt_feedback.py fed w_z = +alpha_dot but the true rotational optic flow is w_z = -alpha_dot (= -psi_dot_b). Validated -0.91 vs IMU. The wrong sign flipped h_d's cross(w,s) rotation FF + the old c-term's omega_dot x s / 2w x h -> spurious anti-restoring feedforward -> drone flies OUT at altitude. Fix (w_z=-alpha_dot) makes IC4 converge monotonically (2.5->0.7->0.2->0 vs diverging 2.5->6.4)."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**THE GT-FB w_z SIGN BUG = root cause of the lateral divergence (2026-06-25, user-led).** User flagged
`alpha_dot = -w_z`. The rotational optic flow `w` (manuscript eq 82: w = ^V w_t - psi_dot_b*e3 = -psi_dot_b*e3
for a stationary target) has `w_z = -psi_dot_b = -alpha_dot` (alpha = relative yaw uav-target = +psi_dot_b
stationary). But `gt_feedback.py:140` set `w[2] = +_slope(ts, ry_arr) = +d(alpha)/dt = +alpha_dot` — the
OPPOSITE sign.

**VALIDATED vs IMU (Img_Data, parallel-logged perception+IMU):** perception lstsq `w_z` correlates **-0.91**
with the body yaw rate (ratio -0.69) -> perception `w_z = -psi_dot_b` (CORRECT, matches manuscript). So the
PERCEPTION path was right; only the GT-FB SUBSTITUTE had the flipped sign. The wrong w_z corrupted BOTH:
(a) `h_d`'s rotation FF `+cross(w,s)` (the desired flow), and (b) the OLD c-term's w-cross-products
(`omega_dot x s`, `2 w x h`; `w x(w x s) ∝ w_z^2` is sign-invariant) -> a spurious ANTI-RESTORING feedforward.

**MECHANISM (GT, metric lateral offset = drone_xy - target_xy, no 1/Z):** with the WRONG w_z, the drone
flies OUT at altitude — IC4 (2,2,7) offset 2.5 -> **6.4 -> 5.4** m at alt 6->4 (commanded acceleration was
even weakly inward in world-frame, but the spurious FF in h_d/c-term drove the lateral velocity OUT). With
the CORRECTED w_z (`w[2] = -_slope`), IC4 converges MONOTONE: 2.5 -> **0.7 -> 0.2 -> 0** -> lands 0.21m
(worst baseline 8m fly FIXED). n=2 (8 reps, flakes): 6 sub/1 marg/1 fly vs baseline 12/15 sub 2 fly — the
fly tally understates it (the IC4-type altitude divergence is GONE; the IC4 8m fly is now a clean land).

**Why this was invisible for the whole session:** every fly-away "fix" was a CAP (kappa_max, CTERM_DWS_MAX,
COMMIT_AU_MAX, breach-recovery) bounding the OUTPUT of a sign-flipped feedforward. The image-frame a_u
"anti-restoring" reads were dismissed as frame artifacts; the WORLD-frame command actually was ~inward, so
the bug was upstream in h_d/c-term, not the command direction. The two long-standing failure modes split
exactly here: **mode 1 (IC4 fail-to-converge / fly OUT at altitude) = FIXED by the w_z sign; mode 2
(terminal deck LAUNCH, c-term omega_dot x s yaw-accel x centroid -> thrust saturation, [[feedback_terminal_launch_flow_loop]])
= STILL present** (IC1_rep2 9.2m), now cleanly isolated.

**How to apply.** FIX is in `gt_feedback.py` (`w[2] = -_slope(ts, ry_arr)`), GT-FB ONLY — perception was
already correct (lstsq), so production/perception-on is UNAFFECTED. Confirm n=3 then bake the GT-FB fix.
The CLEAN c-term (CH_CLEAN) is a SEPARATE path that AVOIDS w (uses psi_dot_b) — it regressed (its own
psi_dot_b sign, PLASMC_CH_PSIDOT_SIGN knob added) and is moot now that the OLD c-term works with correct
omega. REMAINING: the terminal deck launch (mode 2) — the now-isolated next target. Continues
[[project_gt_feedback_control_tuning]], [[feedback_terminal_root_lateral_zeta_r]]. Corrects the
"c-term is mis-derived / use clean form" framing — the OLD c-term is FINE; the omega INPUT was sign-flipped.
