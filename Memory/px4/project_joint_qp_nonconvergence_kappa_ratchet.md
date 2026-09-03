---
name: project-joint-qp-nonconvergence-kappa-ratchet
description: First concrete evidence the joint-QP (cbf_visibility.py) fails to converge near touchdown — theta_cone chatters frame-to-frame under a tight FoV margin, ratcheting kappa into a_u=1274 m/s^2. GT-feedback, so not perception noise; not yaw-related.
metadata:
  type: project
---

**Found 2026-09-04 while diagnosing an `a_u=1274` blow-up** in a Q8 yaw-feedforward probe rep
(`test_data/Q8_SpinFF_slow/off`). Confirmed **unrelated to yaw or the feedforward work** —
`e_a`/`yaw_c`/`w_u[2]` stay small and calm through the entire window. Confirmed **unrelated to
perception noise** — the rep ran under `PLASMC_GT_FEEDBACK=1` (exact `s`/`h`).

## The chain, evidenced from Control_Data

1. **Near touchdown, `MARKER_EXTENT_PX` saturates at 318** (frame-filling) starting ~t=7.8s,
   alt dropping through ~0.9→0.2m. The FoV box margin is tight.
2. **`theta_cone` starts CHATTERING frame-to-frame**, not smoothly tightening:
   `0.16 → 0.20 → 0.10 → 0.71 → 0.10 → 0.77 → 1.02` (t=8.17→8.34, ~20ms steps). That step-to-step
   sign/magnitude instability is the signature of a solver NOT converging, not of a genuinely
   changing constraint.
3. **`I_a_xy` swings wildly** in sync — smooth deceleration until t≈8.13s, then oscillates
   positive/negative every frame (`(+0.64,+2.33)→(+0.77,+0.99)→(-0.51,-0.21)→(-2.04,+3.11)…`).
4. **`kappa[1]` ratchets monotonically** every cycle instead of settling:
   `0.09→0.10→0.11→0.13→0.16→0.20→0.30→0.59` (t=8.06→8.38) — the documented kappa-ratchet
   mechanism ([[project_20260828_ic_motion_batch_gt_touchdown_and_kappa_ratchet]]), because the
   adaptive law never sees a settled `sigma` to relax against.
5. Once `kappa` is large enough, `a_u` (∝ `kappa·sigma`) detonates: peak **1273.9 m/s²** at
   t=11.48s. The vehicle then genuinely flies laterally away (GT relative x: −0.18→+2.78m over
   1.8s, confirmed against raw `Ground_Truth.npy` poses, not just the internal `s` signal).

## Why this matters beyond this one rep

This is the first CONCRETE evidence for the gap flagged (not fixed) in the 2026-09-03
`CBF_visibility.tex` correction (`fc44c953`, §4): *"the solver runs a fixed iterate budget with no
convergence test and no residual logged, so convergence is currently unobserved."* — the fixed
6-outer × 5-inner loop, `cbf_visibility.py:322 for _outer in range(6):`. It was flagged as
theoretically unaddressed; this rep shows it is **actually failing to converge**, with a plausible
mechanism connecting non-convergence → chatter → kappa ratchet → terminal blow-up.

**Not yet confirmed as THE general cause of terminal blow-ups** — this is one rep. What would
confirm it: logging a per-outer-iterate residual (e.g. `||Ia_lat_new - Ia_lat_prev||`) or a
convergence flag, then checking whether it correlates with terminal blow-up reps across the
existing corpus (`SPCampaign`, `Rover_Turning`, the various `KappaRatchet_*` bundles already in
test_data). That is the natural next step, not yet done.

## Scope note

Independent of [[project_thrust_sphere_bug_and_bake]] (the `A_CAP` deliverability bug, already
fixed) — this is a DIFFERENT failure inside the same joint-QP block, on the FoV-box side rather
than the sphere side. Both live in `cbf_visibility.py`'s `CBF_JOINT_QP` path.
