---
name: project_20260824_ic5_perception_fov_margin_gap
description: "Traced WHY IC5's off-center wall reproduces under real perception but NOT under GT-feedback (both cross-marker AND ArUco -- not marker-type-specific). Root mechanism is NOT subtle s_e_n noise breaking a control-law invariant: real detection at IC5 (3m altitude, 2m/2m offset) repeatedly hits 'KLT corners left image bounds -- stopping fallback' -- the marker's corners GENUINELY leave the camera frame, a real geometric FoV-margin problem GT-feedback structurally cannot experience (it has no imaging chain at all). This is why dtheta/CBF fires 65-98% of frames at IC5 (project_20260824_dtheta_az_filter_self_defeating_feedback) -- it's genuinely fighting a tight margin, and still loses sometimes. TARGET_LOST fires at ~3-3.6s, well before the funnel-shape convergence law (already fixed/baked, confirmed clean under GT-FB by t~5-9s) gets the time it needs."
metadata: 
  node_type: memory
  type: project
  modified: 2026-08-24T19:00:56.881Z
  originSessionId: 0f9c8dc0-a837-4722-95e7-0ea102167469
---

Same-day follow-up to [[project_20260824_crossmarker_offcenter_convergence_wall]]'s major correction (the wall is not the old kappa-leakage architecture gap -- that's fixed and confirmed clean under GT-feedback at IC5). This entry identifies what the real, still-open perception-mode gap actually is.

## Test design

Isolated the `dtheta` az-correction as a confound by setting `PLASMC_DTHETA_AZ_GAIN=0` (fully disabled), then compared IC5 (2,2,3 ENU) across:
- 2x perception-mode cross-marker reps (`test_data/PerceptionVsGT_IC5/20260824-220500/cross_perception_dtheta0_rep{1,2}`)
- 1x perception-mode ArUco rep (`.../aruco_perception_dtheta0`) -- **the key control**, to check if this is cross-marker-flicker-specific or general
- vs the existing GT-feedback cross-marker baseline (`test_data/KappaWall_IC5/20260824-210000/gtfb_baseline`, `PLASMC_GT_FEEDBACK=1`)

## Result: reproduces on ArUco too -- NOT cross-marker-specific

All 3 perception-mode runs (both marker types) failed `TARGET_LOST` within 3-3.6s: cross rep1 `xy_err=2.429`, cross rep2 `xy_err=7.946`, ArUco `xy_err=1.554`. The GT-feedback run (same IC, same baked config) ran the full ~8.87s and landed precise+soft (`xy_err=0.066`). This rules out cross-marker's known detection-flicker issue ([[feedback_cross_marker_detection_flicker]]) as the (sole) explanation -- ArUco, which doesn't have that flicker problem, fails the same way.

## Root mechanism: a real, physical FoV-margin problem, not noise-on-s_e_n

Checked the ArUco perception run's log directly: **`ArUco lost — KLT fallback active` (8x) followed by repeated `KLT corners left image bounds — stopping fallback`.** This is not a soft detection-confidence issue the KLT fallback could bridge -- the corners are GENUINELY exiting the camera frame. `MARKER_EXTENT_PX` sits flat at 96.9px for ~1.8s (t=0.84→2.61s, a held/stale value consistent with repeated decode failure) before jumping to 156-167px as the drone continues closing distance.

**GT-feedback structurally cannot experience this at all** -- it reads relative pose directly, with zero dependency on the imaging chain, so it never risks a corner leaving frame. That's WHY the same baked control law (funnel-shape fix, `PR0=10`/`XIR=0.10`) works flawlessly under GT-FB but fails under real perception at this specific IC: it's not that the control law behaves differently, it's that perception's OWN precondition (marker stays decodable) breaks down before the control law gets the ~5-9s it needs to pull `s_e_n` down (confirmed from the GT-FB trace: `s_e_n` is still 0.6-0.8 at t=2-3s even in the CLEAN run -- comparable to what the perception runs show right before losing lock; it just isn't given the rest of that time under perception).

**This connects directly to the `dtheta`/CBF investigation from earlier the same day** ([[project_20260824_dtheta_az_filter_self_defeating_feedback]]): `dtheta` fires 65-98% of frames at IC5 specifically because the CBF is genuinely fighting a tight FoV margin at this IC (close range + large offset = larger `delta_k` margin consumption, per `cbf_visibility.py`'s `h_k = phi_max_k - |cr_k| - delta_k`). The CBF's job is exactly to prevent corners leaving frame -- and even with it actively constraining lateral authority almost continuously, real-world noise/dynamics are still enough to push the corners out sometimes. The CBF is not failing to do its job; the margin it has to work with at this IC is just very thin.

## Conclusion

IC5's real-perception failure is a genuine **geometric FoV-margin / detection-reliability problem specific to real imaging**, not a control-law defect (that's fixed and GT-FB-verified) and not a cross-marker-specific perception bug (ArUco shows an equivalent failure). This is the strongest evidence yet on the "is IC5 too extreme" question from earlier the same session ([[project_20260824_dtheta_ic5_flyaway_rootcause]]'s discussion) -- at 3m altitude with a 2m/2m offset, the marker sits close enough to the FoV boundary that ordinary flight noise is sometimes enough to physically exit frame, independent of any control-law bug. This does NOT necessarily mean IC5 should be relaxed (per the earlier "fix causes not limits" discussion) -- but it does mean the fix, if pursued, is in the PERCEPTION/CBF-margin domain (e.g. tighter reserve margin, better corner-recovery/derotation-under-motion, or an even more conservative CBF bound), not a controller-gain or funnel-shape change (those are already solved for the case the imaging chain can actually deliver).

## Follow-up (2026-08-25): IC2/IC3 checked -- same mechanism, different TIMING, not a different bug

Ran the identical ArUco perception-mode probe (`PLASMC_DTHETA_AZ_GAIN=0`) at IC2 (2,2,5 ENU) and IC3 (-2,2,5 ENU), same session (`test_data/PerceptionVsGT_IC5/20260824-220500/aruco_perception_dtheta0_IC{2,3}`).

**The exact same log signature occurs at both** -- `ArUco lost` + `KLT corners left image bounds — stopping fallback` (IC2: 2 occurrences; IC3: 52, noisier, consistent with IC3's known detection-flicker exposure). But the OUTCOME is benign at both: IC2 `xy_err=0.300m` (`TARGET_LOST`, but `min_alt≈-0.01m` -- already on the ground), IC3 `xy_err=0.427m` (`FAIL`, `min_alt≈0.02m`). Both flights ran the FULL 7.0-7.5s duration (Control_Data), essentially reaching the target before any corner-exit event mattered.

**Root distinction from IC5: WHEN the corner-exit happens, not WHETHER it happens.** At IC2/IC3 (5m start altitude), the log lines showing corner-exit cluster in the LATTER portion of a 7-7.5s flight -- i.e. near touchdown, where the marker naturally grows to fill/exceed frame as an expected part of terminal approach (already anticipated by this project's terminal-perception-loss handling). At IC5 (3m start altitude, SAME 2m/2m lateral offset), the identical event happens almost immediately (~3s into flight, altitude still 2-3m) -- catastrophic because the vehicle hasn't converged yet, vs. benign at IC2/IC3 because it's already essentially on target by the time it happens.

**Conclusion: this is NOT an IC5-exclusive bug -- it's the same universal geometric mechanism (thin FoV margin under a 2m/2m offset), just triggered far earlier in the trajectory because IC5's lower starting altitude leaves much less runway before the margin gets tight.** IC5 isn't exposed to a different failure mode than IC2/IC3; it's exposed to the SAME one at a much less forgiving point in the descent.

## Status / not done

n=1-2 per config -- directionally solid (mechanism trace: log messages + extent trace are direct, unambiguous evidence, not a statistical inference) but not a validated sweep. Not yet checked: whether tightening the CBF's own margin (`delta_k`) further, or a corner-recovery mechanism specifically for near-boundary transient exits, would close this gap -- would help IC5 (where the exit currently occurs too early to be benign) without needing to touch IC2/IC3 (where the existing terminal handling already covers it fine).
