---
name: feedback_moment_loom_cal_bypass_invariant
description: "INVARIANT: with FLOW_LOOM_DECOUPLE=1 (baked default-on), the moment loom is ALREADY the calibrated scale-free vz/Z, so _sensor_cal_hw row 2 must be BYPASSED at EVERY site that emits h_z. An asymmetry (observer branch applied cal row 2, LK branch bypassed it) shipped undetected because each site reads correct in isolation; fixed 00ba40d 2026-07-17."
metadata:
  node_type: memory
  type: feedback
---

**INVARIANT — the decoupled moment loom must bypass `_sensor_cal_hw` row 2 at EVERY emitting site.**

With `FLOW_LOOM_DECOUPLE=1` (BAKED default-on since 2026-06-23), `h_z` is the moment loom
`−½·d(lnM)/dt` (M = μ20+μ02 of the de-rotated corner scatter), which is **already the calibrated,
scale-free vz/Z**. `_sensor_cal_hw` row 2 is the *raw-pinv→cal* map: applying it to a moment loom
over-scales ~7% AND re-injects the lateral/rotational cross-coupling the decoupling exists to remove.

**Why this class of bug hides:** every producer of `h_z` reads *locally correct*. The cal multiply
`_sensor_cal_hw @ V` is the obviously-right line at each site; the bypass is the surprising one. So a
NEW loom producer gets written with the plain multiply and nothing flags it — the loom silently changes
scale + coupling depending on WHICH path produced it that frame. Found 2026-07-17 (fixed **00ba40d**):
the observer branch (LK failed, marker decoded, `img_data.py` ~:2579) applied cal row 2 while the
LK-succeeded branch (~:2450) bypassed it — same physical quantity (both assemble row 2 as
`_hz = _loom_dec`, ~:2044), two treatments, for ~1 day. It reached CONTROL (that branch feeds the
fusion EKF the controller consumes by default) on every LK-dropout-at-altitude.

**How to apply:** when adding/editing ANY path that emits `h_z`, `grep -n "_corner_cal\[2\]\|_out\[2\] = "
src/img_data.py` and confirm the new site matches. Known correct sites (all bypass under
`_loom_decouple`): LK-succeeded ~:2457, observer ~:2592, getter KF :3769, getter Savgol :3765.
The coast path is safe *because* it appends raw and is calibrated at the getter — don't "optimize"
it to pre-calibrate. Ring flow uses its own `_sensor_cal_ring` (inert: `FLOW_FUSE_RING=0` since
2026-07-09). Cal-derivation tools are diagonal-only and never re-fit row 2's cross-terms
`[+0.0535, −0.0044]` (an older board-fit) — the bypass is what makes that harmless.

**Validation status: NOT flight-validated** (code-reading fix, correct by construction vs the
LK-branch's own documented reasoning). Expected effect = ~7% loom scale correction on LK-dropout
frames ONLY. Judge it on `h_z` continuity ACROSS LK-dropout frames — an SP-count A/B at n=5 is
underpowered vs the ±5–7 SP noise floor ([[project_why_sp_achieved]]).

Relates [[feedback_klt_fallback_merge_no_separate_cal]], [[reference_gt_optical_flow]].
