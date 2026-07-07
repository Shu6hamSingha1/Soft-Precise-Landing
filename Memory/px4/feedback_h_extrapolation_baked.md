---
name: feedback_h_extrapolation_baked
description: "h (optical flow) extrapolation BAKED default-ON 2026-07-07 (PLASMC_H_EXTRAP=1), reversing the 2026-05-13 hard-zero policy. The old failure was a fixable implementation bug (self-reference), not a reason to avoid extrapolating h."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

**`PLASMC_H_EXTRAP` baked default-ON (img_data.py ~720)**, replacing the 2026-05-13 hard-zero-on-miss
policy for `h` (optical flow / velocity). Validated A/B (IC2 x5 baseline vs x3 valid h-extrap reps,
observer+DESCENT_GATE): baseline hard-zero had 1/5 bad misses (GT xy 2.703m); h-extrap had 0/3 misses
(GT xy 0.075-0.097m all), zero fly-aways either leg.

**Why the 2026-05-13 hard-zero decision was NOT wrong, but was based on a fixable bug.** The old
deg-1 polynomial extrapolation cascaded (10^5+ outliers or pinned at the clip ceiling forever)
because of THREE specific, fixable implementation flaws, not because extrapolating h is inherently
unsafe:
1. **Self-referential feedback**: the OLD code (and CURRENT `s`/`_img_feature_param` extrapolation,
   STILL BROKEN, see below) appended its own extrapolated GUESS back into the same history buffer the
   next extrapolation fits against -> compounding cascade.
2. **No decay**: once wrong+clipped, the value propagated forever with no path back to a safe default.
3. **Fit through clipped samples**: a saturated sample's true value is censored; fitting a trend
   through it as exact data biases/amplifies the slope.

**The new implementation (img_data.py, `_h_real_t`/`_h_real_v`, ~line 720-745) fixes all three:**
(1) separate REAL-ONLY history buffer, never fed extrapolated output; (2) `_decay = max(0, 1 -
consec_misses/H_EXTRAP_DECAY_FRAMES)` (default 10 frames) converges to the same safe zero hard-zero
gave; (3) samples within `H_EXTRAP_CLIP_BOUND` (1e-3) of the ±10 lstsq clamp are excluded from the fit.
Bounded at `H_EXTRAP_MAX` (default 5.0).

**⚠ OPEN, NOT YET FIXED: `s` (position/centroid) extrapolation has the EXACT SAME self-reinforcement
bug** (`_img_feature_param.append(extrapolated_img_feature_param)`, img_data.py ~1942) — proven live
(pab2 rep2 trace: s_y drifted linearly for 2.4s with h=0, GT lateral grew 0.35m->3.16m in lockstep).
This is the next fix target, NOT yet implemented. [[feedback_savgol_predict_suspect_flyaway]]
describes a DIFFERENT, separate mechanism that can ALSO inject bad h downstream of this fix.

**How to apply:** when investigating a fly-away/miss with h-extrap active, the flow-freeze/snap
mechanism is FIXED — look elsewhere first (savgol-predict spike-reconstruction, s-extrapolation,
near-contact re-lock gain). Do NOT re-litigate "should h extrapolate" — that question is settled;
the remaining open bugs are in OTHER code paths that touch h/s downstream.
