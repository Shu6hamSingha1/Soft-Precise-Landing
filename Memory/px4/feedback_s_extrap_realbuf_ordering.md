---
name: feedback_s_extrap_realbuf_ordering
description: "s/_img_feature_param self-reference extrapolation FIXED (2026-07-09, mirrors h-extrap) - but the first fix version captured the real-buffer BEFORE the ds outlier-hold guard, poisoning the fit with an un-rejected spike -> terminal kick; ordering rule = capture real-sample buffers AFTER all outlier guards."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 32b3f473-5219-48ce-b070-323ef9cc309a
---

**Fix 1 (the planned one):** `_img_feature_param` (centroid s) had the SAME self-reference extrapolation bug h-extrap had (deg-1 fit fed its own extrapolated output back → cascading outliers / clip-pinning; flagged as "next target" in [[feedback_h_extrapolation_baked]]). Fixed with the identical pattern: separate REAL-only buffers `_img_feature_param_real/_real_t` (never receive extrapolated output), decay-to-zero over `_h_extrap_decay_frames` consecutive misses, clipped-sample rejection from the fit basis, alpha (s[3]) still held+wrapped not extrapolated.

**Fix 2 (the bug I introduced doing Fix 1, then root-caused from a failed landing):** the first version appended to the real buffer at the DETECTION site — BEFORE the `ds_max=0.15` outlier-hold guard runs. A single-frame genuine-detection spike (fparam_x=−0.44 sandwiched between ~0 values, n_corners=157 so "real") was correctly rejected by the guard in the value the controller consumes, but the PRE-guard value still entered the extrapolation's "real-only" history. 12 frames later the marker was lost → the poisoned deg-1 fit jumped s from ~0.12 to 1.03 in one step → s_e_n/zeta_r funnel breach → terminal kick (traced end-to-end, commit 9346aac).

**Why / How to apply — the general rule:** any "real-sample-only" history buffer feeding a model/fit must be populated from the POST-guard corrected value, downstream of every outlier-rejection stage — never at the raw detection site. "Real" (freshly detected) ≠ "clean." When adding such buffers, grep for every append site and check what guards run between detection and consumption. Validation: post-fix rerun of the same scenario had zero |Δs|>0.3 jumps and landed precise+soft.
