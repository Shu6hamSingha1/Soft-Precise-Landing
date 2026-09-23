---
name: project_20260924_rover_platform_corner_detection
description: "rover_cross start-of-descent misses traced to the dark gate admitting the platform's shaded side face; detector returns the plate corner; confirm stages reject it correctly but can't recover the true cross"
metadata:
  node_type: memory
  type: project
  originSessionId: b50a458b-5a2b-4939-9b33-eb63d1375bc0
  modified: 2026-09-23T20:03:22.653Z
---

2026-09-23/24, offline replay of `test_data/RoverIC_raw/rover/IC3_rep{1,2,3}` (static rover_cross, GT-FB, IMG_RECORD=1 raw frames; recorder `test_data/RoverIC_raw/record_rover_ic_raw.sh`).

**Mechanism (verified by drawing the fitted points):**
- `inRange(V<=100)` admits the platform's SHADED SIDE FACE (V≈90) as a thick bar plus a thin top-edge shadow. The `rover_cross` platform-colour fix (model.sdf, 2026-09-01, diffuse 0.6) does not cover the shaded face.
- At native res the detector pairs side face (~550-730 pts) + top edge (~60 pts) → returns the PLATE CORNER (~px 109,171) instead of the cross centre (~53,207). About 10% of frames at 4-6 m. ROI_FRAC_X 0.65 vs 1.0 makes no difference; only a 2× downscale changes it (the thin arms/edges vanish).
- The tracked crop flips between native and 2×-downscaled depending on crop size (`DETECT_WORK_MAX_PX=200`), so the output alternates corner/true every frame.
- The live `CROSS_S_JUMP_GATE` then keeps the corner (it seeds `_s_recent`) and rejects the TRUE centre as a jump. Live log IC3_rep2: `s`≈(0.08,0.08) vs GT (0.31,0.42) for 0.7 s. The live "miss" frames have Fail Reason `None` = detector OK, rejected post-detection.

**Confirm stages (ring+balance, default OFF):** they rejected 24/30 corner frames ≥1 m. The apparent "15 true centres lost" was a PAIRING ARTEFACT — the variants' tracker states diverge, and every balance rejection is a genuine corner (balance d≈0.6 = centre outside the arm span). They can only reject; they can't recover the true cross because the side face outranks the 1-px arms in `_best_pair`.

**Proposed (not yet done, awaiting user go-ahead):** next-best-pair retry after a confirm rejection, behind an env flag; offline-score it on IC3 + RobustnessFrameset + GtfbMulti_col (make sure `inv` corners stay rejected).

**Caveats:** the below-1 m "corner" rates are confounded by an altitude-dependent reference offset (`ey ≈ 0.12 − 0.26/(z+0.2)`, cause unknown). IC5 (cropped marker at start) is still unrecorded — SITL was held by other sessions.

Related: [[feedback_cross_detector_contrast_not_darkness]], [[feedback_cross_detector_robustness_requirement]], [[project_20260901_rover_cross_perception_diagnosis]]
