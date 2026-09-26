---
name: project_20260924_rover_platform_corner_detection
description: "rover_cross start-of-descent misses traced to the dark gate admitting the platform's shaded side face; detector returns the plate corner; confirm stages reject it correctly but can't recover the true cross"
metadata:
  node_type: memory
  type: project
  originSessionId: b50a458b-5a2b-4939-9b33-eb63d1375bc0
  modified: 2026-09-23T20:03:22.653Z
---

> ⛔ **SUPERSEDED same day by the STROKE detector** (`19cdabff`, 2026-09-24 13:00, `CROSS_DETECTOR=stroke`, locked design). Rescored with the updated validate_detector_gt.py (true-bearing ref) on IC3 rover: POISON legacy 4.7/10.6/9.5%, legacy+ring+balance+component-retry 4.3/5.9/6.6%, **stroke 4.0/2.3/1.6%** (detOK 98-99%, misses only in 0.3-0.7 m, `stroke_not_x_junction`). Component retry NOT to be ported — it patches the legacy dark-gate pipeline. The mechanism notes below stay valid for LEGACY. Item-2 near-deck offset was largely the old reference (median err 0.076→0.057 under the true-bearing ref).

2026-09-23/24, offline replay of `test_data/RoverIC_raw/rover/IC3_rep{1,2,3}` (static rover_cross, GT-FB, IMG_RECORD=1 raw frames; recorder `test_data/RoverIC_raw/record_rover_ic_raw.sh`).

**Mechanism (verified by drawing the fitted points):**
- `inRange(V<=100)` admits the platform's SHADED SIDE FACE (V≈90) as a thick bar plus a thin top-edge shadow. The `rover_cross` platform-colour fix (model.sdf, 2026-09-01, diffuse 0.6) does not cover the shaded face.
- At native res the detector pairs side face (~550-730 pts) + top edge (~60 pts) → returns the PLATE CORNER (~px 109,171) instead of the cross centre (~53,207). About 10% of frames at 4-6 m. ROI_FRAC_X 0.65 vs 1.0 makes no difference; only a 2× downscale changes it (the thin arms/edges vanish).
- The tracked crop flips between native and 2×-downscaled depending on crop size (`DETECT_WORK_MAX_PX=200`), so the output alternates corner/true every frame.
- The live `CROSS_S_JUMP_GATE` then keeps the corner (it seeds `_s_recent`) and rejects the TRUE centre as a jump. Live log IC3_rep2: `s`≈(0.08,0.08) vs GT (0.31,0.42) for 0.7 s. The live "miss" frames have Fail Reason `None` = detector OK, rejected post-detection.

**Confirm stages (ring+balance, default OFF):** they rejected 24/30 corner frames ≥1 m. The apparent "15 true centres lost" was a PAIRING ARTEFACT — the variants' tracker states diverge, and every balance rejection is a genuine corner (balance d≈0.6 = centre outside the arm span). They can only reject; they can't recover the true cross because the side face outranks the 1-px arms in `_best_pair`.

**Next-best PAIR retry: FALSIFIED before implementation** — on corner frames only 2 angle clusters exist (both plate edges). The cross is lost earlier: `_isolate_marker_by_shape` keeps the LARGEST cross-plausible component = the plate-edge "Γ" (935 px, fill ~0.15), dropping the real cross (261 px, fill ~0.09).

**Component-level retry — PROTOTYPED OFFLINE (scratch monkeypatch, NOT in src/):** on a confirm-stage rejection, rerun `_detect_core` on the next-largest cross-plausible component (≤2 retries). Prototype = /tmp/claude-1001/-home-shubham-Soft-Precise-Landing/b50a458b-5a2b-4939-9b33-eb63d1375bc0/scratchpad/comp_retry2.py (scratchpad, ephemeral — re-derive from this description).
- Unguarded: rover IC3 detOK 100% on all 3 reps (ring+balance alone 92-99.7%), corners stay removed; inert on base/bright/lowsun/darkbg; `inv` unchanged (no corners let back in). ONE REGRESSION: `dim` at 1 m — the dark gate admits plate texture speckle, a 303 px texture blob passes the shape test (fill 0.235 < 0.25) AND ring+balance, and the tracker locks onto it for 3 frames. ⇒ the confirm stages reject corners but are NOT a guard against small blobs.
- **Guard: area floor 0.1** (retry candidate ≥ 0.1 × first-ranked component area; rover true cross 0.28, dim blob 0.03) fixes dim exactly (back to ring+balance values) with no rover cost (blocked 11 retries, lost 1 of 43 rescues). The no-tracker-lock-on-rescue guard is weaker (1 bad frame left).
- Status: offline only. Not ported to src/ (another session had uncommitted edits in cross_marker_detector.py on 2026-09-24). Needs: port behind `CROSS_COMPONENT_RETRY` (default off), then a perception-mode SITL A/B on rover_cross.

**Caveats:** the below-1 m "corner" rates are confounded by an altitude-dependent reference offset (`ey ≈ 0.12 − 0.26/(z+0.2)`, cause unknown). IC5 (cropped marker at start) is still unrecorded — SITL was held by other sessions.

Related: [[feedback_cross_detector_contrast_not_darkness]], [[feedback_cross_detector_robustness_requirement]], [[project_20260901_rover_cross_perception_diagnosis]]
