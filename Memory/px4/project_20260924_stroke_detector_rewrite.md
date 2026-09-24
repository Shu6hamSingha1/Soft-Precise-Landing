---
name: project_20260924_stroke_detector_rewrite
description: "2026-09-24 ⭐ STATIONARY SOLVED (sperc+stroke 25/25 manuscript SP, xy = GT p=0.5); moving Sinusoidal 0.17 m vs gt 0.04 (rate/latency-bound). IN PROGRESS — user-chosen FULL locked-design detector rewrite. src/cross_stroke_detector.py (CROSS_DETECTOR=stroke, default legacy): multi-scale ridge strokes + X-junction confirm. Kills the rover corner-lock (poison 25-29%->0%) and lighting poison (dim 21%->0%). Open: terminal <0.3 m refusals, col, inv (GT pairing broken), speed, downstream integration. ALSO: deck-world GT target is the ROVER not the marker (~0.14 m offset)."
metadata:
  node_type: memory
  type: project
  originSessionId: d1fc52b7-905a-496e-9fb1-d06cdf4109aa
  modified: 2026-09-24T06:29:05.045Z
---

Context: [[project_20260924_terminal_perception_rate_drop]] (why: sperc stationary 6/25, rover 0/5; corner-lock + frozen-s runaway) and [[feedback_cross_detector_robustness_requirement]] (the locked design + validation rules: accuracy/poison + flight outcome, never detect-rate alone; eval set first).

**Built (all uncommitted as of 2026-09-24):**
- `src/cross_stroke_detector.py` — ridge = σ²(|λ1|−|λ2|) − K·σ|∇I| (K=1 cancels a step edge's lobe exactly), scales 0.8–40 px on a pyramid (per-level max, one upsample/level); NMS centrelines; orientation-consistent Hough → seed band 6 px → TLS refit + re-collect at max(2, 0.25σ) (collecting straight off the 1° Hough peak split diagonals into one-sided segments); dedupe; pair = crossing lines with BILATERAL support (a side too short to verify is None, not balanced — frame-corner border lines passed otherwise); BORDER_REPLICATE + 3 px margin (BORDER_REFLECT turned an edge-touching step into a fake ridge; the interim 2σ exclusion blanked σ30-40 — superseded); channel cascade L→a→b; tracked ROI + scale window. Dispatch `cross_marker_detector.DETECTOR` / `CROSS_DETECTOR=stroke`.
- `tools/validate_detector_gt.py`: TRUE-bearing reference (V_s_true) default, terminal bands 0.3-0.7/0-0.3, POISON rate, per-tag meta.json (marker_dz), exact frames.tsv pairing. Variants `current`, `stroke`.
- `src/cross_marker_perception.py`: IMG_RECORD writes `frames.tsv` (frame -> capture stamp).
- `tools/build_evalset_tag.py`: pairs rep<->raw by CONTIGUOUS stamp run (set membership is NOT safe: unrelated runs of one campaign share 59-70% of stamps — sim clock restarts at 0; true pairing 0.76-1.00 vs <=0.068 contiguous).
- `scripts/record_perception_evalset.sh` -> `test_data/PerceptionEvalSet/<tag>/`; tags so far rover_sin_r1-5, rover_circ_r1-5, flat_IC1 (+ queued flat_IC3/IC4, clutter, rover_static_IC2/4, rob_* re-records).

**Measured (stroke vs current, true-bearing ref):** RobustnessFrameset base/bright/darkbg/dim/lowsun: poison 0% (current 0.3-21%), detOK 93-100%, err med 0.006-0.011. rover_sin: poison 25-29% -> 0-0.3%, within-0.15 69-73% -> 100%. Terminal <0.3 m: stroke REFUSES often (rover_sin 30-57%, dim 0%) — refusals not poison; next to fix. col 16% detOK. inv unscoreable: RobustnessFrameset/inv tail-offset pairing is WRONG (f150 ~2 m vs paired GT alt 4.9 m) — every inv number ever reported is invalid; rob_inv re-record (with frames.tsv) queued.

**⭐ FULL EVAL (2026-09-24, test_data/PerceptionEvalSet, 18 valid tags excl. rover_circ):** stroke POISON 0-0.6% (current 0.4-36.6%), within-0.15 99-100% (63-100%), err med 0.004-0.009 (0.006-0.125), detOK 93-99% (88-100%). rob_inv (correctly paired): current poison 36.6% err 0.125 -> stroke 0.4% / 0.006. rob_col via CHANNEL CASCADE (L then Lab a,b; X-test arbitrates) 96% detOK 0% poison. Full CrossMarkerPerception with stroke: s poison ~0 (legacy 30-34% on rover/inv), ~23-33 ms/frame (tracked ROI + scale window; untracked ~30 ms). Fixes that mattered: BORDER_REPLICATE (not REFLECT + 2σ exclusion, which blanked σ30-40); σ up to 40; re-collect inliers on the REFINED line. Remaining: <0.3 m detOK 33-88% (err 0.03-0.04; legacy 94-100% but err 0.05-0.32) — refusals from the terminal junction BLOB (arms merge -> one line one-sided); vmin 4w->2w did nothing. First SITL flight test (sperc+stroke, stationary IC1-5 + Sinusoidal) launched 12:18.

**⭐⭐ SITL FLIGHT RESULT (2026-09-24, detector md5 c259cc70, CROSS_DETECTOR=stroke, GT-FB except s):**
- STATIONARY IC1-5×5 (`test_data/SPercGTFB_AB_stroke/`): **25/25 manuscript SP**, xy mean 0.017 (gt 0.015, MWU p=0.5), vel mean 0.049 max 0.169 (gt 0.016, p=0.003 — higher but inside 0.2). Legacy sperc was 6/25. ⇒ perception s CAN replace GT s for a stationary target (1 pre-ring-version rep set aside in `_prering/`, replaced).
- MOVING Sinusoidal×5 (`test_data/SPercGTFB_rover_stroke/`): xy med 0.168 (legacy 2.52, gthold 0.058, gt 0.039), vel med 0.77. No runaways. Mechanism: stroke is SLOW live — frame gap 48-80 ms (legacy 16-31); detection err at capture only 0.012-0.033 but GT s drifts 0.021-0.035 per gap (0.085 <0.3 m) on the moving target -> held-s error 0.04-0.06 in flight. For MOVING targets rate/latency DO matter (unlike stationary). Next: speed (tracked 19 ms offline -> ~3x live) + KF-rate latency compensation of s.

**⚠ DECK-WORLD GT IS THE ROVER, NOT THE MARKER.** rover_cross_deck: `deck_follower.py` teleports `deck_platform` to the rover pose at 20 Hz; GT/GT-FB use POSE_IDX_TARGET=1 = rover. Both detectors (independent) see the marker a median 0.141 m (IQR 0.09-0.16) off the GT point at 0.39 m/s (≈0.36 s lag); rigid rover_cross Sinusoidal: 0.013 m. Direction vs velocity NOT yet checked (lag vs fixed offset unconfirmed). Implication: GT-FB Linear/Circular steer at the rover; gt_Circular 0/5 relaxed, xy 0.21 m. rover_circ_* tags unfit for accuracy scoring until the deck pose is recorded as target. Flagged to user, not fixed.

**Moving `gt` baseline (IC2, n=5):** even full GT-FB gets 0/5 manuscript SP (Sinusoidal relaxed 4/5, Circular 0/5) — judge moving-target perception against the gt arm, not manuscript thresholds.
