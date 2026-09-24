---
name: project_20260924_terminal_perception_rate_drop
description: "2026-09-24 — sperc (perception s, rest GT) 6/25 manuscript SP vs GT 25/25. CAUSE = close-range s NOISE below ~0.5 m, NOT staleness: GT s held at perception's exact update timing (PLASMC_GT_S_HOLD=stamp) lands 25/25 like GT. The 49->12 Hz terminal rate drop (detect() cost scales with arm pixels) is real but harmless."
metadata:
  node_type: memory
  type: project
  originSessionId: d1fc52b7-905a-496e-9fb1-d06cdf4109aa
  modified: 2026-09-23T21:14:21.804Z
---

**Test (2026-09-24).** GT-FB with only `s` from perception (`GT_ABLATE=h,hz,yaw,wz`) vs full GT-FB, stationary cross-marker, IC1-5 × n=5 (`scripts/run_sperc_gtfb_ab.sh`, data `test_data/SPercGTFB_AB/`). Both 25/25 at the relaxed 0.15 m/0.5 m/s; at the manuscript 0.08 m/0.2 m/s: GT **25/25**, sperc **6/25**. xy_err 0.015 → 0.059 m (MWU p=4.5e-7), touchdown speed 0.016 → 0.257 m/s.

**s accuracy by altitude** (`tools/analyze_s_perc_vs_gt.py`): RMSE ~0.017 normalized down to 0.5 m, no meaningful bias; 0.036 at 0.3–0.5 m; 0.057/0.078 below 0.3 m. So perception s is NOT the problem at altitude — the terminal band is.

**Why s thins out:** decode does NOT fail (352/31k misses, not altitude-concentrated). The processed-frame gap goes 16 → 48 → 68 → 96 ms (every 1st → 3rd → 6th camera frame at 62.5 Hz) as MARKER_EXTENT_PX saturates ~300 px (0.5-1 m band onward). Identical in the `gt` arm → a pipeline-throughput property, not caused by feeding s back.

**Stage timing** (`tools/time_perception_stages.py`, offline replay, identity quat, overlay inpainted): process_frame 5.6 ms (extent <100 px) → 28 ms (>290 px); `detect()` is ~75% of it. Inside detect at >290 px: ~5.6k line points (vs 242 far); `_detect_core` ~24 ms (mostly `cv2.fitLine` in `_robust_fit_line` over thousands of pts, ×3 iters, ~8 calls), plus `_scale_detection` + `_shift_detection` ~10 ms EACH — pure-Python `tuple(map(tuple, ...))` conversion of the point lists. The 2026-08-28 working-res cap (`CROSS_DETECT_WORK_MAX_PX=200`) bounds the pixel work but not these per-point costs. [[project_20260827_framerate_and_h_texture_investigation]] reported the MEAN rate fix (46.5 Hz); the terminal band was never measured separately.

**Open / unverified:** live gap (48–96 ms) is ~2–3× the offline time (28 ms); likely GIL contention (controller 83 Hz + ROS callbacks in the same process) and sim CPU load — NOT instrumented live. Also not shown causally that staleness (vs close-range s noise) drives the terminal xy/speed loss — a fix-and-rerun of the sperc arm would confirm.

**Fix 1 tried (2026-09-24), NO live effect.** `_pts_tuple()` (`.tolist()`) in `_scale/_shift_detection`: bit-identical (458 frames, full process_frame s/h equal), offline close-range detect 34→26 ms. Re-run sperc IC1-5×5 (`test_data/SPercGTFB_AB_tolist/`): live frame gap 0.5–1 m 48→48 ms, 0.15–0.3 m 96→84 ms; manuscript SP 6→9/25 (Fisher p=0.54), vel mean 0.26→0.30 (p=0.40) — noise. ⇒ offline detect cost is NOT what sets the live rate (live is 2–3× offline); don't chase fix 2 (fitLine subsample) blind. Next: instrument live wall-clock per stage, and/or causal test — GT s zero-order-held at the recorded perception update times.

**⭐ CAUSAL TEST (2026-09-24) — staleness REFUTED.** `PLASMC_GT_S_HOLD=stamp` (controller.py, default off): GT s refreshed only when perception finishes a frame, sampled at its capture stamp (reproduces hold+latency; held-fraction 0.50→0.83 matches sperc). Arm `gthold` IC1-5×5 (`test_data/SPercGTFB_AB_gthold/`): **25/25** manuscript SP, xy 0.015 m, vel 0.018 m/s — indistinguishable from gt (MWU xy p=0.91, vel p=0.38), vs sperc p=2.7e-7 / 1.4e-9. ⇒ the terminal gap is the VALUE of perception s below ~0.5 m (RMSE 0.036→0.078, p95 0.19), not its rate. Speeding up the pipeline is NOT the fix; the close-range centroid estimate is.

**⭐ 1/z GAIN ALSO REFUTED (2026-09-24).** GT-FB feeds s = x/(z+0.2) (`gt_feedback.py` Z_REG) but a camera measures the TRUE bearing x/z — 1.27x/1.5x/1.9x larger at 0.75/0.4/0.22 m. Arm `gtbear` (`PLASMC_GT_S_Z_REG=0.02`, s only; h keeps 0.2), IC1-5×5 (`test_data/SPercGTFB_AB_gtbear/`): **25/25** manuscript SP, xy 0.009 m (BETTER than gt's 0.015, p=0.002), vel 0.021. ⇒ the controller is fine with true-bearing s; the stationary sperc gap is purely perception s VALUE error, concentrated below ~0.3 m. ⚠ Scoring perception against V_s_g (regularized) books correct close-range bearings as 30-90% errors — score against `compute_gt_flow(...)['V_s_true']` (added 2026-09-24; `validate_detector_gt.py` now defaults to it, `--ref reg` for old numbers). On RobustnessFrameset/base the current detector vs TRUE bearing: err med 0.006-0.017 in every band down to 0.3 m, 0.046 below.

**Moving target (2026-09-24, `test_data/SPercGTFB_rover/`, target speed verified 0.55/0.30 m/s):** sperc Sinusoidal xy med 2.52 m (0/5), Circular 0.15 m but vel 1.56. Mechanism: detector isolates the rover platform's dark L-shaped SIDE FACE/SHADOW as the cross, returns the plate CORNER (span rescue admits it — its own comment documents ~23 bad/100 on rover); drone lags, marker exits frame, s HELD frozen -> runaway. gthold also 0/5 SP (vel ~0.65) — needs a moving `gt` arm to judge staleness (queued). User chose the FULL locked-design rewrite ([[feedback_cross_detector_robustness_requirement]]), eval set first.

**How to apply:** work on the close-range (<0.5 m, extent ≳290 px, marker overfilling) centroid accuracy of cross_marker_perception s, then re-run the sperc arm of `run_sperc_gtfb_ab.sh`. Don't spend effort on frame rate for this.
