---
name: project_20260924_terminal_perception_rate_drop
description: "2026-09-24 — perception s is fine but goes STALE near touchdown: process_frame drops 49 Hz -> ~12 Hz below 1 m because detect() cost scales with arm-pixel count (242 -> ~5.6k pts); decode does NOT fail. Explains most of sperc's 6/25 manuscript-SP vs GT 25/25."
metadata:
  node_type: memory
  type: project
  originSessionId: d1fc52b7-905a-496e-9fb1-d06cdf4109aa
  modified: 2026-09-23T20:27:35.581Z
---

**Test (2026-09-24).** GT-FB with only `s` from perception (`GT_ABLATE=h,hz,yaw,wz`) vs full GT-FB, stationary cross-marker, IC1-5 × n=5 (`scripts/run_sperc_gtfb_ab.sh`, data `test_data/SPercGTFB_AB/`). Both 25/25 at the relaxed 0.15 m/0.5 m/s; at the manuscript 0.08 m/0.2 m/s: GT **25/25**, sperc **6/25**. xy_err 0.015 → 0.059 m (MWU p=4.5e-7), touchdown speed 0.016 → 0.257 m/s.

**s accuracy by altitude** (`tools/analyze_s_perc_vs_gt.py`): RMSE ~0.017 normalized down to 0.5 m, no meaningful bias; 0.036 at 0.3–0.5 m; 0.057/0.078 below 0.3 m. So perception s is NOT the problem at altitude — the terminal band is.

**Why s thins out:** decode does NOT fail (352/31k misses, not altitude-concentrated). The processed-frame gap goes 16 → 48 → 68 → 96 ms (every 1st → 3rd → 6th camera frame at 62.5 Hz) as MARKER_EXTENT_PX saturates ~300 px (0.5-1 m band onward). Identical in the `gt` arm → a pipeline-throughput property, not caused by feeding s back.

**Stage timing** (`tools/time_perception_stages.py`, offline replay, identity quat, overlay inpainted): process_frame 5.6 ms (extent <100 px) → 28 ms (>290 px); `detect()` is ~75% of it. Inside detect at >290 px: ~5.6k line points (vs 242 far); `_detect_core` ~24 ms (mostly `cv2.fitLine` in `_robust_fit_line` over thousands of pts, ×3 iters, ~8 calls), plus `_scale_detection` + `_shift_detection` ~10 ms EACH — pure-Python `tuple(map(tuple, ...))` conversion of the point lists. The 2026-08-28 working-res cap (`CROSS_DETECT_WORK_MAX_PX=200`) bounds the pixel work but not these per-point costs. [[project_20260827_framerate_and_h_texture_investigation]] reported the MEAN rate fix (46.5 Hz); the terminal band was never measured separately.

**Open / unverified:** live gap (48–96 ms) is ~2–3× the offline time (28 ms); likely GIL contention (controller 83 Hz + ROS callbacks in the same process) and sim CPU load — NOT instrumented live. Also not shown causally that staleness (vs close-range s noise) drives the terminal xy/speed loss — a fix-and-rerun of the sperc arm would confirm.

**Fix 1 tried (2026-09-24), NO live effect.** `_pts_tuple()` (`.tolist()`) in `_scale/_shift_detection`: bit-identical (458 frames, full process_frame s/h equal), offline close-range detect 34→26 ms. Re-run sperc IC1-5×5 (`test_data/SPercGTFB_AB_tolist/`): live frame gap 0.5–1 m 48→48 ms, 0.15–0.3 m 96→84 ms; manuscript SP 6→9/25 (Fisher p=0.54), vel mean 0.26→0.30 (p=0.40) — noise. ⇒ offline detect cost is NOT what sets the live rate (live is 2–3× offline); don't chase fix 2 (fitLine subsample) blind. Next: instrument live wall-clock per stage, and/or causal test — GT s zero-order-held at the recorded perception update times.

**How to apply:** before claiming "s can be replaced by perception", fix the terminal throughput (keep point lists as numpy arrays through scale/shift; subsample points before fitLine) and re-run `run_sperc_gtfb_ab.sh`.
