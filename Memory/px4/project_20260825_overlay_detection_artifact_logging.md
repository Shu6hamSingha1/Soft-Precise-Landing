---
name: project_20260825_overlay_detection_artifact_logging
description: "2026-08-25: closed a real gap in tools/overlay_image_features.py -- it was re-running detect()/LK offline on the recorded (lossily re-encoded) mp4 to draw s-lines/h-flow-field, which sometimes disagreed with the live Detection Status. Fixed by logging the actual raw-pixel detection artifacts (Line Points I/J, Stub Points, Flow Points Prev/Curr Px) in cross_marker_perception.py's getLogData(), with the overlay tool preferring them and falling back to offline reconstruction only for pre-fix runs."
metadata: 
  node_type: memory
  type: project
  modified: 2026-08-25T15:50:47.181Z
  originSessionId: 64366844-45aa-444e-96db-ea10a4e7d714
---

## What was found

`tools/overlay_image_features.py` (built [[project_20260824_cross_marker_montage_overlay_robustness]]
session, extended to draw decoded s-lines/intersection, an alpha-0 reference arrow+arc, and an
h tracked-points flow field) was re-deriving the underlying line/stub/flow-point geometry by
re-running `cross_marker_detector.detect()` and its own LK tracking OFFLINE on the recorded video.
Spot-check on the `linear_verify2` run found real disagreement: frames 30/35 have
`Detection Status='ok'` in the live log, but offline re-detect on the (lossily re-encoded) mp4
returned `insufficient_fit_points`. The tool was designed fail-safe (never draws WRONG lines, just
omits them on a reconstruction miss), so it never showed false info -- but it was an unreliable
lower bound on real detection uptime, unsuitable for debugging genuine detection flicker (the
user's stated eventual use case for these overlays, beyond just explaining the s/alpha/h math).

## Fix

`cross_marker_perception.py`: added 5 new per-frame logs, populated directly from the same
`det`/`_compute_hw` objects the live pipeline already computes (no re-derivation):
- `Line Points I` / `Line Points J` -- raw px points fit to each cross-arm line (split out of the
  previously unexposed, combined `_feature_pts_raw_log`)
- `Stub Points` -- raw px points fit to the stub line
- `Flow Points Prev/Curr Px` -- the exact raw-pixel LK correspondences `_solve_jacobian`'s V-frame
  lstsq solved, snapshotted in `_compute_hw` (new `self._last_flow_prev_px`/`_curr_px`) BEFORE
  `_getVirtualPts` normalization -- distinct from the existing `Point Diag Log`, which already
  logged prev_n/curr_n but in normalized V-frame coords, not raw pixels (can't be drawn on the
  video without inverting the leveling transform).

`overlay_image_features.py`: prefers these logged arrays when present (`have_logged_pts` gate on
`Line Points I` being non-None), falls back to the old offline re-detect/re-track only for runs
recorded before this fix, printing a `[overlay] NOTE:` when that fallback fires.

**Validated**: fresh `gapfill_verify` run (882 frames) -- `Line Points I` nonempty status has ZERO
mismatches against `Detection Status=='ok'` (guaranteed by construction now, both come from the
same `det` at the same time).

## How to apply

Any run recorded 2026-08-25 or later (after this commit) gets exact-parity overlays automatically
-- no action needed. Runs recorded BEFORE this fix (e.g. `linear_verify2`, `static` from
2026-08-24) will still print the fallback NOTE and use offline reconstruction when overlaid --
re-record if exact per-frame detection-flicker debugging is needed on those specific scenarios.
