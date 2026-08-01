---
name: project_cross_marker_pipeline_20260801
description: "Cross+stub fiducial pipeline built 2026-08-01 as a decode-free ArUco alternative — status, architecture, and what's next"
metadata: 
  node_type: memory
  type: project
  originSessionId: dbc47060-f292-4fca-8a3d-75a6066d2b5c
  modified: 2026-08-01T15:08:59.355Z
---

Built and committed (916aa53, pushed to main) a standalone perception pipeline for a
decode-free cross+stub marker, as an alternative to ArUco. Motivated by ArUco's
decode-coupled corner-dropout failure mode and the handover/PlanarFeatureMap
complexity needed to extend its decodable range across altitude.

**Why:** yaw is not currently needed (design confirmed early in the build), so the
marker doesn't need ArUco's ID/orientation-from-decode — center recovery via line
geometry is enough, plus an optional one-sided stub for heading if ever needed.

**Architecture (separate from ArUco, not routed through img_data.py's IMG_PROCESSOR):**
- `src/cross_marker_detector.py` — rotation-invariant relative-angle line clustering +
  robust line intersection for center. Handles partial occlusion (per-line robust fit
  degrades gracefully), off-frame extrapolation fallback (intersection can be computed
  even if it lands outside the visible frame — validated separately from a
  support-gated cluster-pairing fix so perspective skew under tilt doesn't let a
  spurious ~90°-apart noise pair beat the real, non-exactly-90° arms). Mask-centroid
  consistency check rejects biased in-frame fits (split into in-frame vs off-frame
  branches with different criteria).
- `src/cross_marker_perception.py` — computes the manuscript's actual image features
  (not the project's ArUco-shaped naming): `s` = homogeneous centroid `[xc,yc,1]`,
  `alpha` = unweighted 2nd-moment orientation over real detected pixels (no ArUco-style
  synthetic corner weighting — the stub's asymmetry is real), `h,w` = optical flow via
  the same image-Jacobian pseudo-inverse as ArUco, fed Shi-Tomasi points sampled across
  the whole color/shape-gated marker plate (not just 4 corners) — this fixes ArUco's
  documented point-starvation and Jacobian ill-conditioning problems with real data
  instead of synthetic scaled-quad points. Point-count-robust h/w tracking added after
  diagnosing GFT starvation at range (mask dilation + two-threshold hysteresis).
- `src/controller.py` — routes to `CrossMarkerNode` instead of `IMG_PROCESSOR` under
  `MARKER_TYPE=cross`. Single-point visibility CBF wired in (reuses `cbf_visibility.py`
  unmodified — its Phase-1 barrier was already centroid-only by design, corner-extent
  only mattered for Phase-2 fallback, which degenerates cleanly to zero for a
  single-point input).
- `src/img_data.py` — reverted to ArUco-only (an earlier synthetic-4-corner-packaging
  approach into IMG_PROCESSOR was tried first, then superseded by the separate pipeline
  once the "don't need PlanarFeatureMap/handover" design decision was made explicit).
- Gazebo asset: `cross_marker` model (3m plane, speckle-textured — texture chosen for
  KLT/Shi-Tomasi feature density) + `cross_marker.sdf` world + dedicated launcher
  `scripts/run_cross_marker_altitude_test.sh` (copied from `run_aruco_landing.sh`
  rather than parameterizing it, to keep the real hardware-adjacent launcher
  untouched). `scripts/validate_image_feed.sh` parameterized with `WORLD`/
  `VALIDATE_SCRIPT` env vars for reuse.
- Test apps: `apps/cross_marker_hover_sanity.py` (exercises the real perception
  pipeline hover-only, no PLASMC loop), `apps/cross_marker_altitude_test.py`,
  `apps/cross_marker_tilt_test.py` (oscillatory attitude commands to test detection
  under real tilt — found the drone drifts the marker out of frame fast with no
  position hold, so tilt validation is time-limited per attempt).

**Validated:** offline (synthetic rotation/occlusion sweep + saved live frames) and
live SITL (ground level, 5m, 7m hover — center detection solid at all three; h/w and
alpha noisier at range but degrade gracefully, not garbage). Partial live tilt
validation (~17° real pitch, short windows before drift) — coherent, no evidence of
the pairing bug recurring, but not a long sustained trial.

**Known gaps / NOT done:**
- `CrossMarkerNode` only implements the interface subset `controller.py`'s hover path
  touches (`center`, `focal`, `close`, `is_alive`, `join`, `CONTROLLER_READY`,
  `FEATURE_IS_VISIBLE`, `_feature_pts`, `getImgFeatureParam`, `getOptFlowAngVel`,
  `getLogData`, `getParams`, `RECORD`, `_ring_loom_source`, `get_center_px`). Terminal-
  kick, ring-loom-fusion, and other descent-specific `IMG_PROCESSOR` attributes are
  NOT implemented — a real closed-loop descent attempt will likely hit
  `AttributeError`s there. This is an explicit, known scope boundary, not silent.
  Fix needed before a real landing attempt.
- Residual sanity-check gap: symmetric-occlusion cases can bias both the mask centroid
  and the line-fit intersection in the same direction (~24px error slipped through in
  the characterization sweep) — the centroid-consistency check can't catch a bias it
  shares with the reference it's checked against.
- Ship-heave test (rocking the marker plate via Gazebo's `set_pose` service, to
  isolate perspective-skew testing from drone-drift confound) was proposed but not
  built — user chose to accept partial tilt data instead.
- `s` (position) is solid and load-bearing; `alpha` explicitly tolerates interruption
  (hold-last-good, no observability-metric gate — user confirmed yaw control handles
  this fine); `h,w` degrades to zero-output under point starvation rather than
  garbage — none of these three needs re-litigating design-wise, only further testing/
  tuning if problems surface.

**Next planned step (as of 2026-08-01):** output calibration (`_sensor_cal_hw`
equivalent) for the cross-marker path before further SITL validation — the ArUco
sensor-cal workflow (`io-calibration` skill) will need adapting since the cross
marker's h/w computation is architecturally different (own image-Jacobian solve, not
routed through img_data.py at all).
