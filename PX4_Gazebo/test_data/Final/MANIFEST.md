# test_data/Final — IC1-5, GT-FB, rover world, cross-marker (final recordings)

Code: main @ 5b342689. `PLASMC_GT_FEEDBACK=1` (GT-fed s/h, isolates CONTROL from
PERCEPTION), `WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=0` (stationary
target on the rover platform), `MARKER_TYPE=cross`, default params, 1280x960 chase cam.

Each IC<n>/ holds: <IC>_montage.mp4 (combined: onboard s/alpha + onboard h + chase +
plots), <IC>_onboard_cam.mp4, <IC>_chase_cam.mp4, <IC>_overlay_s_alpha.mp4,
<IC>_overlay_h.mp4, and dataset/ (Control_Data, Control_Params, Ground_Truth, Img_Data,
Img_Params, Telemetry_Data).

| IC | init ENU (E,N,U) | xy_err (m) | rel_vel (m/s) | precise | soft | source run |
|----|------------------|-----------|---------------|---------|------|------------|
| IC1 | 0,0,5 | 0.0093 | 0.1047 | True | True | `Tue Sep 22 23-34-24 2026` |
| IC2 | 2,2,5 | 0.0125 | 0.0948 | True | True | `Tue Sep 22 23-35-44 2026` |
| IC3 | -2,2,5 | 0.0298 | 0.0997 | True | True | `Tue Sep 22 23-37-06 2026` |
| IC4 | 2,2,7 | 0.0215 | 0.1051 | True | True | `Tue Sep 22 23-38-31 2026` |
| IC5 | 2,2,3 | 0.0138 | 0.1114 | True | True | `Tue Sep 22 23-39-48 2026` |

Notes:
- All 5 SoftPrecise on the FIRST launch attempt, no retries needed.
- All 5 clear the manuscript's strict 0.08m/0.2m/s thresholds directly (recorded here
  against this harness's default LANDING_PRECISE_TOL=0.10m; re-checked against 0.08m too
  -- still 5/5).
- Chase videos verified frame-by-frame (0/213 identical consecutive pairs on IC1) --
  no freeze artifact.
- 2026-09-23: replaces the prior perception-mode (`cross_marker` world, no GT-FB) IC1-5
  set, which missed precise/soft on IC1/IC3/IC4 (xy 0.11-0.30m, rel_vel 0.36-0.78m/s).
  That set is still in git history if needed for a perception-vs-control comparison;
  this GT-FB set is now the sole one under `test_data/Final/IC<n>/`.
