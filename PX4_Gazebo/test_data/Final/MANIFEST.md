# test_data/Final — VISTA-GT: IC1-5 + trajectory cases, GT-FB, rover world, cross-marker

All recordings below live under `VISTA-GT/` (proposed VISTA controller, GT-feedback
mode). Folders: `IC1`-`IC5` (manuscript initial conditions, stationary target) and
`Static`, `Linear`, `Sinusoidal`, `Circular`, `Lissajous` (trajectory-type cases).

Code: main @ 5b342689. `PLASMC_GT_FEEDBACK=1` (GT-fed s/h, isolates CONTROL from
PERCEPTION), `WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=0` (stationary
target on the rover platform), `MARKER_TYPE=cross`, default params, 1280x960 chase cam.

Each `VISTA-GT/<case>/` holds: `<case>_montage.mp4` (combined: onboard s/alpha +
onboard h + chase + plots), `<case>_onboard_cam.mp4`, `<case>_chase_cam.mp4`,
`<case>_overlay_s_alpha.mp4`, `<case>_overlay_h.mp4`, and `dataset/` (Control_Data,
Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data).

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
  this GT-FB set is now the sole one, under `VISTA-GT/IC<n>/`.
- 2026-09-23: all case folders (IC1-5 + Static/Linear/Sinusoidal/Circular/Lissajous)
  moved from `test_data/Final/<case>/` into `test_data/Final/VISTA-GT/<case>/` to group
  all VISTA (proposed) GT-FB rover-world cross-marker recordings under one namespace.
- 2026-09-23 (later): **`Lissajous` REPLACED.** The previous rep (`Tue Sep 22 23-16-02
  2026`, xy=0.0128m) landed on a rover that was PARKED for its final 6.5s of descent --
  effectively a static landing, not genuine Lissajous tracking (see
  `Memory/px4/project_20260922_ackermann_rover_loops_not_tracking.md`: PX4's Ackermann
  offboard position mode treats each setpoint as an arrival point and stops within 0.5m,
  so a slow-moving setpoint just gets caught and parked). Old set backed up to
  `Obsolete/test_data/Lissajous_VISTA-GT_v1_parked_rover_20260923/`.
  New rep (`RecordGTFB_dev/Lissajous_23ratio_v2r02/Wed Sep 23 04-07-09 2026`) uses
  `rover_drive.py`'s velocity-tracking mode (`ROVER_CTRL=vel`, tracking error <=0.05m
  throughout) with a genuine 2:3-frequency-ratio Lissajous shape (the original profile's
  w1:w2 ratio was ~1:1, i.e. an ellipse, not a real Lissajous figure) at
  `ROVER_SPEED_MULT=7 ROVER_LISS_RADIUS_MULT=5` (tuned for v^2/R=0.204, the point found to
  balance landing precision against a visibly curving path -- see the memory file for the
  full progression/dead-ends). Result: xy=0.111m, rel_vel=0.821m/s (precise, not soft),
  real tilt <=3.5 deg throughout (verified via body-z-axis-from-vertical, not the
  yaw-conflated raw-quaternion angle). Montage/overlays regenerated from this rep.
