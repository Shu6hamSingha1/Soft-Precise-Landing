# test_data/Final — IC1-5 perception-mode cross-marker landings (final recordings)

Code: main @ 4d7bc210 (post-revert baseline). Perception feedback (NO PLASMC_GT_FEEDBACK),
MARKER_TYPE=cross, WORLD=cross_marker, CROSS_ALPHA_0=radians(0.58), default params.

Each IC<n>/ holds: <IC>_montage.mp4 (combined: onboard s/alpha + onboard h/w + chase + plots),
<IC>_onboard_cam.mp4, <IC>_chase_cam.mp4, <IC>_overlay_s_alpha.mp4, <IC>_overlay_h_w.mp4,
and dataset/ (Control_Data, Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data).

| IC | init ENU (E,N,U) | xy_err (m) | rel_vel (m/s) | precise | soft | source run |
|----|------------------|-----------|---------------|---------|------|------------|
| IC1 | 0,0,5 | 0.1222 | 0.4198 | False | False | `Mon Aug 31 16-17-08 2026` |
| IC2 | 2,2,5 | 0.0651 | 0.3644 | True | False | `Mon Aug 31 16-18-23 2026` |
| IC3 | -2,2,5 | 0.3042 | 0.7767 | False | False | `Mon Aug 31 19-17-04 2026` |
| IC4 | 2,2,7 | 0.1133 | 0.3938 | False | False | `Mon Aug 31 19-18-26 2026` |
| IC5 | 2,2,3 | 0.0576 | 0.5053 | True | False | `Tue Sep  1 08-44-32 2026` |

Notes:
- IC1-4: from the 2026-08-31 alpha0 montage batch (IMG_RECORD=1 run; IMG_RECORD perturbs
  touchdown, so IC1 0.122 / IC3 0.304 / IC4 0.113 m are misses on rel_vel/xy — the
  clean-touchdown baseline for these ICs is ICValidation/20260831-144626 (no video).
- IC2: PRECISE 0.065 m.
- IC5: re-recorded 2026-09-01 (montage_IC5_final_20260901), PRECISE 0.058 m — replaces the
  earlier alpha0 IC5 montage which was a 6.44 m TARGET_LOST.

---

# IC1-5, GT-FB, rover world, cross-marker (2026-09-22)

Code: main @ 5b342689. `PLASMC_GT_FEEDBACK=1` (GT-fed s/h, isolates CONTROL from
PERCEPTION), `WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=0` (stationary
target on the rover platform), `MARKER_TYPE=cross`, default params, 1280x960 chase cam.

Each IC<n>_RoverGTFB/ holds the same 5 videos + dataset/ as the perception-mode IC1-5
above (`<IC>_montage.mp4`, `<IC>_onboard_cam.mp4`, `<IC>_chase_cam.mp4`,
`<IC>_overlay_s_alpha.mp4`, `<IC>_overlay_h.mp4`).

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
  no freeze artifact (a peer-session concern raised and then retracted the same day, see
  Memory/px4).
- Compare against the perception-mode IC1-5 above: same 5 spawn positions, isolates how
  much of that set's misses (IC1/IC3/IC4 failing precise/soft) were perception- vs
  control-driven -- this GT-FB set landing 5/5 SP cleanly suggests those misses are
  perception-side, not a control-law weakness at these ICs.
