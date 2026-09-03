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
