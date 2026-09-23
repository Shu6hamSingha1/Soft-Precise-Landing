# test_data/Final/CHO2022-GT — comparison baseline: Cho et al. 2022 (FF-IBVS)

Code: main @ f8bcaf42 (recorded), 310c50c5 (montage fix applied). `PLASMC_GT_FEEDBACK=1
PLASMC_BASELINE=cho2022` (`src/baselines.py::Cho2022`, feature-following IBVS with an
adaptive-sigmoid altitude gain, fed marker key points projected through the ideal
pinhole), `WORLD=rover_cross[_deck] ROVER_MODEL=rover_cross[_ackermann]`,
`MARKER_TYPE=cross`, default params, ONE attempt per case (no SoftPrecise retry gate —
see `scripts/run_baseline_campaign.sh`).

Each `<case>/` holds `<case>_montage.mp4` (chase + onboard PiP + GT plot panel),
`<case>_onboard_cam.mp4`, `<case>_chase_cam.mp4`, and `dataset/` (Control_Data,
Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data). No
`overlay_s_alpha`/`overlay_h` feature-overlay PiPs for baselines (scope decision
2026-09-23).

| Case | Outcome | xy_err (m) | rel_vel (m/s) | precise | soft | dur (s) | source run |
|------|---------|-----------|---------------|---------|------|---------|------------|
| IC1 | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 06-04-34 2026` |
| IC2 | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 06-07-59 2026` |
| IC3 | **Aborted** (descent stall) | — | — | — | — | 32.6 | `Wed Sep 23 06-11-31 2026` |
| IC4 | **Aborted** (descent stall) | — | — | — | — | 34.8 | `Wed Sep 23 06-14-47 2026` |
| IC5 | **Aborted** (descent stall) | — | — | — | — | 29.5 | `Wed Sep 23 06-18-09 2026` |
| Static | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 06-21-45 2026` |
| Linear | **Aborted** (descent stall) | — | — | — | — | 36.6 | `Wed Sep 23 06-25-21 2026` |
| Sinusoidal | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 06-28-05 2026` |
| Circular | **Aborted** (descent stall) | — | — | — | — | 36.1 | `Wed Sep 23 06-30-38 2026` |
| Lissajous | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 06-34-01 2026` |

**Result: 0/10 landed.** Every single case hits the identical
`RuntimeError: descent stall: no >0.30 m descent in 25s — hovering, aborting` — the
harness's own descent-stall watchdog, not a crash or exception in the baseline's own
control law. This is a consistent, reproducible failure (same error, same failure mode,
10/10 cases) — looks like a real property of this FF-IBVS baseline under this
SITL/GT-FB harness (it appears to never initiate a sustained descent at all), not
run-to-run noise the way the other baselines' scattered stalls are. **Root cause not
yet investigated** — worth digging into `src/baselines.py::Cho2022`/
`controller.py::_baselineStep`'s handling of it if this needs explaining for the
manuscript. See `Memory/px4/project_20260923_comparative_baseline_campaign.md`.

Notes:
- Video+dataset are still recorded/promoted for every case despite none landing — this
  is a genuine negative result (constant hover near the platform, never descending),
  not missing/corrupted data.
- All 10 montages were regenerated 2026-09-23 after a touchdown-detection bug fix in
  `tools/make_landing_montage.py` was found from a user report that the plots and
  chase-cam feed didn't match for these recordings. Root cause: the original montage
  generator mistook a mm-scale altitude-sensor noise dip during the hover for a real
  touchdown, silently truncating both the plots AND the composited video itself well
  before the recording's actual end (e.g. `Circular`: 24.6s → 37.0s after the fix, a
  ~12s recovery). See `Memory/px4/feedback_montage_touchdown_argmin_bug.md` for the
  full root cause and fix.
- All 10 cases use the same GT-FB rover-world cross-marker recipe as `VISTA-GT/`
  (`../VISTA-GT/`).
