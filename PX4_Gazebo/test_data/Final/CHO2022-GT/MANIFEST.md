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
control law. **Root-caused 2026-09-23** (see
`Memory/px4/feedback_cho2022_never_lands_rootcause.md`): this baseline's FF-IBVS law is
a pure feature-error regulator converging to a FIXED, shallow depth setpoint
(`~0.4m` camera-to-marker, hard-coded in `controller.py::_baselineStep`'s `px_d`
formula) — once converged, the commanded acceleration settles to EXACT hover
(verified in `Control_Data.npy`: `I_a_z -> -9.81 m/s^2` by ~t=9s, held for the rest of
the flight) and the drone simply parks a few cm above the platform, never producing the
genuine physical ground contact (PX4 `LandedState`/accelerometer impact spike) needed
to register as landed. Baseline runs also bypass PLASMC's own loom-inversion touchdown
latch entirely (it lives inside `self.PLASMC()`, never called when a baseline is
active), so that fallback path is unavailable too. The other 3 baselines land at least
sometimes because their control laws overshoot past hover into forceful contact
(zhang2026 lands with `rel_vel` up to 18.5 m/s); cho2022's smooth, non-overshooting
regulation never does.

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
