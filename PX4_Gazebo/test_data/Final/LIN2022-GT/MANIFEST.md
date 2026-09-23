# test_data/Final/LIN2022-GT — comparison baseline: Lin et al. 2022 (PBVS+PPC)

Code: main @ f8bcaf42 (recorded), 310c50c5 (montage fix applied). `PLASMC_GT_FEEDBACK=1
PLASMC_BASELINE=lin2022` (`src/baselines.py::Lin2022`, PBVS with prescribed-performance
control, fed ground-truth pose+velocity directly per the MATLAB comparison harness
convention), `WORLD=rover_cross[_deck] ROVER_MODEL=rover_cross[_ackermann]`,
`MARKER_TYPE=cross`, default params, ONE attempt per case (no SoftPrecise retry gate —
comparison baselines are not required to land precisely; see
`scripts/run_baseline_campaign.sh`).

Each `<case>/` holds `<case>_montage.mp4` (chase + onboard PiP + GT plot panel),
`<case>_onboard_cam.mp4`, `<case>_chase_cam.mp4`, and `dataset/` (Control_Data,
Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data). No
`overlay_s_alpha`/`overlay_h` feature-overlay PiPs for baselines (scope decision
2026-09-23, predates finding `tools/overlay_image_features.py` reusable — not
re-litigated).

| Case | Outcome | xy_err (m) | rel_vel (m/s) | precise | soft | dur (s) | source run |
|------|---------|-----------|---------------|---------|------|---------|------------|
| IC1 | Landed | 0.209 | 3.320 | False | False | 17.2 | `Wed Sep 23 04-38-32 2026` |
| IC2 | **Aborted** (descent stall) | — | — | — | — | 34.2 | `Wed Sep 23 04-41-26 2026` |
| IC3 | **Aborted** (descent stall) | — | — | — | — | 34.1 | `Wed Sep 23 04-45-45 2026` |
| IC4 | Landed | 0.372 | 1.990 | False | False | 25.4 | `Wed Sep 23 04-47-34 2026` |
| IC5 | **Aborted** (descent stall) | — | — | — | — | 32.7 | `Wed Sep 23 04-50-29 2026` |
| Static | **Aborted** (descent stall) | — | — | — | — | 34.1 | `Wed Sep 23 04-54-35 2026` |
| Linear | Landed | 1.491 | 1.544 | False | False | 5.0 | `Wed Sep 23 04-58-08 2026` |
| Sinusoidal | Landed | 0.248 | 2.085 | False | False | 13.7 | `Wed Sep 23 04-59-57 2026` |
| Circular | Landed | 1.230 | 9.520 | False | False | 13.6 | `Wed Sep 23 05-02-13 2026` |
| Lissajous | Landed | 0.726 | 4.208 | False | False | 20.0 | `Wed Sep 23 05-04-46 2026` |

**Result: 6/10 landed, 0/10 precise+soft.** "Aborted" = `RuntimeError: descent stall:
no >0.30 m descent in 25s — hovering, aborting` (the harness's own descent-stall
watchdog, not a crash). Dataset/video are still recorded/promoted for aborted cases —
the flight just stops progressing rather than terminating abnormally.

Notes:
- **`IC3` has no `IC3_montage.mp4` and no `IC3_onboard_cam.mp4`** — that rep's onboard
  down-cam recording never saved during the original campaign run (unrelated failure,
  root cause not investigated). `IC3_chase_cam.mp4` and the full `dataset/` are present
  and valid. Attempting to regenerate the montage against the missing onboard file
  produces a broken 0-frame PiP — don't retry without first re-recording IC3's onboard
  footage. See `Memory/px4/feedback_montage_touchdown_argmin_bug.md`.
- Montages for the aborted cases (IC2/IC3(n/a)/IC5/Static) were regenerated 2026-09-23
  after a touchdown-detection bug fix in `tools/make_landing_montage.py` — the original
  versions truncated both the plots and the video itself early because they mistook a
  hover-noise altitude dip for touchdown. See
  `Memory/px4/feedback_montage_touchdown_argmin_bug.md` for the full root cause.
- All 10 cases use the same GT-FB rover-world cross-marker recipe as `VISTA-GT/`
  (`../VISTA-GT/`), so `precise`/`soft` are directly comparable against that set's
  numbers at the same 0.10 m / 0.2 m/s harness thresholds.
