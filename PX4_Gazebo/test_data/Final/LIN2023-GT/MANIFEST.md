# test_data/Final/LIN2023-GT — comparison baseline: Lin et al. 2023 (robust circle-feature IBVS + funnel)

Code: main @ f8bcaf42 (recorded), 310c50c5 (montage fix applied). `PLASMC_GT_FEEDBACK=1
PLASMC_BASELINE=lin2023` (`src/baselines.py::Lin2023`, robust IBVS with a
prescribed-performance funnel on circle/marker features; replaced the earlier
Chen2025 baseline — see project CLAUDE.md), `WORLD=rover_cross[_deck]
ROVER_MODEL=rover_cross[_ackermann]`, `MARKER_TYPE=cross`, default params, ONE attempt
per case (no SoftPrecise retry gate — see `scripts/run_baseline_campaign.sh`).

Each `<case>/` holds `<case>_montage.mp4` (chase + onboard PiP + GT plot panel),
`<case>_onboard_cam.mp4`, `<case>_chase_cam.mp4`, and `dataset/` (Control_Data,
Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data). No
`overlay_s_alpha`/`overlay_h` feature-overlay PiPs for baselines (scope decision
2026-09-23).

| Case | Outcome | xy_err (m) | rel_vel (m/s) | precise | soft | dur (s) | source run |
|------|---------|-----------|---------------|---------|------|---------|------------|
| IC1 | Landed | 5.197 | 11.976 | False | False | 22.3 | `Wed Sep 23 05-26-05 2026` |
| IC2 | **Aborted** (descent stall) | — | — | — | — | 38.8 | `Wed Sep 23 05-29-37 2026` |
| IC3 | **Aborted** (descent stall) | — | — | — | — | 38.8 | `Wed Sep 23 05-34-08 2026` |
| IC4 | **Aborted** (descent stall) | — | — | — | — | 43.4 | `Wed Sep 23 05-38-43 2026` |
| IC5 | **Aborted** (descent stall) | — | — | — | — | 37.4 | `Wed Sep 23 05-43-16 2026` |
| Static | **Aborted** (descent stall) | — | — | — | — | 39.0 | `Wed Sep 23 05-47-42 2026` |
| Linear | Landed | 3.474 | 16.267 | False | False | 3.8 | `Wed Sep 23 05-51-21 2026` |
| Sinusoidal | Landed | 0.533 | 3.112 | False | False | 20.2 | `Wed Sep 23 05-53-13 2026` |
| Circular | Landed | 0.992 | 3.033 | False | False | 43.1 | `Wed Sep 23 05-56-48 2026` |
| Lissajous | Landed | 11.252 | 14.135 | False | False | 25.6 | `Wed Sep 23 06-01-13 2026` |

**Result: 5/10 landed, 0/10 precise+soft.** "Aborted" = `RuntimeError: descent stall:
no >0.30 m descent in 25s — hovering, aborting`. Dataset/video are still
recorded/promoted for aborted cases.

Notes:
- Montages for all 5 aborted cases (IC2/IC3/IC4/IC5/Static) were regenerated
  2026-09-23 after a touchdown-detection bug fix in `tools/make_landing_montage.py` —
  the originals truncated the plots/video early (a hover-noise altitude dip was
  mistaken for touchdown). See `Memory/px4/feedback_montage_touchdown_argmin_bug.md`.
- `Circular`'s montage was investigated (its argmin-touchdown index falls ~4% short of
  the recording's true end) but NOT regenerated — it's a genuine post-touchdown bounce
  (uz jumps from ~-0.012 to 0.13 m after touchdown), the same by-design crop-at-
  touchdown behavior `VISTA-GT` already uses, not the argmin bug. Left as originally
  recorded.
- All 10 cases use the same GT-FB rover-world cross-marker recipe as `VISTA-GT/`
  (`../VISTA-GT/`), directly comparable at the same 0.10 m / 0.2 m/s thresholds.
