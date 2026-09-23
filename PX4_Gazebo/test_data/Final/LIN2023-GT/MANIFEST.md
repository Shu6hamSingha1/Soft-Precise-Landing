# test_data/Final/LIN2023-GT — comparison baseline: Lin et al. 2023 (robust circle-feature IBVS + funnel)

**RE-RECORDED 2026-09-23 (evening) with the corrected `BASELINE_ZF=0.2`** (MATLAB/Common/
Constants.m canonical value; the first recording used a stale 0.3 default). The old
zf=0.3 set is superseded (in git history at f8bcaf42/310c50c5). Code: main @ f90d448f.
`PLASMC_GT_FEEDBACK=1 PLASMC_BASELINE=lin2023` (`src/baselines.py::Lin2023`, robust IBVS
with a prescribed-performance funnel on circle/marker features; replaced Chen2025),
`WORLD=rover_cross[_deck] ROVER_MODEL=rover_cross[_ackermann]`, `MARKER_TYPE=cross`,
default params, ONE attempt per case (no SoftPrecise retry gate; see
`scripts/run_baseline_campaign.sh`, re-run via `BASELINES=lin2023`).

Each `<case>/` holds `<case>_montage.mp4` (chase + onboard PiP + GT plot panel),
`<case>_onboard_cam.mp4`, `<case>_chase_cam.mp4`, and `dataset/` (Control_Data,
Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data). No
`overlay_s_alpha`/`overlay_h` PiPs for baselines (scope decision 2026-09-23).

| Case | Outcome | xy_err (m) | rel_vel (m/s) | precise | soft | dur (s) | source run |
|------|---------|-----------|---------------|---------|------|---------|------------|
| IC1 | Landed | 0.444 | 1.391 | False | False | 19.2 | `Wed Sep 23 20-23-22 2026` |
| IC2 | **Aborted** (descent stall) | — | — | — | — | 39.7 | `Wed Sep 23 20-26-38 2026` |
| IC3 | **Aborted** (descent stall) | — | — | — | — | 40.0 | `Wed Sep 23 20-31-10 2026` |
| IC4 | **Aborted** (descent stall) | — | — | — | — | 45.2 | `Wed Sep 23 20-35-59 2026` |
| IC5 | **Aborted** (descent stall) | — | — | — | — | 40.2 | `Wed Sep 23 20-40-52 2026` |
| Static | **Aborted** (descent stall) | — | — | — | — | 39.7 | `Wed Sep 23 20-45-25 2026` |
| Linear | Landed | 9.194 | 3.792 | False | False | 5.7 | `Wed Sep 23 20-49-08 2026` |
| Sinusoidal | Landed | 4.427 | 5.988 | False | False | 38.9 | `Wed Sep 23 20-51-37 2026` |
| Circular | **Aborted** (descent stall) | — | — | — | — | 40.4 | `Wed Sep 23 20-56-09 2026` |
| Lissajous | Landed | 0.934 | 2.453 | False | False | 40.6 | `Wed Sep 23 21-00-52 2026` |

**Result: 4/10 landed, 0/10 precise+soft.** "Aborted" = `RuntimeError: descent stall:
no >0.30 m descent in 25s -- hovering, aborting`.

**Effect of the zf fix (old zf=0.3 -> new zf=0.2), single attempt each, so treat as noisy:**
IC1 5.20m/11.98 -> 0.44m/1.39 (much better); Lissajous 11.25m/14.14 -> 0.93m/2.45 (much
better); Linear 3.47m/16.27 -> 9.19m/3.79 (mixed); Sinusoidal 0.53m/3.11 -> 4.43m/5.99
(worse); Circular landed -> aborted (worse); IC2-5/Static aborted both times. Net: 5/10 ->
4/10 landed, no consistent improvement and still 0 precise/soft -- the zf fix is a
correctness alignment with MATLAB, not a performance fix. n=1 per case; don't read
per-case swings as signal.

Notes:
- Montages were built with the touchdown-detection fix in `tools/make_landing_montage.py`
  (aborted cases use the full untrimmed series); no 0-frame-PiP anomalies this run.
- Same GT-FB rover-world cross-marker recipe as `VISTA-GT/` (`../VISTA-GT/`).
