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

**Platform-landing audit (2026-09-23, added after checking the geometry):** the landing deck is
a 0.6 x 0.6 m box (`rover_cross/model.sdf`, top at z=+0.50 m), so a centre >~0.3 m from the
target with `alt_above_surface_end` well below 0 is a *ground impact beside the platform*, not
a deck landing. The harness's `landed` flag (PX4 LandedState / accelerometer spike) does not
distinguish these. Read "landed" in the table above as "reached a physical impact", not "landed
on the target". This baseline: 0 on the deck; 4 'landed' cases are off-platform (IC1 xy 0.44 m is just past the deck edge; Linear 9.19 m, Sinusoidal 4.43 m, Lissajous 0.93 m); 6 aborted.

**Run-to-run variance check (2026-09-24):** re-ran two cases 3 more times each with the current
(`zf=0.2`) code, nothing changed between runs (repeats live only in the untracked
`RecordBaseline_dev/`; not promoted, to avoid cherry-picking a best rep).

| Case | Recording | Outcome |
|---|---|---|
| IC1 | old zf=0.3 (campaign) | 5.20 m / 11.98 m/s, off-deck |
| IC1 | zf=0.2 campaign | 0.44 m / 1.39 m/s (deck edge) |
| IC1 | zf=0.2 repeats 1, 2, 3 | 0.28 m / 1.60 (on deck); **5.11 m / 18.0 (off-deck)**; 0.18 m / 2.04 (on deck) |
| Lissajous | old zf=0.3 (campaign) | 11.25 m / 14.14 m/s, off-deck |
| Lissajous | zf=0.2 campaign | 0.93 m / 2.45 m/s, off-deck impact |
| Lissajous | zf=0.2 repeats 1, 2, 3 | 0.28 m / 0.36 (on deck); **aborted (stall)**; **aborted (stall)** |

Same code, same case: IC1 gave three near/on-deck landings and one 5.1 m / 18 m/s outlier;
Lissajous gave one on-deck landing, one off-deck impact and two stall aborts. **A single
campaign recording per case is one draw from a wide distribution** -- per-case numbers in the
table above are not reliable on their own. The old "5.2 m -> 0.44 m" IC1 improvement cannot be
credited to the `zf` fix: the old side is n=1 and the outlier mode reappears at `zf=0.2`. If
`zf=0.2` helps at all (IC1: 3 of 4 runs near the deck vs 0 of 1), it is unproven. No run in this
check was precise+soft (best: 0.18 m / 2.04 m/s, and 0.28 m / 0.36 m/s). Only lin2023 IC1 and
Lissajous were repeated; lin2022, zhang2026 and cho2022 variance is untested.
