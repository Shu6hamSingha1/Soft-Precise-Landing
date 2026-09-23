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
check was precise+soft (best: 0.18 m / 2.04 m/s, and 0.28 m / 0.36 m/s). The same check was then run for lin2022, zhang2026 and cho2022 -- see below.

**Run-to-run variance check, all 4 baselines (2026-09-24):** IC1 and Lissajous re-run 3 more
times each per baseline, same code, nothing changed between runs.

| Baseline | Case | Campaign ; rep 1 ; rep 2 ; rep 3 (xy / rel_vel m/s; "(deck)" = on the 0.6 m deck) | Landed | On deck | xy range | Precise+soft |
|---|---|---|---|---|---|---|
| lin2022 | IC1 | 0.21 m / 3.32 (deck) ; 0.67 m / 1.46 ; stall abort ; 0.68 m / 4.53 | 3/4 | 1/4 | 0.21-0.68 m | 0/4 |
| lin2022 | Lissajous | 0.73 m / 4.21 ; 0.85 m / 4.17 ; 0.12 m / 1.54 (deck) ; 0.25 m / 1.99 (deck) | 4/4 | 2/4 | 0.12-0.85 m | 0/4 |
| zhang2026 | IC1 | 2.65 m / 4.28 ; 0.45 m / 1.90 ; 0.14 m / 0.31 (deck) ; 0.44 m / 0.51 | 4/4 | 1/4 | 0.14-2.65 m | 0/4 |
| zhang2026 | Lissajous | 12.19 m / 14.98 ; 10.64 m / 12.33 ; 11.17 m / 13.41 ; 10.02 m / 11.73 | 4/4 | 0/4 | 10.0-12.2 m | 0/4 |
| cho2022 | IC1 | stall abort x4 | 0/4 | 0/4 | — | 0/4 |
| cho2022 | Lissajous | stall abort x4 | 0/4 | 0/4 | — | 0/4 |
| lin2023 | IC1 | 0.44 m / 1.39 ; 0.28 m / 1.60 (deck) ; 5.11 m / 18.01 ; 0.18 m / 2.04 (deck) | 4/4 | 2/4 | 0.18-5.11 m | 0/4 |
| lin2023 | Lissajous | 0.93 m / 2.45 ; 0.28 m / 0.36 (deck) ; stall abort ; stall abort | 2/4 | 1/4 | 0.28-0.93 m | 0/4 |

**How far to trust a single campaign recording depends on the controller AND the case:**
- *Consistent failures are trustworthy at n=1:* cho2022 aborted 8/8 (every run, both cases);
  zhang2026 Lissajous missed by 10-12 m at 12-15 m/s in 4/4.
- *Everything else is not:* the same case can land on the deck, crash beside it, or abort.
  zhang2026 IC1's campaign run (2.65 m) was an outlier -- 3 repeats landed at 0.14-0.45 m, so the
  campaign UNDERSTATED zhang2026 on the static case. lin2023 IC1 spans 0.18-5.11 m.
- lin2022 is the steadiest landing baseline (7/8 landed, xy 0.12-0.85 m, no multi-metre outliers).
- **0/32 runs precise+soft** across all four baselines. Closest: zhang2026 IC1 0.14 m / 0.31 m/s,
  lin2022 Lissajous 0.12 m / 1.54 m/s (precise needs <=0.10 m, soft <=0.2 m/s).

Verification: all 7 no-result repeats were confirmed as the harness's descent-stall watchdog by
replaying its logic on the recorded altitude -- each ends 25.00 s after its last >0.3 m descent.
Nothing runtime-relevant changed between the campaign and the repeats (only commits since are this
session's own; the B_T saturation is actuator-equivalent to <=6e-5 throttle; `zf` is not on the
lin2022/zhang2026 code path). Only IC1 and Lissajous were repeated -- the other 8 cases per baseline
are single draws of unknown spread. Repeats stay untracked in `RecordBaseline_dev/` (not promoted,
to avoid cherry-picking). **For a paper table, use several runs per case, not these single recordings.**
