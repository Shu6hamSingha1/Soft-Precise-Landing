# test_data/Final/ZHANG2026-GT — comparison baseline: Zhang & Wu 2026 (PBVS+AEDO)

Code: main @ f8bcaf42. `PLASMC_GT_FEEDBACK=1 PLASMC_BASELINE=zhang2026`
(`src/baselines.py::Zhang2026`, PBVS with an auxiliary estimator/disturbance-observer
term, fed ground-truth pose+velocity directly), `WORLD=rover_cross[_deck]
ROVER_MODEL=rover_cross[_ackermann]`, `MARKER_TYPE=cross`, default params, ONE attempt
per case (no SoftPrecise retry gate — see `scripts/run_baseline_campaign.sh`).

Each `<case>/` holds `<case>_montage.mp4` (chase + onboard PiP + GT plot panel),
`<case>_onboard_cam.mp4`, `<case>_chase_cam.mp4`, and `dataset/` (Control_Data,
Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data). No
`overlay_s_alpha`/`overlay_h` feature-overlay PiPs for baselines (scope decision
2026-09-23).

| Case | Outcome | xy_err (m) | rel_vel (m/s) | precise | soft | dur (s) | source run |
|------|---------|-----------|---------------|---------|------|---------|------------|
| IC1 | Landed | 2.650 | 4.278 | False | False | 20.2 | `Wed Sep 23 05-07-37 2026` |
| IC2 | Landed | 7.261 | 8.430 | False | False | 5.3 | `Wed Sep 23 05-10-09 2026` |
| IC3 | Landed | 4.701 | 12.435 | False | False | 5.3 | `Wed Sep 23 05-11-49 2026` |
| IC4 | Landed | 4.089 | 11.347 | False | False | 6.2 | `Wed Sep 23 05-13-34 2026` |
| IC5 | Landed | 7.595 | 12.418 | False | False | 5.3 | `Wed Sep 23 05-15-17 2026` |
| Static | Landed | 6.930 | 10.098 | False | False | 5.6 | `Wed Sep 23 05-16-53 2026` |
| Linear | Landed | 3.517 | 18.542 | False | False | 5.6 | `Wed Sep 23 05-18-35 2026` |
| Sinusoidal | Landed | 7.007 | 9.230 | False | False | 5.1 | `Wed Sep 23 05-20-17 2026` |
| Circular | Landed | 3.629 | 7.951 | False | False | 5.0 | `Wed Sep 23 05-21-58 2026` |
| Lissajous | Landed | 12.195 | 14.976 | False | False | 9.9 | `Wed Sep 23 05-23-42 2026` |

**Result: 10/10 landed, 0/10 precise+soft.** This baseline reaches the ground on every
case (unlike lin2022/lin2023/cho2022, which stall out on several), but lands very hard
and off-target — xy_err ranges 2.6–12.2 m, rel_vel up to 18.5 m/s. Note the short flight
durations (5–10 s on most cases, vs ~15–45 s for the other baselines) — this controller
descends fast and reaches the touchdown-detection threshold quickly rather than
approaching gradually; that speed is itself part of why it lands so hard/imprecisely.

Notes:
- None of these montages needed regeneration for the touchdown-detection bug (all 10
  cases landed genuinely — `SoftPrecise.xy_err` is populated in every rep — so the
  original argmin-based touchdown trim was already correct here). See
  `Memory/px4/feedback_montage_touchdown_argmin_bug.md` for what that bug was and why
  it doesn't affect this baseline.
- All 10 cases use the same GT-FB rover-world cross-marker recipe as `VISTA-GT/`
  (`../VISTA-GT/`), directly comparable at the same 0.10 m / 0.2 m/s thresholds.

**Platform-landing audit (2026-09-23, added after checking the geometry):** the landing deck is
a 0.6 x 0.6 m box (`rover_cross/model.sdf`, top at z=+0.50 m), so a centre >~0.3 m from the
target with `alt_above_surface_end` well below 0 is a *ground impact beside the platform*, not
a deck landing. The harness's `landed` flag (PX4 LandedState / accelerometer spike) does not
distinguish these. Read "landed" in the table above as "reached a physical impact", not "landed
on the target". This baseline: 0 on the deck; all 10 'landed' cases are off-platform impacts (xy 2.65-12.2 m).

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
