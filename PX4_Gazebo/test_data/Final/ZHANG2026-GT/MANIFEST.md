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
