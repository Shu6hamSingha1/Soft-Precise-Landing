# test_data/Final/CHO2022-GT — comparison baseline: Cho et al. 2022 (FF-IBVS)

**RE-RECORDED 2026-09-23 (evening)** with the corrected `BASELINE_ZF=0.2` (MATLAB/Common/
Constants.m canonical; the first recording used a stale 0.3) AND the `B_T` saturation fix in
`src/baselines.py::accel_to_rate_thrust`. The old zf=0.3 set is superseded (git history
f8bcaf42/310c50c5). Code: main @ 1a0e6dd8. `PLASMC_GT_FEEDBACK=1 PLASMC_BASELINE=cho2022`
(`src/baselines.py::Cho2022`, feature-following IBVS with an adaptive-sigmoid altitude gain,
fed marker key points projected through the ideal pinhole), `WORLD=rover_cross[_deck]
ROVER_MODEL=rover_cross[_ackermann]`, `MARKER_TYPE=cross`, default params, ONE attempt per
case (no SoftPrecise retry gate; `BASELINES=cho2022 bash scripts/run_baseline_campaign.sh`).

Each `<case>/` holds `<case>_montage.mp4` (chase + onboard PiP + GT plot panel),
`<case>_onboard_cam.mp4`, `<case>_chase_cam.mp4`, and `dataset/` (Control_Data,
Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data). No
`overlay_s_alpha`/`overlay_h` PiPs for baselines (scope decision 2026-09-23).

| Case | Outcome | xy_err (m) | rel_vel (m/s) | precise | soft | dur (s) | source run |
|------|---------|-----------|---------------|---------|------|---------|------------|
| IC1 | **Aborted** (descent stall) | — | — | — | — | 31.7 | `Wed Sep 23 21-20-12 2026` |
| IC2 | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 21-24-06 2026` |
| IC3 | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 21-28-03 2026` |
| IC4 | **Aborted** (descent stall) | — | — | — | — | 34.9 | `Wed Sep 23 21-32-09 2026` |
| IC5 | **Aborted** (descent stall) | — | — | — | — | 29.7 | `Wed Sep 23 21-36-14 2026` |
| Static | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 21-40-02 2026` |
| Linear | Landed | 1.508 | 1.295 | False | False | 11.1 | `Wed Sep 23 21-43-31 2026` |
| Sinusoidal | **Aborted** (descent stall) | — | — | — | — | 31.8 | `Wed Sep 23 21-46-11 2026` |
| Circular | **Aborted** (descent stall) | — | — | — | — | 32.9 | `Wed Sep 23 21-50-16 2026` |
| Lissajous | **Aborted** (descent stall) | — | — | — | — | 31.9 | `Wed Sep 23 21-54-20 2026` |

**Result: 1/10 landed, 0/10 precise+soft.** "Aborted" = `RuntimeError: descent stall: no
>0.30 m descent in 25s -- hovering, aborting`. Same failure as the zf=0.3 recording (0/10):
the zf fix and the B_T saturation did not change it. The only landing is `Linear` (min_alt 0.023 m;
CORRECTED: it is NOT deck contact -- the drone drifted 1.5 m off, ascended (`descent_anomaly: ASCENDING`), then fell to the ground beside the platform with a 318 m/s^2 impact spike; see the audit below).

**Root cause (unchanged; see `Memory/px4/feedback_cho2022_never_lands_rootcause.md`):** the
FF-IBVS law is a pure feature-error regulator converging to a fixed depth setpoint; once
converged, `I_a` settles to exact hover (I_a_z -> -9.81) and the drone parks a few cm above the
platform, never making the physical contact PX4's `LandedState`/impact detector needs.
Baseline runs also bypass `self.PLASMC()`, so PLASMC's loom-inversion touchdown latch never
runs. Other baselines land by overshooting into hard contact; cho2022's smooth regulation
never does.

**Effect of the fixes (old vs new recording):** parked altitude shifted ~0.497 m -> ~0.52 m
(the zf change moves the regulation equilibrium slightly); `max|B_T|` stays bounded (3-11)
in every rep, confirming the saturation held (old recording had ~3 M excursions only in
the deeper-depth-target diagnostic rep, not in the original 10). No behavioural change to
the landing outcome.

Notes:
- Montages use the touchdown-detection fix in `tools/make_landing_montage.py` (aborted
  cases use the full untrimmed series); no 0-frame-PiP anomalies this run.
- Same GT-FB rover-world cross-marker recipe as `VISTA-GT/` (`../VISTA-GT/`).

**Platform-landing audit (2026-09-23, added after checking the geometry):** the landing deck is
a 0.6 x 0.6 m box (`rover_cross/model.sdf`, top at z=+0.50 m), so a centre >~0.3 m from the
target with `alt_above_surface_end` well below 0 is a *ground impact beside the platform*, not
a deck landing. The harness's `landed` flag (PX4 LandedState / accelerometer spike) does not
distinguish these. Read "landed" in the table above as "reached a physical impact", not "landed
on the target". This baseline: 0 on the deck; the single 'landed' case (Linear) is a ground impact 0.76 m below deck level, 1.51 m from the target.

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
