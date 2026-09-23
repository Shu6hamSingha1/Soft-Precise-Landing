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
the zf fix and the B_T saturation did not change it. The only landing is `Linear` (deck
heave carries the platform up into the hovering drone, min_alt 0.023 m) -- incidental
contact, not a controlled touchdown.

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
