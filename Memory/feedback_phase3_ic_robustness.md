---
name: Phase 3 IC robustness — MATLAB is fully robust to PX4-style IC perturbations
description: 15/15 SP in MATLAB at IC velocity perturbations up to 0.32 m/s (2× PX4 mean). The "multi-factor IC sensitivity" identified in PX4 is an effect of loop lag, not a separate cause.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Top finding (2026-05-22)

MATLAB achieves **15/15 soft+precise** with random IC velocity perturbations sampled from N(0, 0.15) m/s — perturbations up to **0.32 m/s**, which is larger than PX4's measured vh_max (0.26 m/s) and 2× PX4's vh_mean (0.089 m/s).

| Metric | MATLAB w/ IC noise | PX4 SITL |
|---|---|---|
| vh at engagement (mean / max) | 0.174 / 0.321 m/s | 0.089 / 0.255 m/s |
| xy_mean | **0.034 m** | 0.484 m |
| xy_max | 0.070 m | 1.039 m |
| SP rate | **15/15 (100%)** | 0/30 (0%) |
| ρ(vh_inj, xy_err) | +0.18 (weak) | +0.78 (strong) |

The correlation ρ(vh, xy) is weak in MATLAB (+0.18) but strong in PX4 (+0.78). The same IC perturbation produces very different outcomes in the two pipelines.

## Implication: revises the IC-sensitivity diagnosis

`feedback_instability_mechanism.md` correctly observed PX4's multi-factor IC sensitivity (σ0/vh0 binding migrates as constraints tighten) but interpreted it as a structural property of the SMC-driven descent. Phase 3 shows that's wrong: **MATLAB's identical SMC-driven descent has no such IC sensitivity when given 2× the perturbation.**

The mechanism is therefore: PX4's 168 ms rate-loop lag amplifies small IC perturbations into uncorrected lateral drift over the ~4 s descent. The σ0/vh0 correlations are downstream effects of lag, not independent causes.

## Closed-form diagnosis (Phase 1 + 2 + 3 + 4)

| Phase | Finding | Verdict |
|---|---|---|
| 1 | MATLAB hits 10/10 SP at PX4-aligned IC | Controller design is correct |
| 2 | PX4 loop lag 168 ms vs MATLAB 13 ms | Lag is 13× larger |
| 3 | MATLAB robust to 2× PX4-style IC perturbation | IC sensitivity is lag-amplified, not intrinsic |
| 4 | PX4 sensor σ_hf = 0.36 px vs MATLAB 0.39 px | Sensor noise matches; not a gap source |

**Single-cause conclusion**: MAVSDK rate-loop lag (~155 ms above MATLAB baseline) is the entire 16× xy_mean gap. All other observed PX4 phenomena (IC sensitivity, sensor cal residuals, gain unresponsiveness) are downstream effects of lag.

## Realistic interventions

Within the rate-mode MAVSDK architecture (torque commands rejected per `feedback_thrust_torque.md`):

1. **PX4 inner-loop tuning** — `MC_ROLLRATE_P`, `MC_PITCHRATE_P`, `MC_YAWRATE_P`. Default x500 gains were tuned for stability on real hardware, not minimum lag. Tightening could reduce the rate-loop time constant by 30-50%.
2. **Smith predictor in Python** — track the lag-compensated image error; controller acts on the predicted state, not the stale measurement. Theoretical perfect compensation, practically limited by model-vs-real discrepancy.
3. **Image-based velocity feedforward** — measure `dxy/dt` from sequential centroid frames, add as feedforward term to outer PID. Bypasses some of the lag.
4. **Accept the ceiling** — document SITL has ~155 ms of MAVSDK overhead, note in paper as "real-hardware would have lower lag due to direct PX4 firmware integration."

Recommend (1) first — cheapest test, no controller architecture changes, can be done via `set_param` calls.

## Data + tooling

- `MATLAB/Multi_init_cond/phase3_ic_sensitivity.m` — 15-run sweep with N(0, 0.15) m/s velocity perturbations
- `MATLAB/Datasets/Phase3/phase3_summary.mat` — results
- `MATLAB/Multi_init_cond/InitVar.m` — extended to honor `IC_VEL_OVERRIDE` global
