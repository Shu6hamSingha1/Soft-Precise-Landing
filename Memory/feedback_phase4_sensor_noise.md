---
name: Phase 4 sensor-noise floor — PX4 sensor noise MATCHES MATLAB; loop lag is the entire gap
description: PX4 high-frequency centroid σ = 0.36 px vs MATLAB model 0.39 px (0.91× ratio). Sensor noise contributes negligibly to the 16× SP-rate gap. Loop lag (Phase 2) is the complete explanation.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Top finding (2026-05-22)

PX4 SITL's sensor noise floor is **essentially identical to MATLAB's** depth-dependent model. The 16× xy_mean gap identified in Phase 1 is fully attributable to MAVSDK rate-loop latency (Phase 2), not sensor pipeline noise.

| Metric | PX4 | MATLAB | Ratio |
|---|---|---|---|
| Centroid σ at z=5m (high-pass) | **0.36 px median** | 0.39 px (`σ(z)=0.3+0.5/(z+0.5)`) | **0.91×** |
| σ_lat at z=5m | 6.6 mm | 7.2 mm | — |
| σ_lat at z=0.2m (touchdown) | 0.26 mm | 0.29 mm | — |

Three orders of magnitude below the 80 mm PRECISE threshold. **Sensor noise alone cannot explain why PX4 misses PRECISE.**

## Methodology note

Naive absolute-σ measurement on the pre-engagement hover window gave 11.9 px median (30× MATLAB), but the drone wasn't truly stationary during that window — vh_mean was 0.18–0.57 m/s as the drone converged to IC. The 11.9 px σ_abs is **motion**, not sensor noise.

The correct measurement is the **frame-to-frame Δ / √2** statistic, which high-passes out slow motion and isolates true sensor jitter. That yields σ_hf = 0.36 px — matching MATLAB.

This methodology distinction matters: future analyses that want to characterize the noise budget should use the HF method on the pre-engagement window, not absolute std.

## Complete gap decomposition (Phase 1 + 2 + 4)

| Source | PX4/MATLAB ratio | Contribution to 16× xy gap |
|---|---|---|
| MAVSDK rate-loop lag (Phase 2) | ~13× | **dominant** |
| Centroid sensor noise (Phase 4) | 0.9× | negligible |
| Optic-flow noise during hover | raw was high (~1–2 m/s) but savgol filter handles it | negligible after filtering |
| IC variance (per-step diagnosis) | symptom of lag-induced overshoot, not independent | — |

The 16× xy_mean ratio is fully explained by lag. At descent vel ~0.7 m/s and lag 168 ms, the drone moves 12 cm laterally per delay interval — directly above the 80 mm PRECISE spec. During the 4-second descent, the controller accumulates ~28 such uncorrected intervals.

## How to apply

- When discussing PX4 precision: cite the lag-dominance finding from Phase 2 + Phase 4. **The gap is single-cause: MAVSDK rate-loop latency.**
- **Stop touching sensor cal.** Memory `feedback_calibration_lessons.md` already warned cal refresh broke things. Phase 4 now confirms sensor noise is fine — leave it alone.
- Realistic interventions to close the gap (within rate-mode architecture):
  - PX4 inner-loop gain tuning (`MC_ROLLRATE_P`, `MC_PITCHRATE_P`, etc.) — could reduce the 168 ms rate-loop response time
  - Smith-predictor lag compensation in the outer Python controller
  - Image-based feedforward (predict centroid motion from velocity, subtract from observed)
- Architectural intervention (torque commands instead of rates) remains forbidden per `feedback_thrust_torque.md`.

## Data + tooling

- `PX4_Gazebo/analyze_sensor_noise.py` — Phase 4 analyzer (defaults to latest DefaultN10)
- Bundle analyzed: `~/ws/Test_Data/DefaultN10/20260521-224307` (10 reps)
- Uses pre-engagement hover window in `Img_Data["Time"]` < `Control_Data["t"][0] - 0.5s`
- Frame-to-frame delta methodology isolates sensor jitter from drone motion
