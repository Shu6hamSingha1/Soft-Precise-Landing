---
name: Phase 2 loop-latency budget — MAVSDK rate-loop lag is the 16× gap
description: PX4 loop delay ~168ms vs MATLAB ~13ms (13× ratio), matching the 16× xy_mean gap. MAVSDK→PX4 rate-controller round-trip dominates; image pipeline is fast (~41ms total).
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Top finding (2026-05-22)

The 16× PX4 vs MATLAB xy_mean gap identified in Phase 1 is dominated by **MAVSDK rate-loop latency**, not by sensor noise or pipeline jitter.

| Stage | Median delay | Notes |
|---|---|---|
| Camera → ROS bridge → centroid | ~6 ms | image age at controller (median) |
| Savgol group delay (WIN=7) | ~35 ms | (WIN−1)/2 / 86 fps |
| **Image pipeline subtotal** | **~41 ms** | fast, well-characterized |
| MAVSDK → PX4 → actuator → measured ω | **127–262 ms / axis** | cross-correlation peak from commanded `w_u` to `Angular Velocity Body`; std 100–130 ms |
| **Total loop delay (median)** | **~168 ms** | range [−8, 341] |

MATLAB reference: dt=10 ms controller + ZOH=3 (30 Hz image) ≈ 13 ms effective.
PX4 / MATLAB ratio: **~13×**, matching the 16× xy_mean ratio (Phase 1).

## Why this is the dominant gap

Soft+precise target is xy ≤ 0.08 m at descent vel ~0.7 m/s → 114 ms terminal time-constant. A 168 ms loop delay means the controller acts on image error that is now ~100 ms stale; lateral position has moved 7 cm in that interval, larger than the precision spec. No amount of gain tuning fixes this — the response is fundamentally phase-shifted.

## Caveats on methodology

- Cross-correlation peak coefficients are low (0.05–0.23). The lag-at-peak is noisier than the median-across-reps suggests.
- Y-axis median 262 ms may be hitting the 300 ms search-window ceiling.
- The order-of-magnitude finding (10s vs 100s of ms) is solid; the exact number deserves a cleaner measurement (e.g., chirp-injection test).
- The low correlation also implies the rate-loop tracking quality itself is poor on certain axes — possibly saturation, disturbance, or coarse cross-coupling.

## How to apply

- When discussing PX4 precision: cite this lag finding. **The bottleneck is MAVSDK rate-loop response, not gain tuning.**
- Sensor noise (Phase 4, not yet run) and IC variance (`feedback_instability_mechanism.md`) are real but secondary — they'd matter less if lag weren't already 13×.
- The longstanding "thrust+torque refactor" question (forbidden per `feedback_thrust_torque.md`) is the architectural intervention that would actually move this. **Still forbidden** — user has rejected it explicitly. Within the rate-mode architecture, the realistic options are (a) reduce loop lag by tuning PX4's internal MC_ROLLRATE_P / MC_PITCHRATE_P gains, (b) use predictive image-error compensation (Smith predictor), or (c) accept the SITL ceiling and note it in the paper.
- For Phase 3 / Phase 4 planning: Phase 4 (sensor noise) is still worth running, but expectation is now it'll contribute ≤ 30% of the gap, not the majority.

## Data + tooling

- `PX4_Gazebo/analyze_loop_latency.py` — bundle-dir analyzer (defaults to latest DefaultN10)
- Bundle analyzed: `~/ws/Test_Data/DefaultN10/20260521-224307` (10 reps)
- Cross-correlates `Control_Data['w_u(t)']` vs `Telemetry_Data['Angular Velocity Body']` on a 5 ms grid, lag search ±50 to +300 ms
