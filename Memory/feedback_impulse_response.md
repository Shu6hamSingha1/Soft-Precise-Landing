---
name: Impulse-response rate-loop test — revises Phase 2 lag down to ~40 ms
description: Dedicated hover-and-step test gives pitch axis 38 ms total lag (30 ms MAVSDK + 8 ms PX4). PX4/MATLAB lag ratio is ~6× not 13×; MAVSDK transport is now the dominant single component.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

> ⚠ **UPDATE 2026-07-03:** the 38 ms figure stands as the plant characteristic, but the 'uXRCE-DDS is the lag lever' framing is closed — the DDS low-latency path was ⛔ RULED OUT by user 2026-07-02 (lag accepted as a design constant).
## Top finding (2026-05-22)

The Phase 2 cross-correlation estimate of ~168 ms loop lag was an artifact of low cross-correlation coefficients (0.05–0.20) in convergent-flight data. A dedicated impulse-response test (hover + step in body-rate command) gives a sharper measurement: **38 ± 4 ms total rate-loop lag** on the pitch axis.

| Component | Time | Notes |
|---|---|---|
| MAVSDK transport (deadtime) | 30 ± 2.5 ms | gRPC → mavsdk_server → UDP → mavlink_app → uORB |
| PX4 rate-controller rise (τ) | 8 ± 2 ms | `mc_rate_control` is already fast |
| **Total rate-loop lag** | **38 ± 4 ms** | clean, repeatable across 3 pitch impulses |

Roll and yaw data was noisy — the ±0.5 rad/s impulses triggered the impact detector. Pitch was clean enough that the characterization is solid. Future runs should use smaller magnitudes (~0.2 rad/s) for roll/yaw if needed.

## Implication: intervention ranking flips

| Lever | Old estimate (Phase 2) | New estimate (impulse) |
|---|---|---|
| MC_*RATE_P tuning | 30–80 ms reduction | **~5–10 ms** (τ already short) |
| uXRCE-DDS migration | 10–30 ms reduction | **~20–30 ms** (now the biggest single component) |
| Smaller savgol WIN=5 | ~12 ms | ~12 ms (unchanged) |
| Telemetry rate 60→200 Hz | improves diagnostics only | confirmed not a control-path lever |

**uXRCE-DDS migration is now the highest-value single intervention** — opposite of the earlier ranking that put MC_*RATE_P first. The PX4 rate controller doesn't need tuning; the MAVSDK transport is where the deadtime lives.

## Implication: lag doesn't fully explain the 16× gap

- PX4 control-loop lag: ~80 ms (rate-loop 38 ms + image pipeline 41 ms)
- MATLAB control-loop lag: ~13 ms
- Lag ratio: ~6×
- xy_mean ratio: 16×

Lag accounts for *some* but not all of the 16× gap. Other factors that may contribute:
- SMC trajectory dynamics may differ when commanded rates execute with 30 ms deadtime vs instantly (chattering, integral windup)
- PX4 SITL physics is lockstep with Gazebo; numerical integration may differ from MATLAB's RK5
- Air-frame mass/inertia/motor model in Gazebo SDF vs MATLAB Constants.m may have small mismatches

A future SMC-trajectory comparison (MATLAB σ(t) vs PX4 σ(t) at matched timestamps) would isolate this.

## How to apply

- When discussing the lag-reduction intervention order: cite this revised estimate. **uXRCE-DDS first, then MC_*RATE_P if any residual gap.** The earlier (Phase 2-era) ranking is superseded.
- When estimating control-loop lag: prefer the impulse-response number (38 ms pitch) over the Phase 2 cross-correlation number (168 ms). The cross-correlation was noisy.
- The Phase 1 controller-correctness finding still holds: MATLAB controller is sound, PX4 plant adds the variance.
- The original Phase 2 + Phase 4 conclusion ("lag is the gap source") is **partially correct** but oversimplified — lag explains ~6× of the 16× outcome ratio, not all of it. The remaining factor likely lives in SMC trajectory dynamics under deadtime.

## Data + tooling

- `PX4_Gazebo/impulse_response.py` — test script (takeoff + 9-impulse sequence + land)
- `PX4_Gazebo/run_impulse_response.sh` — launcher (wraps `run_aruco_landing.sh` via `PY_SCRIPT=impulse_response.py`)
- `PX4_Gazebo/analyze_impulse_response.py` — first-order fit + per-axis summary
- `PX4_Gazebo/run_aruco_landing.sh` — now honors `PY_SCRIPT` env to swap entry points
- `~/ws/Test_Data/ImpulseResponse/20260522-113300/impulse_log.npy` — first clean dataset
