---
name: input-cal-yaw-lag-anomaly
description: "PX4 yaw rate-loop lag ~275 ms (5-6× roll/pitch ~45 ms) is a PHYSICAL AUTHORITY/INERTIA FLOOR — RESOLVED 2026-06-03 by exhausting all three software levers: (1) CA_ROTOR_KM allocation 0.05→0.016 = no change; (2) telemetry artifact ruled out (GT quaternion-diff = 275 ms, real); (3) MC_YAWRATE_K rate-gain 1→2.5→4 = flat (285/255/265 ms). No software fix exists. Keep yaw detuned (0.2); it is correct design, not a workaround. Benign for stationary-target landing; would bind for moving/rotating targets."
metadata:
  type: feedback
  originSessionId: 367ac3aa-13f8-41ea-b23e-c929f5e1e50d
---

## The finding (2026-06-01)

`tools/aggregate_input_calibration.py` on n=8 input-cal recordings (sinusoidal body-rate cmd):

| Axis | Pearson r | xcorr lag | gain |
|------|-----------|-----------|------|
| ωx (roll)  | 0.947 | 63 ms | 1.160 |
| ωy (pitch) | 0.923 | 53 ms | 1.106 |
| ωz (yaw)   | **0.763** | **286 ms** | 0.877 |

## CA_ROTOR_KM hypothesis — TESTED AND FALSIFIED (2026-06-03)

I hypothesised the yaw lag came from a SITL allocation mismatch: Gazebo x500 rotors deliver
`momentConstant=0.016`, but the base airframe (`4001_gz_x500`) tells PX4's allocator `CA_ROTOR*_KM=0.05`
(3.1× too high) → yaw loop at 32% design gain → low bandwidth → 287 ms. Predicted: fixing KM → 90-110 ms.

**Measured it directly.** Set `CA_ROTOR*_KM 0.05→0.016` in airframe 4014, re-ran input-cal (n=2, routed to
`calibration_data/input_KMfix/`), same methodology:

| Axis | lag WITH KM fix | lag baseline | Δ |
|------|-----------------|--------------|---|
| ωx | 57 ms | 63 ms | unchanged |
| ωy | 48 ms | 53 ms | unchanged |
| ωz | **280 ms** | **286 ms** | **UNCHANGED** |

gain ωz = 0.876 (baseline 0.877) — also unchanged. **The KM fix does nothing to the lag.** Hypothesis dead.

**Why it was wrong (post-mortem):** the input-cal excitation is small-signal (0.1 rad/s, far from rotor
saturation). CA_ROTOR_KM scales the allocator's yaw column, which sets control-authority HEADROOM
(saturation margin), not the small-signal closed-loop bandwidth. With integral action and no saturation,
the rate loop drives commanded amplitude regardless of KM — so KM changes neither the gain (0.88, never 0.32)
nor the lag. My "32% torque → 3.1× bandwidth deficit" chain conflated open-loop static gain with closed-loop
bandwidth; they are not the same in an integrating loop. The gain≈0.88 I cited as "integrator recovering from
0.32" was never 0.32 to begin with.

**Cost of the error:** KM fix was applied then reverted; while applied it destabilised the validated landing
gains (LateralRestore attempt-3 c1 went soft→hard, vel 0.16→2.7 m/s) — pure downside, no lag benefit.
KM is REVERTED. Do not re-apply without a saturation-regime reason (it only matters near actuator limits).

## Root cause — STILL OPEN (candidates, untested)

1. **Telemetry/estimation path lag**: input-cal correlates COMMANDED ωz vs MAVSDK-reported ωz. If yaw rate is
   reported through a slower fusion path than gyro-direct roll/pitch, the 280 ms is partly a MEASUREMENT
   artifact, not control reality. Test: compare against Gazebo GT angular velocity (not MAVSDK telemetry).
2. **Yaw rate-setpoint / D-term filtering**: MC_YAWRATE_D=0 but the yaw setpoint may pass a slower filter.
3. **Inherent low yaw authority** (drag-reaction torque, 0.016 m equiv arm vs 0.174 m thrust arm; Izz 1.85×Ixx)
   → genuinely lower achievable bandwidth — but this would show as gain<1 too, and gain is 0.88, so this alone
   doesn't explain a 280 ms phase lag.

The OLD attribution in earlier versions of this memory (slower PX4 yaw gains; EKF heading drift) is also wrong:
MC_YAWRATE_P=0.2 is HIGHER than roll/pitch 0.15, and PX4 rate control feeds back gyro, not EKF heading.

## How to apply

- **Treat the 280 ms as real for control purposes** (yaw IS the slow axis), but its ROOT CAUSE is unknown —
  do not cite the KM/allocation story. The lag shapes gain choices (yaw detuned to 0.2, [[convergence-ordering]]).
- **Next diagnostic** (if yaw lag matters enough to chase): re-run input-cal logging Gazebo GT ω alongside
  MAVSDK telemetry; if GT ωz tracks the command at ~55 ms, the 280 ms is a telemetry artifact and the real
  yaw control is fine — which would mean OUR controller should consume GT/derived yaw rate, not MAVSDK ωz.
- **General lesson**: measure before claiming a root cause. The allocation argument was plausible and wrong.

## Cross-references

- [[feedback_impulse_response]] — pitch rate-loop 38 ms (single-impulse); sinusoidal xcorr finds 52 ms
- [[convergence-ordering]] — yaw detuned to 0.2 because of this lag
- [[compass-yaw-drift]] — EKF yaw drift affects OUR outer-loop yaw, not PX4's rate loop
- [[feedback_mc_rate_p_dead]] — MAVSDK runtime tuning of MC_*RATE_P breaks SITL preflight

## Aggregator location

`tools/aggregate_input_calibration.py` — input-cal-specific. Takes a dir arg (default calibration_data/input).


## RESOLVED 2026-06-03 — physical floor, all three levers exhausted

Third and final test: swept the yaw rate-loop gain MC_YAWRATE_K (the clean PID multiplier) via airframe-init
over {1.0, 2.5, 4.0} (of max 5.0), re-measured GT yaw lag (tools/analyze_gt_rate_lag.py), n=2/cell:

| MC_YAWRATE_K | yaw lag | roll | pitch |
|---|---|---|---|
| 1.0 | 285 ms | 45 | 40 |
| 2.5 | 255 ms | 50 | 40 |
| 4.0 | 265 ms | 50 | 40 |

**Flat within noise, non-monotonic — 4× gain does nothing.** Roll/pitch unchanged (knob touched only yaw = good
control). Combined with the earlier two falsifications, the yaw lag is now conclusively a PHYSICAL
AUTHORITY/INERTIA FLOOR, not any software-tunable quantity:
- NOT telemetry path (GT confirms 275 ms is real)
- NOT control allocation (CA_ROTOR_KM no effect)
- NOT rate-loop controller gain (MC_YAWRATE_K 4× no effect)

Physics: yaw torque is rotor drag-reaction (~0.016 m equivalent arm) vs roll/pitch thrust-differential
(0.174 m arm) → ~11× less torque per actuator effort; Izz (0.040) = 1.85× Ixx → net ~6× lower achievable
yaw bandwidth. The gain<1 (0.88) + τ≈0.30 s first-order signature is a genuine bandwidth limit, and a 4×
loop-gain increase can't push the physical plant faster → authority-limited, as expected. All quadrotors
have this asymmetry; it is not a SITL artifact.

**Implication for control (the reason this matters):** the yaw channel MUST be designed around ~275 ms lag.
Keeping yaw detuned (chi_alpha=gamma_alpha=0.2, [[convergence-ordering]]) is the CORRECT design, not a
workaround — per the MATLAB outer-delay envelope (MATLAB_Test_Record Test 2) a 287 ms yaw is past the
manuscript-gain cliff. For the STATIONARY ArUco target (drone starts ~1-2° yaw-aligned, no yaw slews
demanded) the slow yaw loop is BENIGN as long as yaw stays detuned. It would become binding for
MOVING/ROTATING targets (must track target heading) — future work, needs the w-estimation fix too.
Stop chasing yaw lag; it is understood and bounded. Redirect effort to the lateral channel (the
LateralRestore staircase: MATLAB predicts manuscript-lateral beats x0.35) and the terminal touchdown.