---
name: input-cal-thrust-units
description: "Controller B_T is in Newtons; convert_2_sys_cmd's 0.738/45 empirical mapping produces near-physical FC response (TEL/(B_T/m) gain = 0.94 ± 0.05). Plotter cmd→a_z should use T_u/mass (physical), NOT the open-loop derivation g·T_u/(45·0.738)"
metadata:
  type: feedback
  originSessionId: 367ac3aa-13f8-41ea-b23e-c929f5e1e50d
---

## The chain (verified 2026-06-01)

```
  controller.py:850                          convert_2_sys_cmd                MAVSDK
  B_T = m·(I_a[2]+g)/(cos·cos)  ───────►   0.738 - B_T/42.3   ───────►   set_attitude_rate
  B_T = 0 at hover (Newtons)               thrust_norm in [0,1]            (offboard rate mode)
```

The `0.738` constant approximates the drone's true hover thrust_norm. The slope was **tightened from 1/45 to 1/42.3** on 2026-06-01 so that 1 N of B_T produces exactly 1/mass m/s² of body-z accel (Newton's law). Empirical gain on n=7 input-cal runs: 0.937 (old) → ~1.00 (new, projected). The 6% change is below the controller's adaptive integrator authority — PLASMC tuning should absorb it without needing re-tune, but validate via a fresh input cal + landing test before assuming.

## Empirical verification (n=7 input-cal runs, ωx/ωy/ωz/thrust sinusoidal sweep)

Comparing `tel_kin_z = a_t[:, 2] + g` against two candidate `a_z_cmd` formulas:

| formula | gain (tel/cmd) | r | interpretation |
|---|---|---|---|
| `a_z_cmd = T_u / mass`               (physical Newton's law) | **0.937** | 0.995 | ✓ matches within 6% |
| `a_z_cmd = g·T_u / (45·0.738)`       (open-loop from formula) | 1.586 | 0.995 | ✗ 59% off |

Best lag = **36 ms** (matches `feedback_impulse_response` pitch rate-loop = 38 ms). The 6% magnitude gap is consistent with cos(tilt) losses, drag, and PX4 control smoothing.

## The plotter trap I almost fell into

Pre-2026-06-01, the input-cal plot showed `-T_u/mass` (sign-flipped). I "fixed" it to `g·T_u/(45·0.738) = 0.295·T_u` — algebraically derived as the open-loop prediction "thrust_norm gets reduced by `cmd[3]/45`, so a_z = g · (1 - thrust_norm/0.738)". That formula is the prediction *if* 0.738 is the true hover. But the measured 1.59× gain showed the open-loop derivation is wrong — the FC's actual closed-loop response per cmd[3] unit is ~1/m m/s² (Newton's law applied to the *physical* mapping that 0.738/45 was empirically tuned to produce).

Correct plotter formula (notebooks/plotter_input_calibration.ipynb cell 27):
```python
a_z_cmd = T_u / mass
ax.plot(t_g, a_z_cmd, label='Cmd → expected a_z (T/m)', linestyle='--')
```

Sign: positive cmd[3] → less thrust → drone falls → +a_z in FRD-down. (No leading minus, no g, no 0.738/45 factors — those cancel because the empirical pair was tuned to make B_T → a_z behave like 1/m.)

## When this matters

- **Plotter / analysis:** always use `T_u / mass` to interpret B_T as expected accel. NOT `B_T/(45·0.738/g)` (= my flawed open-loop derivation). NOT `B_T / (m·g/0.738)` (= max-thrust normalization).
- **FC formula:** `convert_2_sys_cmd` itself is calibrated and shouldn't be changed without re-tuning the controller. The 0.738 and 1/45 are a tuned pair; touching either invalidates the PLASMC gain set.
- **Across drones:** if drone mass changes or motor capabilities differ, the (0.738, 1/45) pair stops approximating 1/m. Re-tune by:
  1. Hover the drone in offboard rate mode with cmd[3]=0; measure altitude drift over 10s.
  2. If drone holds altitude → 0.738 is still correct hover. If it climbs/falls → adjust.
  3. Send a cmd[3]=+X step; measure a_z response. Compute gain = a_z/X.
  4. If gain ≈ 1/m → 1/45 slope is correct. If different → adjust 1/45 to (1/45) × actual_gain / (1/m).

## Why the empirical 1/45 isn't 1/MAX_THRUST_N

If 0.738 were the *true* hover and physics applied directly: 1 N of B_T should reduce thrust_norm by 1/(m·g/0.738) = 0.0376, NOT 1/45 = 0.0222. The mismatch (0.0222 vs 0.0376, ratio 0.59) is exactly inverted from the closed-loop FC's empirical boost (~1.59× the formula prediction). So the 1/45 slope was tuned to *compensate* for PX4's offboard-rate-mode thrust behavior — whatever PX4 actually does with the thrust setpoint in this mode, the (0.738, 1/45) pair makes B_T ≈ physical Newtons in the closed loop.

## Cross-references

- [[input-cal-yaw-lag-anomaly]] — ωz tracking lag 287 ms vs ωx/ωy 52-61 ms
- [[impulse-response]] — pitch rate-loop = 38 ms (single-impulse method)
- `apps/record_input_calibration.py:48` — the convert function (input cal)
- `apps/landing_test.py:72` — same convert function (landing path)
- `src/controller.py:850` — `B_T` formula (Newtons, with +g correction)
- `notebooks/plotter_input_calibration.ipynb` cell 27 — `a_z_cmd = T_u / mass`
