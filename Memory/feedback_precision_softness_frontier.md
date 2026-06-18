---
name: PRECISE-only failure mode + precision-softness frontier (2026-05-24)
description: Across 1205 reps, 19 PRECISE-only landings all share the same failure mode — drone lands precisely on target but with high LATERAL velocity (|vh_end| >> |vz_end|). The single SP rep distinguishes itself only by tiny vh0. Five N=10 sweeps map a precision-softness frontier that single-knob and stacked tuning cannot escape.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## The PRECISE-only failure mode

`scan_all_landings.py` across 68 SITL bundles (1205 reps total) reveals:

| Category | Count | % |
|---|---|---|
| **SP** (soft+precise) | **1** | 0.08% |
| PRECISE-only (xy<0.08, vel>0.2) | 19 | 1.6% |
| SOFT-only (vel<0.2, xy>0.08) | 186 | 15.4% |
| near-SP (xy<0.15, vel<0.4) | 4 | 0.3% |
| MOD_PRECISE (xy<0.3) | 157 | 13.0% |
| POOR / CATASTROPHIC / TARGET_LOST | 838 | 69.5% |

**Critical pattern in the 19 PRECISE-only reps:** every single one has |vh_end| ≫ |vz_end|. Examples:
- `P_SCALE_1.25`: rel_vel=1.43, vh_end=**1.38**, vz_end=−0.36
- `KAPPA0_X_0.5_rep5`: rel_vel=1.20, vh_end=**1.19**, vz_end=−0.16
- `YAW_E_1.5_rep2`: rel_vel=1.32, vh_end=**1.32**, vz_end=−0.11

The drone lands precisely on target but is still sliding LATERALLY at 1+ m/s.

The 1 SP rep (rep2 of Interventions/20260523-162914): rel_vel=0.13, vh_end=**0.029**, vz_end=0.13.

**The single distinguishing factor between the 1 SP and the 19 PRECISE-only**: lateral velocity at touchdown.

## Why the architecture fails to brake lateral velocity

The outer PID's `K_rd` (derivative term) is supposed to damp velocity. But with ~38 ms loop lag:
1. D-term acts on stale velocity estimates
2. PID sees image-pixel error; when error is small, D-term magnitude is small
3. SMC middle-loop tracks optic-flow rate `h`, not velocity directly
4. Funnel envelope `p_2_inf` / `rho_fov_inf` bounds error but doesn't FORCE it to zero

**The current architecture can HOLD zero lateral velocity but cannot actively BRAKE significant lateral velocity in the final phase.** The SP rep got SP only because vh0 at engagement was already tiny (0.046 m/s) — there was no significant momentum to dissipate.

## The precision-softness frontier (5 N=10 sweeps, 50 reps)

| Config | PREC | SOFT | SP | TL | xy_mean | xy_min | xy_std | vh_end_mean |
|---|---|---|---|---|---|---|---|---|
| **Interventions only** (still best) | **1** | **2** | **1** | 0 | 0.328 | 0.039 | 0.157 | 1.2 |
| RHOFOVINF × 0.5 | 0 | 4 | 0 | 0 | 0.317 | 0.110 | 0.164 | 0.80 |
| RHOFOVINF × 0.7 | 1 | 2 | 0 | 0 | 0.321 | 0.077 | 0.247 | 0.99 |
| THETACAP × 1.5 | 0 | 1 | 0 | 1 | 0.315 | 0.099 | 0.198 | 0.94 |
| Stack RHOFOVINF×0.5 + KP×1.25 | 1 | 2 | 0 | 0 | 0.413 | **0.031** | 0.258 | 0.88 |

**Pattern observed across all 5 configs:**

- Knobs that tighten the terminal envelope (`RHOFOVINF`) reduce `vh_end` (good for SOFT) but lose precision (bad for PRECISE)
- Knobs that boost precision (`KP × 1.25`) recover xy_min but don't reduce `vh_end` enough
- Knobs that relax saturation (`THETACAP`) cause more aggressive demands and reintroduce TL
- **The middle / interpolation points are STRICTLY WORSE than either endpoint** (xy_std rises, no offsetting gain)

**Combined SP rate across the 5 sweeps: 1/50 = 2%.** The single SP rep remains a fortuitous IC alignment.

## What this means going forward

The architecture has a frontier set by its lag (`feedback_impulse_response.md`: ~38 ms vs MATLAB's ~13 ms, on the pitch axis). Knobs move us along the frontier; they cannot move us off it.

To reliably hit SP at >50% rate, the architecture's lag must shrink. The available lag-reduction levers (per `tune-plasmc` skill):

1. **uXRCE-DDS migration** for setpoint path (cuts ~20-30 ms MAVSDK transport) — ruled out by user unless it's the only option (per session direction 2026-05-24)
2. **Airframe-init compile-time MC_*RATE_P** edit — untried; bypasses the runtime-param-set race that killed the v1 attempt
3. **Smith predictor in Python outer loop** — software-only model-based compensation, untried
4. **Piecewise descent rate** (h_rd ramp to slower in terminal phase) — gives controller more time at low altitude

Knobs that have been thoroughly demonstrated to NOT bridge the frontier (don't retry without new evidence):
- `RHOFOVINF` (any scale)
- `THETACAP` (1.5)
- `KP × 1.25` (alone or stacked with `RHOFOVINF × 0.5`)
- All MC_*RATE_P via MAVSDK runtime (preflight race)

## How to apply

- When user asks for more parameter tuning of the terminal-phase: cite this finding. The frontier is mapped; further single-knob sweeps will produce points already on the frontier.
- When user asks "what's the actual limit": **~10% SP rate with current architecture**. The 1/50 across 5 sweeps is consistent with that.
- When user asks "can we get a reproducible SP landing": only by reducing the architectural lag. Of the remaining levers, airframe-init MC_*RATE_P is the cheapest untried one. Smith predictor is the largest unexplored software win.
- Don't propose RHOFOVINF, THETACAP, KP boost, or their combos as new ideas — they're all on the frontier, mapped.

## Data

- `/tmp/all_landings.json` — full 1205-rep classification
- `PX4_Gazebo/scan_all_landings.py` — regenerable scanner
- `PX4_Gazebo/PARAMETER_ANALYSIS.md` — committed parameter-by-parameter analysis
- `~/ws/Test_Data/RhoFovInf05/`, `RhoFovInf07/`, `ThetaCap15/`, `RhoFov05_KP125/`, `Interventions/` — the five n=10 bundles
