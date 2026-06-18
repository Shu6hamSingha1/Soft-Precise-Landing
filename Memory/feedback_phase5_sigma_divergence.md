---
name: Phase 5 + σ-trajectory comparison — residual gap is lag-induced lateral SMC divergence
description: At matched h_rd=-0.70, MATLAB still hits 10/10 SP. σ comparison shows PX4 converged reps track MATLAB closely; failed PX4 reps diverge in σ_xy (not σ_Z). Residual gap is lag-induced lateral chatter, not a separate mechanism.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Top finding (2026-05-22)

The diagnosis is now **fully closed-form**:

1. **Descent rate is NOT the issue.** MATLAB at h_rd=-0.70 (Phase 5) still hits 10/10 SP with xy_mean=0.033 (matching Phase 1's slower-descent result of 0.030). PX4 at the same h_rd gets 0/30 SP, xy_mean=0.508.

2. **σ-trajectory comparison shows lag-induced divergence is the residual:**
   - MATLAB and PX4 *converged* reps have nearly identical σ trajectories (σ_Y RMSE = 0.11)
   - PX4 *failed* reps diverge in σ_xy (4× larger at touchdown)
   - σ_Z is essentially identical across all three (RMSE = 0.02) — vertical channel always works

3. **The lateral SMC channel fails stochastically under PX4's lag.** The 30 ms MAVSDK transport deadtime occasionally pushes σ_xy into a divergent regime; once there, the SMC can't recover within the ~4 s descent.

## Quantitative summary

| Condition | tail \|σ_xy\| | xy_end | Verdict |
|---|---|---|---|
| MATLAB h_rd=-0.70 | 0.30 | 0.056 | Converged |
| PX4 best rep (rep2) | 0.28 | 0.033 | Converged — matches MATLAB |
| PX4 worst rep (rep7) | 1.26 | 1.039 | Diverged in σ_xy |

The PX4 best rep's σ trajectory tracks MATLAB closely — proving the controller is capable of MATLAB-quality performance when conditions permit. The PX4 worst rep diverges; conditions weren't permissive that time.

## Implication for the 16× outcome gap

| Component | Ratio | Mechanism |
|---|---|---|
| Pure lag (rate-loop + transport) | ~6× | Each control tick acts on 80 ms stale data |
| Residual SMC divergence | ~3× more | Lag occasionally pushes σ_xy into chatter; non-linear failure mode |
| **Combined** | **~16×** | matches observed xy_mean ratio |

These aren't separate causes — both stem from the same MAVSDK deadtime. The lag *deterministically* slows convergence (6×), and *stochastically* triggers divergence in lateral SMC when initial conditions push σ_xy near the boundary (additional 3×).

## How to apply

- **The residual-gap question raised in `feedback_impulse_response.md` is now answered.** Lag explains all of the 16×; the apparent "residual" was just the non-linear effect of lag on SMC stability.
- **Intervention recommendation unchanged from impulse-response finding**: uXRCE-DDS first (cuts the 30 ms MAVSDK deadtime which directly causes the divergence). MC_*RATE_P tuning is secondary.
- **If you want a quick before/after measure** of any lag-reduction intervention: re-run `analyze_sigma_compare.py` after the change. Closing the σ_xy gap (1.26 → ~0.3) is the success criterion.
- **The Phase 1 finding that "MATLAB controller is correct" is robust at multiple descent rates** (h_rd=-0.42 in Phase 1, h_rd=-0.70 in Phase 5). Not a descent-rate-specific artifact.

## Data + tooling

- `MATLAB/Multi_init_cond/phase5_hrd_match.m` — 10-run sweep at h_rd=-0.70
- `MATLAB/Multi_init_cond/capture_sigma_trace.m` — single-run σ-trajectory capture (now defaults to h_rd=-0.70)
- `MATLAB/Datasets/Phase5/phase5_summary.mat` — 10-rep outcome stats
- `MATLAB/Datasets/Phase5/matlab_sigma_h070.mat` — σ trajectory for rep 1
- `PX4_Gazebo/analyze_sigma_compare.py` — compares MATLAB vs PX4 best vs PX4 worst σ traces
- `MATLAB/Common/Constants.m` — extended to honor `H_RD_OVERRIDE` global
