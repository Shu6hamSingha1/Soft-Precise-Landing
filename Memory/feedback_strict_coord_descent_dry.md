---
name: strict-coord-descent-converged-no-sp
description: "Strict-protocol per-axis coord descent (3/3 IC1 + IC2-5 gate, 47 axes log-uniform [1e-4, 1e4], 5 max passes) converged at pass 3 with 0 SP across ~414 reps. Definitive evidence gain-tuning cannot achieve repeatable IC1 SP under current SITL lag."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Result

`coord_descent_tune.py` strict-protocol run finished 2026-05-26 (bundle `20260526-112046`):

- 138 axis tests over 3 passes (converged early; max 5)
- ~414 IC1 reps; 0 reps achieved SP
- 0 candidates triggered the IC2-5 validation gate (would have required 3/3 IC1 first)
- Best composite: **0.175** (xy_min=0.143 m, vel=0.064 m/s) — close to SP boundary on individual reps but never repeatable
- Pass 3 accepted axes (composite-only, no SP): `E_Z=42.84`, `KR_YAW=4.163`, `KD_Y=0.4072` — do NOT merge to defaults

## Why this matters

This is the cleanest refutation of "broad gain tuning can find SP-reproducible IC1 config":
- 47 individual axes (not uniform vectors) — addresses earlier "lazy uniform scale" concern
- Log-uniform [1e-4, 1e4] — 8 orders of magnitude per axis
- Strict 3/3 SP at IC1 + IC2-5 ≥4/8 gate (eliminates lucky-IC artifacts)
- Coord descent semantics (re-tune all axes after any acceptance) — addresses parameter coupling

**Result: convergence with no SP.** The search space has no SP-repeatable basin reachable from current defaults under existing SITL latency.

## How to apply

- When user asks for "tune harder" / "search wider": cite this result. The search has been done strictly; the bottleneck is not gain choice.
- Real SP-reproducibility at IC1 requires architecture changes:
  - uXRCE-DDS rate-loop migration (Phase 2 finding: ~38 ms MAVSDK lag vs ~13 ms MATLAB)
  - Smith predictor / observer for the body-rate loop
  - Or accept that PX4 SITL at this latency floors SP-rate at ~vh0<0.05 m/s IC-luck regime
- Don't merge the 3 composite-accepted axes — they reduce composite by softening, not by precision improvement.

## Data

- `~/ws/Test_Data/CoordDescent/20260526-112046/` — bundle (state.json, log.jsonl, pass1-3/)
- Related: [[feedback_coord_descent_sp_lucky_ic]], [[feedback_phase2_loop_latency]], [[feedback_impulse_response]], [[feedback_phase5_sigma_divergence]]
