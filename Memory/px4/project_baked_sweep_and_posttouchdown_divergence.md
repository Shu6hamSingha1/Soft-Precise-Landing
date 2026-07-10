---
name: project_baked_sweep_and_posttouchdown_divergence
description: "2026-07-09 baked-config IC1-5 n=5 sweep results + failure-mechanism taxonomy - IC3 SOLVED-class (0.248), IC1 REGRESSED via a NEW dominant post-touchdown attitude-divergence mechanism (2/2 reps, roll or pitch escalates AFTER clean touchdown -> oblique view -> yaw-decode corruption -> permanent marker loss); IC5 = known short-runway canary."
metadata: 
  node_type: memory
  type: project
  originSessionId: 32b3f473-5219-48ce-b070-323ef9cc309a
---

**Config: perception-ON, defaults after the 2026-07-09 bakes** (FLOW_FUSE_RING=0, MARKER_KLT_RELAX_GATE=1, s-extrap self-ref + real-buffer-ordering fixes; see [[feedback_ring_fusion_marker_overlap]], [[feedback_klt_relax_gate_parallelogram]], [[feedback_s_extrap_realbuf_ordering]]). Bundle `test_data/ICValidation/20260709-074333`; pre-bake partials 070441 (IC1), 071548 (IC2), 072143 (IC3).

**Full baked sweep (n=5 each):**
| IC | soft | prec | mean xy | max xy | note |
|---|---|---|---|---|---|
| IC1 | 1 | 2 | 2.344 | 7.951 | REGRESSED (was 0.620 pre-ring-bake) — see mechanism below |
| IC2 | 0 | 0 | 0.728 | 1.827 | flat-ish (was 0.526) |
| IC3 | 4 | 3 | **0.248** | 0.872 | DRAMATIC fix (was 12.25 incl a 59m κ-ratchet fly-away) |
| IC4 | 1 | 3 | **0.247** | 0.615 | good |
| IC5 | 0 | 0 | 2.187 | 2.938 | flights 4.9–6.4s = the documented short-runway canary, pre-existing |

**NEW DOMINANT IC1 MECHANISM — post-touchdown attitude divergence (confirmed 2/2: rep1 roll, rep3 pitch):** clean, precise touchdown (min_alt_xy 0.117/0.047) → drone sits ~2cm for seconds → an attitude axis escalates (≈1°→7.5°+ over ~0.5–7s; axis varies) → view goes oblique → DISCONTINUOUS yaw-feature corruption (α jumps −25°→+105° / 43°→117°→150° in single frames; corner-shift non-rigid) → tracking degrades past the 3/4 rescue floor → PERMANENT marker loss (152–327 frames to end) → 3.3–8.0m endpoints. It is a POST-touchdown loop instability (armed drone still closing the loop on the deck), same family as the loom-kick/κ-ratchet oscillations but able to escalate to total divergence. The KLT relax gate fired correctly and did NOT contribute (corruption predated it). OPEN question: whether FLOW_FUSE_RING=0 made this escalation MORE likely (lost incidental damping) or it's run variance — needs a paired A/B, unattributable at n=2. OPEN lever: why no landed-state commit/disarm engages once genuinely down.

**Session failure-mechanism taxonomy (each traced to evidence, distinct roots):**
1. **Ring-fusion loom spike** → lateral kick via the loom×flow cross term — FIXED by FLOW_FUSE_RING=0 ([[feedback_ring_fusion_marker_overlap]]).
2. **κ-ratchet** (IC3_rep1 59m, pre-bake): sustained off-center σ saturation → adaptive switching term θ·sat(σ)·κ explodes ([5.6,30.1]) while reach/zeta_r stay small; coincides with extent collapse. Matches [[feedback_kappa_4axis_hexy_param_map]] ratchet. Largely absent post-bake (ring removal took away a σ driver).
3. **Post-touchdown attitude divergence** (above) — NEW, now the binding IC1 problem.
4. **Corner-correspondence decode anomaly** (IC2_rep2 pre-bake): single fresh decode with Δyaw=12.5°/frame (~313°/s implausible), one corner frozen while three shift in lockstep (non-rigid) → immediately precedes a 573-frame (5.7s!) never-recovering blackout the drone flies blind through; `getFailureCause()`=UNKNOWN (taxonomy gap — neither overflow nor drift_off fired). Raw frame unavailable (IMG_RECORD_RAW off).
5. **Terminal ~1Hz limit cycle** (IC1 pre-bake reps 2/3/5, multiple growing s_e_n peaks post-touchdown) = the documented lag×1/Z phase-margin cycle ([[project_residual_cycle_wumax_bake]], [[project_handoff_terminal_oscillation]]) — signal is TRUTHFUL (real position oscillates); do NOT reject/freeze it.

**Methodology notes that paid off:** honest min_alt_xy vs endpoint xy separates "never converged" from "converged then ballooned"; exact reach/switch/resid a_u decomposition is reconstructable from logged sigma/G/theta/kappa/a_v (G diagonal) without AU_DECOMP_DBG; always align Img/Ctrl/GT clocks via gt['Start Time'] (ctrl_t is already absolute = start-time-based; img_t same clock; gt['Time'] is 0-based).
