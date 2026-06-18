---
name: multisine-cal-ic25-collapse
description: Multisine cal (cal of record since 2026-06-02) collapses IC2-5 off-center landings to ~5-6m xy for ANY gain config; old-cal IC2-5 was 0.7-2m. Separate problem from the IC1 explosion.
metadata: 
  node_type: memory
  type: project
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**The multisine sensor cal regressed off-center (IC2-5) landings ~3-5×, independent of controller gains.**

Two-arm IC2-5 gate (2026-06-03, `ICValidation/20260603-000751` + `-001453`, n=8/arm):
- defaults (DH_D_MAX=50): xy mean **5.38 m**, vel 0.74 m/s
- DH_D_MAX=5 candidate:   xy mean **5.84 m**, vel 1.13 m/s (within variance of each other)
- historical old-cal IC2-5 baselines: **0.7–2 m**

**Why:** flight times are only 8–15 s — the drone reaches the ground before the lateral correction completes. The outer-PID input `s_e_n` is 1.85× hotter under the new cal (cal_s 0.58→1.10) and the measured flow scaling changed ~5-13× (cal_hw diag 0.07-0.18 → 0.61-0.86), so the controller's effective loop gains and descent behaviour no longer match the regime the gains were tuned in. The IC1 explosion fix ([[dsd-touchdown-spike]]) does not address this — at IC2-5 the drone lands too far from the marker for the 1/Z touchdown spike to even fire.

**Why this matters:** the cal is correct (GT-validated, kept per user decision); the controller must be re-tuned to it. IC1-only tuning will not surface this — any default change must still pass the IC2-5 gate, but the gate baseline itself is now ~5-6 m.

**Candidate fixes (untested as of 2026-06-03):**
1. Outer-PID rescale ×1/1.85 (`PLASMC_PID_SCALE=0.54`) — restores the designed loop gain (CONTROLLER_PARITY.md §6)
2. Slower descent (`LANDING_REF_RAD_OPT_FLOW` toward −0.2) — widens the lateral-convergence window
3. Descent gating on lateral error (MATLAB does this implicitly via funnel z-coupling)

**How to apply:** before any IC2-5 comparison, remember the baseline is now 5.38 m (new cal), not the historical 0.7–2 m. Don't interpret a candidate's ~5 m at IC2-5 as "the candidate broke it."
