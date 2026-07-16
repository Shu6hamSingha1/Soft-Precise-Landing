---
name: parameter-record-logging
description: "Log every PX4 tuning experiment into test_data/Landing_Test/parameter_record.ods (PX4_IC1_Tuning sheet) — the user's authoritative cross-phase tuning record"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

**Standing preference (2026-06-02):** the user keeps a hand-curated tuning log at `PX4_Gazebo/test_data/Landing_Test/parameter_record.ods` and wants **PX4 tuning runs logged into it** (their words: "Log PX4 runs into it").

**Why:** one authoritative record across MATLAB and PX4 phases, in the same config|metrics|remark format they used for the original MATLAB hand-tuning (Sheet1, Jul 2025, 30 experiments).

**How to apply:**
- Sheet1 = MATLAB-phase log (read-only history). Key lessons: X-axis was the unstable one ("a_x is oscillating unstable"); keep Omega_x LOW (0.05); raising p_inf_x INCREASES instability; kappa_0_x boost → very unstable.
- `PX4_IC1_Tuning` sheet = PX4 log (created 2026-06-02 with the 15-experiment per-axis gain session). Append new experiments as rows: Sl | Date | Config (env deltas from base) | xy_err | rel_vel | Class | Remark.
- Write with odfpy (installed in `~/ws/scripts/env2025`); ALWAYS back up the .ods to `Obsolete/` before editing.
- The .ods is git-tracked via a gitignore carve-out (`!PX4_Gazebo/test_data/Landing_Test/parameter_record.ods`) — commit after updating.

Related: [[dsd-touchdown-spike]] (the session logged there), [[feedback_sensitivity_sweep_methodology]].


**Sheet structure (current, 2026-06-03):**
- `PX4_Gain_Record` — PX4 SITL tuning trials (60-col per-axis layout, trials 1-46+)
- `MATLAB_Test_Record` — MATLAB simulation sweeps (delay-robustness etc.; one row per sweep x arm,
  per-delay SP columns). MATLAB results go HERE, never in PX4_Gain_Record (user directive 2026-06-03).
- `Removed_Parameters` — parameters/mechanisms removed from controller.py with when/why/replacement
