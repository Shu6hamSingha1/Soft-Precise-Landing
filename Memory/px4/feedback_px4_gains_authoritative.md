---
name: feedback_px4_gains_authoritative
description: "USER DIRECTIVE 2026-07-02: MATLAB gains are OUTDATED; the PX4 baked gains are the correct/authoritative set to work with. Do NOT re-sync/port the new MATLAB vdf_params (locked-kappa-engagement bake) into PX4 — despite the sync-gains skill and the historical MATLAB->PX4 auto-align direction."
metadata:
  node_type: memory
  type: feedback
---

**USER DIRECTIVE (2026-07-02): "MATLAB gains are outdated. PX4 gains are correct to
work with."**

**Why:** stated right after the 2026-07-02 pull brought in MATLAB's new
`vdf_params` bake ([[project_locked_kappa_engagement]]: kappa0 .05 / N .10 /
Pleak [.5;.5;1.5] / E .5 / Xi_r 0.3 / p_rinf 0.85 / theta_per_axis) and the new
`sync-gains` skill. The PX4 gain set evolved through the GT-FB campaign
(Z_REG=0.2, W_U_MAX=2.0, VDS_KF_Q=1, GAMMA 0.25, PR0=10/XIR=0.10, kappa0_xy 0.5,
N_xy 0.1, ...) and is validated on the SITL plant (38 ms lateral / 287 ms yaw
lag); the MATLAB set is tuned for the lag-free MATLAB plant.

**How to apply:**
- Do NOT port/auto-align the new MATLAB vdf_params into PX4's combined-barrier
  gains; PX4's baked defaults in controller.py stand.
- Reinforces [[feedback_matlab_gains_not_portable]] (chi_r=2.0 catastrophe) —
  now as a standing direction-of-authority, not just a porting caveat.
- If the `sync-gains` skill is invoked for PX4, the sync direction is at most
  PX4→documentation, never MATLAB→PX4, unless the user explicitly reverses this.
- MATLAB findings still transfer as MECHANISM/UNDERSTANDING (e.g. the 2π alpha,
  the yaw-orbit ceiling, prescribed-rate h_d designs) — just not gain VALUES.
