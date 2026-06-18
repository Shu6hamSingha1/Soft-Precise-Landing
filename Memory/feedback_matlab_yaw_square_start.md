---
name: feedback_matlab_yaw_square_start
description: "MATLAB (the canonical PLASMC reference) ALWAYS spawns square: q_c=identity (yaw=0, alpha~0) in BOTH InitVar.m and InitVar_loop.m (Monte-Carlo). It NEVER perturbs initial yaw (only position/velocity). So the yaw control law is validated ONLY for the square-start / small-alpha-hold regime — never for a large initial yaw. MATLAB also has NO psi_d rate clamp (u_a_sat=u_a). PX4 starts ~100deg (random Gazebo spawn + IC doesn't square it) and clamps psi_d to 0.7*W_U_MAX=40deg/s."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

**Checked the MATLAB yaw control (2026-06-08) to ground the PX4 yaw tuning.** Findings:

- **MATLAB always starts square.** `Multi_init_cond/InitVar.m:29` and `InitVar_loop.m:19` both hard-code `q_c=[1,0,0,0]` (identity → yaw=0); `psi_d=yaw_init=0`. The Monte-Carlo / IC generators perturb **position and velocity only — never the initial yaw**. So every MATLAB run (nominal AND MC) begins at alpha≈0, and the yaw ASMC only ever does small alpha-holds. **There is NO MATLAB evidence the control law converges a large (e.g. 100°) initial yaw — it was never exercised for it.**

- **MATLAB does NOT clamp psi_d.** `visualControl_IBVS_adaptive.m:500`: `u_a_sat = u_a;  % no cap; pass ASMC rate directly to heading integrator`, then `psi_d = psi_d + u_a_sat*dt`. The gains (`Gamma_a=0.5, Omega_a=0.5, n_a=1.0, p_a=2, kappa_a_0=2.0, E_a=3.0`) are "slowed to match ~0.8 rad/s natural slew for |e_a|=π/2" (line 67) — i.e. the *gains*, not a clamp, set the slew.

- **PX4 diverges in two ways:** (1) it starts at ~**100° yaw** (random Gazebo spawn; the slow IC servo `dmax=0.3` doesn't square it within budget — confirmed: GT yaw=100° at the first control sample, e_a up to 116°); (2) it **clamps psi_d** to `PLASMC_YAW_PSID_RATE(0.7)·W_U_MAX(1.0)=40°/s` (`controller.py:774-776`) — a pure lag-mitigation (prevents psi_d racing ahead of the achievable body yaw rate → e_R windup → ±180° ring-out). The achieved GT yaw rate (~33°/s) ≈ that 40°/s clamp, NOT a PX4 hardware limit (peak achieved 285°/s).

**Implication:** "start square" is **the reference design's operating assumption, NOT a crutch.** The PX4 IC's job is to reproduce `q_c=identity`; the IC leaving 100° is the real divergence from MATLAB. **"Converge from any initial yaw" is a deployment-robustness EXTENSION beyond what MATLAB validates** — legitimate, but new territory; the yaw law's large-yaw behavior is unproven. **Decision (user, 2026-06-08): proceed with (B) — extend the control for arbitrary initial yaw**, via un-throttling psi_d (`YAW_PSID_RATE` toward MATLAB's no-clamp) + yaw gains, then drive down steady-state error. See [[feedback_moment_yaw_canonical]] (the 2π wrap), [[feedback_dont_conclude_lag_floor]] (the yaw rate is clamp-limited, not lag-floored).
