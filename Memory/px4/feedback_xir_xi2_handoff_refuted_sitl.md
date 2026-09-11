---
name: feedback_xir_xi2_handoff_refuted_sitl
description: "NEGATIVE result: the 2026-09-10 MATLAB handoff (docs/HANDOFF_xir_xih_moving_target.md) candidate PLASMC_XIR_{X,Y}=0.20 + PLASMC_XI2_{X,Y}=0.20 does NOT help in SITL and reintroduces the IC1 kappa-ratchet. GT-FB rover-world A/B (n=3, base XIR0.10/XI2 1.0 vs cand, moving+static, IC1+IC2): baseline already lands 3/3 precise on moving IC2 (xy~0.04 m, terminal s_e_n ~0.1, no growth-back) -> the MATLAB funnel-deficit failure does not reproduce. Candidate: moving IC2 2/4 precise (worse spread, p_r collapses ~1.9), moving IC1 1/3 DETONATION (kappa_xy 0.5->30=cap, a_u_xy 1.3e5, xy 12 m), static neutral-to-slightly-worse. Keep XIR=0.10 / XI2=1.0."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f78fde4c-2847-4689-96f6-0693b2bc0c15
  modified: 2026-09-10T18:34:15.421Z
---

**The 2026-09-10 XIR/XI2 handoff does NOT transfer to SITL (tested 2026-09-11, GT-FB, user-approved run).**

**Handoff claim** (`PX4_Gazebo/docs/HANDOFF_xir_xih_moving_target.md`, commit c0198d5b): MATLAB
realistic-plant sim says `PLASMC_XIR_{X,Y}=0.20` + `PLASMC_XI2_{X,Y}=0.20` (vs baked 0.10 / 1.0)
lands soft-precise on **both** stationary and moving targets, where XIR=0.10 supposedly fails the
moving regime — wide position funnel (`G_r ≈ 2/p_r`, XIR sets `p_r` decay) leaves the persistent
moving chase-error unregulated, terminal `|s_e_xy|` grows to 0.79 → marker walks out of frame.

**The test.** `test_data/XirXi2_RoverGTFB/harness.sh` — rover world ONLY (per user), `PLASMC_GT_FEEDBACK=1`
(strips the rover detector-collapse confound so this isolates the control-side funnel claim),
2 arms × 2 motion {moving Circular @ nominal, static `ROVER_MOTION=0`} × 2 IC {IC2 = MATLAB
discriminator, IC1 = kappa-leakage-drift canary the handoff itself flags}, n=3. All other params baked.

**Results** (precise / xy_err m / terminal s_e_n / kappa_xy / a_u_xy):
- **moving base IC2: 3/3 precise, xy 0.03–0.05, s_e_n ~0.1 (converged, no growth-back).** The
  MATLAB failure **does not reproduce** — SITL flow lag / plant dynamics regulate the moving
  residual fine at XIR=0.10. The handoff's whole premise is a noiseless-MATLAB artifact.
- moving cand IC2: **2/4** precise, xy up to 0.107, larger terminal s_e_n, `p_r` collapses ~1.9
  (vs base ~4.0). XI2=0.20's looser loom funnel *weakens* terminal regulation — worse, not better.
- moving base IC1: 2/3 precise, bounded (kappa_xy ≤ 1.22, worst rep xy 0.40).
- **moving cand IC1: 1/3 DETONATION** — rep0 kappa_xy 0.50→30 (=cap), a_u_xy 1.3e5, xy 12 m,
  s_e_n 80. Exactly the [[project_ic1_kappa_leakage_drift_20260721]] fly-away the handoff warned
  about: XI2→0.20 drops terminal G-exposure → `P_XY=2.5` goes over-leaky → κ drains → error
  grows inside the wide funnel → ratchet. Fails handoff success-criterion #3 (kappa bounded).
- static (both IC, both arms): 3/3 soft+precise, candidate marginally worse (IC2 xy 0.025 vs
  0.016). Neutral-to-slightly-worse; no regression, no benefit.

**Why:** same boundary as [[feedback_matlab_gains_not_portable]] / [[feedback_px4_gains_authoritative]] —
MATLAB findings transfer as MECHANISM, not gain VALUES. The MATLAB funnel-deficit mechanism has
no SITL correlate; the SITL lag that breaks *other* MATLAB→PX4 ports here happens to make the
baseline wide-funnel strategy work on the moving target too.

**Why:** keeps a refuted candidate from being re-proposed; documents that the moving-rover
precision lever is NOT XIR/XI2.

**How to apply:** keep `PLASMC_XIR_{X,Y}=0.10`, `PLASMC_XI2_{X,Y}=1.0`. If moving-target
precision needs work later, look elsewhere (velocity damping / detector, not the funnel rates).
Any future XI2 drop MUST first walk `P_XY` down from 2.5 with an IC1 kappa/a_u re-check — but
there is no SITL evidence of the problem this was meant to solve. Caveats on this refutation:
n=3, GT-FB only (perception-ON rover still blocked on detector collapse independently),
single trajectory (Circular @ nominal). Related: [[project_matlab_yaw_rate_law_port_2026_09_09]],
[[feedback_gain_values_not_portable_either_direction]].
