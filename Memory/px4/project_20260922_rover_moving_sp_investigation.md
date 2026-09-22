---
name: project_20260922_rover_moving_sp_investigation
description: "2026-09-22: why rover_cross + GT-FB cannot reach SP on a MOVING target. Static rover 5/5 SP. Circular R=0.8 (0.55-0.6 m/s, 0.65-0.7 rad/s) 0/35 across default law, KP 0.3/0.1/0.02, retune, law-off. R>=1.6 (same speed) 8/9 precise, 0/9 soft. Mechanism = ~1 s lateral tracking lag + descent not gated by lateral error -> terminal a_u blow-up + thrust sacrificed. SUPERSEDES the 09-18 '4/4 with yaw law off' discriminator (it also zeroed PLASMC_YAW_* and never was SP)."
metadata:
  type: project
---

**Correction first.** The 09-18 discriminator "law off lands 4/4" ([[project_20260916_curve_qgate_revalidation]]) was NOT law-off alone: its recorded `Control_Params` overrides also had `PLASMC_YAW_GAMMA/KAPPA0/N/OMEGA=0`, `PLASMC_YAW_ALPHA_FILT=0`, `PLASMC_TERMINAL_COMMIT=0` (yaw ASMC neutered = heading held, UAV yaw rate 0), and those landings were xy 0.11-0.12 m, rel_vel 0.55-0.79 (touchdown, NOT SP). My 09-21/22 reruns used only `PLASMC_YAW_RATE_LAW=0`, so they never reproduced it. Nothing regressed; also not `2177670b` (pre-commit tree fails identically). The circle was already an open problem ([[project_rover_turning_open]]).

**Measured (GT-FB, rover_cross, cross marker, HEAD ~2026-09-22):**
- Static rover: baseline 5/5 SP (xy 0.006-0.009, rel_vel 0.05-0.09). Retune bundle 3/5 (2 vertical OSCILLATION anomalies).
- Circular R=0.8 (target ~0.58 m/s, yaw ~0.62-0.72 rad/s): 0/35 landed-SP: law on (5+15), KP 0.3/0.1/0.02, retune (0/5), law off (0/5, 4 fly-aways).
- Circular R=1.6/3.2/6.4 at the same speed, law default: 8/9 PRECISE (xy 0.07-0.11), 0/9 SOFT (rel_vel 0.3-0.7).
- Perception-vs-GT w_z (stationary IC1-5, 08-31 data): slope 0.73, corr 0.86 -> real deficit ~1.4x (IC5 ~2x); 0.9 gate justified (perception spikes to 2.6-3.2). MATLAB "no deficit" is MATLAB-only.
- The MATLAB adaptation retune (E,N,P,kappa0/max,chi_z,p_hinf_z) FAILS on PX4: 0/25 precise on perception cross_marker, 3/5 on GT-FB static. Do not port.

**Mechanism (law_on R=0.8 rep, time series):** drone starts at rest over the target; target starts at ~0.58 m/s at t~1 s. Lateral speed only matches by ~4 s; lateral error is ~0.5-0.6 m mean (0.8 max) at ALL radii = ~1 s lag. Descent is optic-flow exponential (vz ~ 0.3*z) and is not gated by lateral error, so error/altitude (s_e_n) reaches ~1.2-2 at z<1.2 m; h_d_xy grows 0.2->4.4, a_u_xy 3->110 m/s^2 in 0.75 s, Ia_z -10 -> -4 (thrust sacrificed to lateral demand) -> plunge 2-4 m/s. With yaw tracking (uav yaw rate 0.59) error reaches 1.47 m vs 0.70 m yaw-held: consistent with MATLAB's "s_e orbits in the yaw frame" ceiling.

**Open / next:** reduce the ~1 s lateral lag (target-velocity feedforward / h_d lead), gate descent on lateral error, or stop a_z relief starving thrust; SP on moving targets also needs vertical soft touchdown (rel_vel <=0.2). R=0.8 exceeds the yaw ceiling MATLAB documents (~0.5 rad/s).
**How to apply:** don't re-run KP / yaw-law / adaptation-retune arms for the R=0.8 failure (all null); a fair yaw-off probe must copy the 09-18 override set.
