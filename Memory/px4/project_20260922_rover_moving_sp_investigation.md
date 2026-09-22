---
name: project_20260922_rover_moving_sp_investigation
description: "2026-09-22: why rover_cross + GT-FB cannot reach SP on a MOVING target. Static rover 5/5 SP. Circular R=0.8 0/35 across default law, KP 0.3/0.1/0.02, retune, law-off. R>=1.6 8/9 precise 0/9 soft. Non-Circular sweep (Linear/Sinusoidal/Lissajous/EightShape/CircularYaw) 0/15 SP too -- SPEED, not curve shape, is the dominant variable (Linear straight-line at 2.17 m/s fails as hard as Circular; slowest profiles CircularYaw 0.35 m/s and Circular r>=1.6 ~0.55-0.6 m/s land precise but too fast for soft). Mechanism = ~1 s lateral tracking lag + descent not gated by lateral error -> terminal a_u blow-up + thrust sacrificed. SUPERSEDES the 09-18 '4/4 with yaw law off' discriminator (it also zeroed PLASMC_YAW_* and never was SP)."
metadata:
  type: project
---

## Trajectory-profile sweep (2026-09-22, cont'd) -- speed dominates, not curvature

`rover_cross`, GT-FB, default config, n=3 each, `ROVER_SPEED_MULT=1.0`
(`test_data/RoverTrajSweep/20260922-113047`):

| profile | tgt speed med/max (m/s) | tgt yaw rate med/max (rad/s) | result |
|---|---|---|---|
| Linear | 2.17 / 2.59 | 0.02 / 1.47 | 0/3, xy 0.22-0.34 m, rel_vel 2.0-2.5 m/s |
| Sinusoidal | 0.74 / 0.90 | 0.50 / 0.91 | 0/3, xy 2.4-3.5 m, rel_vel 3.6-4.5 m/s |
| Lissajous | 1.80 / 3.03 | 1.79 / 2.75 | 0/3, xy 1.5-2.2 m, rel_vel 4.5-4.8 m/s (worst overall) |
| EightShape | 0.52 / 0.90 | 0.15 / 0.90 | 0/3 SP: 1 precise-only, 1 soft-only, 1 fail |
| CircularYaw | 0.35 / 0.96 | 0.28 / 0.90 | 0/3 SP but 3/3 PRECISE (xy 0.05-0.08 m); rel_vel 0.22-0.35 m/s, just above the 0.2 m/s soft threshold |

0/15 SP across all 5 non-Circular profiles -- matches Circular's 0/44 (35 at r=0.8 + 9 at
r>=1.6). **Speed, not curve shape, is the dominant variable**: Linear (dead-straight, 0
yaw rate) fails as hard as Circular at 2.17 m/s; the two closest-to-SP results are the
two SLOWEST profiles tested (CircularYaw 0.35 m/s, and Circular r>=1.6's ~0.55-0.6 m/s
from the earlier entry) -- both land precise but arrive 0.2-0.35 m/s too fast, the same
"close but not soft" signature. Sinusoidal and Lissajous (both combine moderate-high
speed with high yaw rate, Lissajous median 1.79 rad/s -- highest tested) are the worst
outcomes of the whole investigation, multi-metre xy error and 3.6-4.8 m/s impacts.
**How to apply:** the fix search (target-velocity feedforward / lead, descent gated on
lateral error, protect terminal thrust) should be validated primarily against TARGET
SPEED as the stress axis, not against a specific curve; a fix that only helps Circular
without addressing the underlying ~1 s lateral lag at speed will not generalize to
Linear/Sinusoidal/Lissajous. CircularYaw (slow, gentle) is the best available near-miss
case to tune a soft-touchdown fix against first before re-testing the harder profiles.

⚠ **2026-09-22 caveat found AFTER this sweep**: the target speeds in this table (and the earlier Circular r-sweep) are not controlled values -- `rover_drive.py` paces its trajectory reference off WALL-CLOCK (`asyncio.sleep`), not Gazebo's simulated `/clock`, so measured GT speed can diverge from the commanded `ROVER_SPEED_MULT`/formula value depending on real-time factor at run time (confirmed code-level, measured 28-250% higher than commanded across Sinusoidal/Lissajous/Circular this session). See [[project_20260922_rover_drive_wallclock_pacing_bug]]. The qualitative "speed dominates" conclusion still holds (it's based on measured GT speed, the real physical stress the drone faced) but don't treat the table's speed column as a settable/reproducible experimental variable.

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
