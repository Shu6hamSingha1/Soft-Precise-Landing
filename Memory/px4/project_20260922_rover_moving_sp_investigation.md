---
name: project_20260922_rover_moving_sp_investigation
description: "2026-09-22: why rover_cross + GT-FB cannot reach SP on a MOVING target. Static rover 5/5 SP. ⭐ Re-run under fixed rover_drive.py clock pacing (see project_20260922_rover_drive_wallclock_pacing_bug): Circular R=0.8 (now exactly MATLAB's own 5/5 wz=0.48 baseline) still 0/3 FAIL -- cleanest evidence yet of a real platform gap, not a speed mismatch. R>=1.6 9/9 precise 0/9 soft. Sinusoidal 3/3 PRECISE-only (was falsely 0/3 catastrophic FAIL pre-fix -- clock artifact). EightShape 2/3 SOFT+PRECISE -- first genuine moving-target SP found. Lissajous still 0/3 FAIL with an UNRESOLVED ~3x speed-tracking gap that survives the clock fix (separate mechanism, not clock pacing). Mechanism for the remaining failures = ~1 s lateral tracking lag + descent not gated by lateral error -> terminal a_u blow-up + thrust sacrificed. Lissajous deep-dive (see project_20260922_lissajous_cbf_and_divergence_mechanism) found the real trigger is the SAME thrust-cannibalization mechanism, not CBF or excursion -- trajectory retuning is the wrong lever; fix belongs in controller.py."
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

✅ **2026-09-22 RESOLVED — re-run under the fixed pacing** (`test_data/ClockFixRerun/20260922-144249`, same profiles/radii, n=3 each, [[project_20260922_rover_drive_wallclock_pacing_bug]]'s fix applied). Controlled speeds now match commanded values closely for every profile except Lissajous (see below). Results CHANGED substantially for two profiles -- do not cite the pre-fix Sinusoidal/EightShape numbers above any more:

| profile/R | controlled speed med (m/s) | result (was, pre-fix) |
|---|---|---|
| Circular R=0.8 (wz=0.48, matches MATLAB's own 5/5 baseline exactly) | 0.45-0.50 | **0/3 FAIL**, xy 0.31-0.47m, rel_vel 0.43-1.58 (was 0/35 across many arms) |
| Circular R=1.6/3.2/6.4 (v_tan constant by design) | 0.40-0.42 | 9/9 PRECISE-only, 0/9 soft (was 8/9 precise, matches) |
| Linear | 1.65-1.68 | 0/3 FAIL (was 0/3 FAIL) -- unchanged |
| **Sinusoidal** | 0.58-0.61 | **3/3 PRECISE-only** (was 0/3 FAIL, 2.4-3.5m errors -- THAT RESULT WAS A CLOCK-PACING ARTIFACT, not a real control failure) |
| Lissajous | 1.44-1.56 (commanded ~0.51 -- STILL a ~3x gap, unlike every other profile) | 0/3 FAIL (was 0/3 FAIL) -- still bad |
| **EightShape** | 0.45-0.48 | **2/3 SOFT+PRECISE** (was 0/3 SP) -- first genuine moving-target SP this whole investigation |
| CircularYaw | 0.26-0.28 | 1/3 FAIL, 2/3 PRECISE-only (was 0/3 SP, 3/3 precise -- roughly similar) |

**Two new findings:**
1. **The single cleanest MATLAB-vs-PX4 comparison available**: Circular R=0.8 now runs at EXACTLY MATLAB's own validated wz=0.48 rad/s (v_tan=0.384 m/s constant by construction), yet PX4 still fails 0/3, all genuine crashes not just "not soft." This is no longer explainable by a speed-mismatch artifact -- it is direct evidence of a real platform/architecture gap (PX4's real actuation lag + Gazebo physics vs MATLAB's largely-idealized plant, per [[project_matlab_px4_lag_model_2026_09_10]]-class reasoning) at the exact condition MATLAB itself proved works.
2. **Lissajous has a SEPARATE, still-unexplained speed-tracking gap** (~3x over commanded, survives the clock fix) that every other profile does not show. Working hypothesis, NOT confirmed: Lissajous's two-frequency path with frequent sharp direction reversals may exceed the rover's pure-pursuit steering controller's ability to track curvature cleanly (20 Hz setpoint stream may under-sample the curve's fastest direction changes), causing real physical overshoot/corner-cutting rather than a software pacing bug. Needs its own investigation before trusting any Lissajous conclusion.

**How to apply (updated):** the earlier "speed dominates, not curvature" conclusion is now MORE precisely supported for Circular/Sinusoidal/EightShape/CircularYaw (all now at controlled, closely-matched speeds) but Lissajous should be EXCLUDED from that generalization until its own speed-tracking gap is understood -- its measured speed was never actually controlled, before or after the clock fix. EightShape's SP success and Sinusoidal's correction are the most promising newly-available directions to pursue for a general moving-target fix.

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

---

⭐⭐ **2026-09-22 UPDATE, cross-linked**: [[project_20260922_lissajous_cbf_and_divergence_mechanism]]
traced Lissajous's failure down to a within-rep divergence trigger at t~2.7s (in a
slowed-speed rep) that is NOT CBF-driven and NOT excursion-driven -- it's this file's own
thrust-cannibalization mechanism (`I_a_z` eaten by growing `I_a_xy`, `h_d_z` never gated on
lateral error) resurfacing on its own timeline regardless of trajectory speed/shape. This is
the clearest evidence yet that TRAJECTORY-level fixes (speed, amplitude, phase -- tried
repeatedly on Lissajous, k=1.0->0.4->0.2->0.1) are the wrong lever; the two controller-side
fixes flagged above (descent gating, protecting I_a_z) are now the priority, not further
profile-speed tuning.
