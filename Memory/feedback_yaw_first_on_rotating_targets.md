---
name: Try yaw control parameters first on rotating-target failures
description: When PLASMC fails on a trajectory that has nonzero target yaw rotation, exhaust yaw-loop knobs (Omega_a, Gamma_a, n_a, p_a, E_a, ff(3), wi(3)) before touching lateral knobs (zp, zd, p_20).
type: feedback
---

When PLASMC fails on a trajectory whose target frame rotates (e.g. Circular has `q = [cos(wz*t/2);0;0;sin(wz*t/2)]` with `wz=0.1 rad/s`), try yaw control parameters first before touching lateral knobs.

**Why:** I jumped to `zp` (lateral P-gain) on Circular Run 5 because it was the same IC and same cone-clamp pattern that broke Linear Run 5. The user corrected: try yaw first. Rotating-target trajectories impose a *ramped* yaw setpoint, which a controller without sufficient integral / feedforward action will lag. That lag rotates the body frame → mis-aligns thrust → looks identical to a lateral-divergence failure even though the root cause is yaw.

**How to apply:**
- Detect: trajectory has nonzero `w` and/or non-identity `q` in `traj_Gen.m` (Circular, CircularYaw, anything with rotation).
- Try in order: `Omega_a` (integral gain — best for ramp tracking), `ff(3)` (yaw rate feedforward in inner loop), `wi(3)` (yaw rate integral), then `Gamma_a`/`E_a` if those don't help.
- Only after exhausting yaw knobs should you touch `zp`/`zd`/`p_20`/`p_2inf`.
- The diagnostic that this is a yaw issue: large yaw drift on the *failed* run that exceeds the trajectory's natural yaw rotation. (On Circular at t=4.46s with wz=0.1, natural yaw drift ≈ 26°. Run 5 had 68° → 42° of *true* yaw error contributed to the divergence.)

**TODO (CLOSED 2026-04-08):** n_a/p_a/kappa_a_0 sweep complete. See `reference_plasmc_parameters.md` for per-parameter empirical impact. Conclusion: defaults `n_a=0.05`, `p_a=2`, `kappa_a_0=0.1` are at the sweet spot — every direction tested either regressed Circular or broke Linear via the same yaw-quiet/rotating trade-off as `E_a=1.5`. Only `Omega_a` (1.0→1.5) gave a strict win across all trajectories because it adds integral action that has nothing to integrate on yaw-quiet targets.
