---
name: feedback-gtfb-lateral-orbit-divergence
description: "Terminal lateral fly-away = PX4-saturation-induced orbit divergence (commands 150× too strong), NOT insufficient authority; fix = cap KAPPA_MAX_X/Y"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: e265057f-9381-4119-bd0b-0fb5e3c7498f
---

The terminal lateral "fly-away" is a **PX4-saturation-induced orbit divergence**, NOT insufficient lateral authority.

**Finding** (GT-feedback campaign NC133–NC141 + fork agent analysis of NC137 rep):
- At alt=0.2m the drone executes a damped circle of radius ~0.05m around the target
- Commands averaged **40.9 m/s²** over alt<0.3m — **150× more than needed** (only 0.27 m/s² would arrest the 0.136 m/s mean drift)
- Commands **saturate PX4** (limit ~25 m/s²); 38ms rate-loop lag causes overshoot on each correction cycle
- 1/Z amplification makes each cycle's overshoot LARGER as altitude decreases → diverging spiral
- θ grows from c-term explosion (large angular velocity from aggressive tilts) → switching grows further → deeper saturation → worse overshoot → cascade

**Why:** NOT insufficient Γ or chi_r. The reaching term (Γ×G_COMBINED⁻¹×σ) is SMALL near the barrier (G_COMBINED⁻¹→0 as h_e_x→P2INF_X). The **switching term** (θ × G_COMBINED × κ_x × sat) is the culprit — it grows as κ_x adapts up during the orbit and θ grows from the c-term explosion.

**Self-reinforcing stability hypothesis for fix:** cap KAPPA_MAX_X/Y at a small value (e.g. 0.05). With κ_x=0.05 and θ=15 (stable when commands are small): switching = 15 × G_x × 0.05. At G_x=5: 3.75 m/s² — well below PX4 limit. Small commands → small tilts → small angular velocity → small c-term → small θ → small commands. The positive feedback loop is broken.

If orbit radius stabilizes at ~0.05m (current tight orbit at alt=0.2m), then at touchdown (alt=0.05m) xy ≈ 0.05–0.10m — sub-0.1m threshold.

**Previously REJECTED lateral levers (don't retry):**
- chi_r=0.85: REGRESSED xy (0.316→2.490m, NC138) — overshoot from lag, orbit expanded not stabilized. Still applies even with GT feedback because PX4 rate-loop lag is 38ms regardless of perception quality
- chi_r=0.5 (current baked): moderate, orbit diverges but slowly
- COMMIT_AU_MAX: previously rejected in non-GT-feedback context; in GT-feedback with small chi_r=0.5 commands, COMMIT_AU_MAX is below activation threshold anyway

**Campaign results n=3 (NC143/145/146, GT-feedback IC1, all-κ_max=0.03 + XI2_Z=1.0):**
- xy: [0.178, 0.202, 0.215], median=0.202m (all reps land ON the marker)
- rel_vel: [0.053, 0.370, 0.667], median=0.370m/s (near-marginal: sometimes soft, usually not)
- SOFT (rel_vel<0.2m/s): 1/3

**GT-feedback ceiling CONFIRMED at κ=0.03:** xy≈0.2m, rel_vel≈0.37m/s median. Both are LAG-LIMITED (38ms PX4 inner loop), not gain-tunable. The orbit is near-marginal: κ=0.03 is the minimum needed to prevent barrier breach (NC144 with κ=0.01 diverged), but it still allows residual stochastic orbits from Gazebo physics.

**Kappa sweep results:**
- κ=natural (0.5-0.89): xy=0.28m, rel_vel=0.63m/s median (orbit diverging)
- κ=0.05: xy=0.192m, rel_vel=0.632m/s (inert — kappa naturally below 0.05)
- κ=0.03: xy=0.202m, rel_vel=0.370m/s median (orbit near-marginal)
- κ=0.01: xy=0.279m, rel_vel=0.663m/s (REGRESSION — insufficient correction near barrier)
- κ=0.03 is the SWEET SPOT: minimum needed to prevent barrier breach without PX4 saturation

**What NC143's 0.053m/s was:** a stochastic outlier (same IC pos_err=0.011m as NC146 which gave 0.667m/s). Not representative. Median rel_vel ≈ 0.37m/s for this config.

**How to apply:** Before diagnosing "insufficient lateral authority" for a fly-away, compute avg |a_u_xy|. If >> 25 m/s²: the issue is saturation-induced overshoot, not weak gains. For production: KAPPA_MAX_Z=0.03 is the primary fix (Z bounce prevention); lateral kappa caps in production are TBD. Chi_r=0.85 + all-kappa_max=0.03 is an untested lever (chi_r=0.85 was rejected in NC138 WITHOUT lateral caps — WITH caps it might work).

**Why:** the lateral wall is NOT a perception/authority ceiling — it's a control-saturation stability problem. [[feedback-gtfb-kappa-z-bounce]] is the Z-axis analog.
