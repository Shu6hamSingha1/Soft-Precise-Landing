---
name: feedback-gtfb-kappa-z-bounce
description: KAPPA_MAX_Z must be ≤0.034 to prevent PX4-saturating Z-terminal bounce in GT-feedback (XI2_Z=1.0) mode; G_z blows up at terminal altitude when p_z=1.5
metadata: 
  node_type: memory
  type: feedback
  originSessionId: e265057f-9381-4119-bd0b-0fb5e3c7498f
---

KAPPA_MAX_Z must be capped ≤ 0.034 to prevent a PX4-saturating Z bounce in the terminal landing phase when XI2_Z=1.0.

**Mechanism:** At terminal altitude (alt≈0.12m), with XI2_Z=1.0 the Z funnel has converged (p_z=1.5). h_e_z grows toward −1.394 in the Z limit cycle:
- G_z = p_z² × (p_z² + h_e_z²) / (p_z² − h_e_z²)² = **99.69** (huge)
- With κ_z=0.5, θ=7.37: switching = 7.37 × 99.69 × 0.5 = **367 m/s²** → PX4 saturates (max ~25 m/s²)
- Bounce: from alt=0.13m to alt=0.40m → drone drifts 0.6m laterally during bounce → xy=1.448m (NC135)
- PX4-saturation threshold: κ_z_max ≤ 25 / (θ × G_z) = 25 / (7.37 × 99.69) = **0.034**

**With KAPPA_MAX_Z=0.03:** switching = 22.1 m/s² (non-saturating) → no bounce → xy=0.316m (NC137), xy=0.245m (NC141).

**Why κ_z_max=0.5 doesn't work:** any κ_z > 0.034 gives PX4-saturating bounce because the PX4 thrust limit is what sets bounce amplitude — higher κ_z just means the saturation lasts longer / more secondary oscillations.

**Why XI2_Z=0.2 (default) doesn't help:** with slow funnel convergence, p_z≈1.666–1.99 at landing → G_z=6–19. But the descent is SLOWER (less z-authority) → more time in the terminal 1/Z zone → more lateral drift. Also G_z×0.5 still saturates PX4 if the landing takes ≥12s. XI2_Z=1.0 + KAPPA_MAX_Z=0.03 = fast descent + no bounce — the winning combination.

**Kappa_z natural value:** with P_z=5, N_z=0.1, κ_z decays naturally to ~0.025–0.030 during clean descent. The 0.03 cap sits just above natural; it's effectively only active during the terminal limit cycle (when κ_z would try to grow above 0.03).

**GT-FEEDBACK ONLY — DO NOT BAKE INTO PRODUCTION:**
NC147-152 confirmed that KAPPA_MAX_Z=0.03 in production (perception on, XI2_Z=0.2) causes REGRESSIONS:
- Natural κ_z in production ≈ 0.03-0.05 via P_z=5 leakage anyway
- But setting κ_max_z=0.03 forces IMMEDIATE step-change from κ0_z=0.25 to 0.03 at step 1
- This weakens early Z noise rejection → σ_z variance grows → occasional σ_z explosion → lateral kick → fly-away
- IC1 fly-away rate: 0/5 (baseline) vs 2/2 consecutive (NC150/NC151 with bake) — clear regression
- Reverted (22 Jun 2026 session). The bake was a one-rep experiment only.

**Production note:** with XI2_Z=0.2 (default): G_z ≈ 18.88 at landing. Natural κ_z ≈ 0.03-0.05 → switching ≈ 4-7 m/s² (marginally below PX4 25 m/s² limit). Z bounce risk is LOW with default config. The 1/5 terminal-1/Z failure mode in production is LATERAL fly-away, not Z bounce.

**How to apply:** In GT-feedback tests with XI2_Z=1.0, always set `PLASMC_KAPPA_MAX_Z=0.03`. NEVER bake into combined barrier auto-alignment. Do not confuse with KAPPA_MAX_X/Y — the lateral kappa issue is a separate problem ([[feedback-gtfb-lateral-orbit-divergence]]).
