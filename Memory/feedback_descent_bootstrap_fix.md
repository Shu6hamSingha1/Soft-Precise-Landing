---
name: feedback_descent_bootstrap_fix
description: "No-descent hover root cause: loom h_z ≈ 0 at 6m (16px marker too small for ring flow); descent is self-reinforcing once bootstrapped. Fix: increase Γ_z or κ_z_0."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a377a083-d63b-447a-908e-12017cf609f1
---

**Root cause of no-descent hover at 6m (2026-06-09, gamma_s=1.0 rep4).**

**Why:** `h_z ≈ 0` when marker < ~30px (altitude > 4m with current board). Ring optical flow can't detect radial expansion at 16px marker size. With h_z=0 and h_d_z=−0.42: σ_z = +0.42 constant, a_u_z builds up slowly via κ_z adaptation (0.52→1.4 m/s² over 25s), thrust_norm reduces only 0.016 over 25s — too gradual to break PX4 hover equilibrium within the 25s timeout.

**The descent is self-reinforcing once started:** slight thrust reduction → drone drops a few cm → marker grows past ~30px → h_z goes negative → σ_z drives stronger a_u_z → descent accelerates. Reps that descend successfully already have h_z < 0 and a_u_z < 0 (braking) at t=5s — meaning they dropped below ~4m within 5 seconds.

**Why gamma_s=1.0 specifically triggers this:** gamma_s=1.0 centers the drone laterally in <3s. Slower gamma_s values had residual lateral drift at t=5s, which generated small optical flow contributions to h_z even at 6m, seeding the bootstrap. gamma_s=1.0's perfect early centering removes all lateral motion → h_z stays zero.

**RESOLVED (2026-06-10): KAPPA0_Z=1.0 + KAPPA_MAX_Z=3.0 baked as defaults.**

`PLASMC_KAPPA0_Z=1.0`: immediate a_u_z → descent starts within 0.5–1.0s (vs 25–43s hover). Validated n=5 with the κ-runaway fix below.

**κ_z runaway root cause (discovered while fixing bootstrap):**
The κ ODE is `κ_dot = θ_norm · N · G · |σ| − N · P · κ`. Three compounding factors at touchdown:
1. **Outlier containment** holds G_z near the barrier singularity (ratio→0.98 → G_z≈10) when h_z overshoots the funnel → sustained large drive even with the real signal clipped.
2. **θ_norm (||Θ||_F regressor norm)** reaches 25–191 from high optical-flow dynamics (c-term: `−dot(h,e3)·h` is quadratic in h).
3. **G_z** amplifies at low altitude.

κ_eq = θ_norm · G_z · |σ_z| / P_z can reach 200 at touchdown. P_z alone can't fix it without starving descent.

**Two-layer fix (both baked as of 2026-06-10):**
1. **κ freeze on containment**: when outlier containment fires on axis i, freeze κ_i (don't adapt from clipped/unreliable signal). Implemented in controller.py `_ctrlLoopUpdate()`.
2. **κ_max cap**: `PLASMC_KAPPA_MAX_Z=3.0` hard cap — a band-aid for the θ_norm blowup. The real θ_norm root cause was diagnosed separately (see [[feedback_theta_norm_klt_drift]]).

**Result n=5 (kap_z capped at 3, kap0_z=1):** xy_med=2.34m, xy_min=0.60m, desc_t=0.4–0.9s, 0/5 TL. Bootstrap fully solved.

**θ_norm TRUE root cause (diagnosed 2026-06-10):** NOT the h_z jump or containment-driven G singularity. The dominant cause was **KLT-drifted centroid s[0]=3.15 rad (off-screen, 2.5× beyond valid bound 0.889)** amplifying `cross(dw, s[:3])` by 3.5×. **Real fix: KLT in-bounds check in img_data.py** — stop KLT fallback when any corner exits the image (resets `_prev_aruco_pts`). DH_D_MAX stays at 50 (not 5); no s-clip or dw-clip (those are band-aids). IC2-5 validation of KLT-bounds fix is pending (trial 49). See [[feedback_theta_norm_klt_drift]].
