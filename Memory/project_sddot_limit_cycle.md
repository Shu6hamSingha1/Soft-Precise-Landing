---
name: project_sddot_limit_cycle
description: "Combined-barrier lateral soft-fails are a FORCED under-damped limit cycle pumped by the s_ddot feedforward; forcing-set (no feedback/optical-flow lever damps it), scale-free SP = drop (most damped) > heavy-tau; z-taper invalid (needs altitude)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 5ef5a2d7-329a-4539-b101-9f7e6204c84a
---

**The combined-barrier lateral soft-fails (low-tau keep-s_ddot, ~17/25) are a FORCED LIMIT CYCLE, not a steady offset or chase lag** (2026-06-18/19). Terminal-phase trajectory (X_DS(1:3)=pos, X_DS(8:10)=vel; state is [p;q;v;w]) shows X (and Y on moving cells) oscillating ~1 s period, ±0.1 m position / ±0.3 m/s velocity, in lock-step with the desired-flow command |h_d_lat| swinging 0.07→0.6. Touchdown catches the cycle at a RANDOM PHASE -> the same cell flips "precise-but-soft" vs "soft-but-precise"; that's why no funnel/cap lever moved it.

**Driver = the s_ddot feedforward (in u_eq / V_dh), a positive-feedback loop through the optical-flow measurement:**
UAV lateral osc -> centroid s osc (1/z-amplified near ground) -> measured s_dot/s_ddot osc -> fed forward into the accel command -> drives the UAV osc. The velocity amplitude is set by the FORCING (s_ddot magnitude), NOT any feedback gain.

**Exhaustive lever check (ALL flat, diverge, or lag-limited — the velocity is forcing-set):**
- chi_r LOWER damps the POSITION amplitude (L3 xy 0.14->0.01!) but NOT velocity (cycle just runs higher-freq); <0.6 diverges.
- p_r, p_2inf, gamma_2(higher diverges), E(mixed), N, P, kappa_0(higher diverges), DH_D_CAP(doesn't bite, |dh_d|<cap) — all flat/diverge.
- Optical-flow filter CB_SDOT_FILT (savgol on s_dot): W=25 partially damps slow cells (Si3 0.33->0.22) but the lag DIVERGES the fast chase (Liss IC3); W>=101 diverges all; never reaches SP. Cycle is slow (~0.5Hz) -> needs wide window -> ~0.5s lag -> breaks moving-target tracking.
- h_d filter: NOT the right point — DROP keeps h_d (measured s_dot) and has NO cycle, proving the h_d reference osc isn't the driver; the s_ddot FF is. (And filter-h_d ~= filter-s_dot = CB_SDOT_FILT.)
- V_dh filter: DH_D_TAU exists (LPF on assembled V_dh, lags c~h too); CB_SDDOT_TAU (filters s_ddot component) is the targeted version we use. Same lag tradeoff.

**Scale-free SP verdict (altitude is OFF-LIMITS — monocular/unknown-depth premise):**
- DROP (s_ddot->kappa): 25/25, 75/75 noisy, **MOST damped (cycle ~0.16, decaying)** -> the OPTIMUM.
- heavy-tau (CB_SDDOT_TAU=1.5): 25/25, 74/75 noisy, but a DAMPED CYCLE not converged (~0.25 amplitude); passes only because the phase catches it low. The residual s_ddot FF re-injects cycle energy -> less margin than drop -> noise tips the fast cell over (Liss IC3 vy=0.288 seed3). "drop + leftover forcing."
- low-tau (0.5): undamped cycle (~0.5) -> 17/25.
- **z-taper (scale s_ddot by min(1,alt/z_ref)) was the only NO-LAG fix -> 24/25 noiseless + 75/75 noisy -> but uses ALTITUDE -> INVALID, removed entirely (reverted to commit a152479).** User: "We cannot rely on altitude in our control formulation."
- Corrects the earlier "heavy-tau equivalent to drop" claim: NOT equivalent — drop is strictly more damped.

**ROOT of the UNDERLYING cycle (present even in drop, decaying):** under-damped lateral loop. Diagnostic (drop C-IC4 vs altitude): sigma_x oscillates +-0.14 but |sigma|<<E=1.0 ALWAYS -> system never leaves the boundary layer -> switching robustness LINEARIZED away -> plain under-damped linear loop, no nonlinear damping. AND attitude error eR~0.10-0.18 rad is LARGER than the commanded tilt (atan(acd~1.0/9.8)~0.10 rad) -> **INNER-LOOP (attitude) LAG delivers lateral accel late -> cascade-bandwidth mismatch under-damps the outer loop.** Hypothesis: speed up kR/kOmega (and/or shrink E so switching re-engages). **Inner-loop kR/kOmega test PENDING.**

See [[project_combined_barrier_matlab]], [[feedback_hd_uses_measured_sdot]], [[feedback_combined_surface_divergence]].
