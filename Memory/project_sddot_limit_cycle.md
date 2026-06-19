---
name: project_sddot_limit_cycle
description: "Combined-barrier lateral soft-fails are a FORCED limit cycle pumped by the s_ddot feedforward; ONLY removing the forcing (drop) de-pumps it. kR/heavy-tau give cosmetic deterministic 25/25 by landing on a velocity null (phase luck) but the cycle persists -> noisy 83/100 + breach. Drop-only stands; z-taper invalid (altitude)"
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

**ROOT of the UNDERLYING cycle (present even in drop, decaying):** under-damped lateral loop. Diagnostic (drop C-IC4 vs altitude): sigma_x oscillates +-0.14 but |sigma|<<E=1.0 ALWAYS -> system never leaves the boundary layer -> switching robustness LINEARIZED away -> plain under-damped linear loop, no nonlinear damping. AND attitude error eR~0.10-0.18 rad is LARGER than the commanded tilt (atan(acd~1.0/9.8)~0.10 rad) -> **INNER-LOOP (attitude) LAG delivers lateral accel late -> cascade-bandwidth mismatch under-damps the outer loop.** Hypothesis: speed up kR/kOmega (and/or shrink E so switching re-engages).

**✅ ROOT CONFIRMED + SCALE-FREE FIX (2026-06-19) — overturns the "drop-only" verdict.** Speeding up the
inner-loop attitude gains DAMPS the cycle (scale-free — kR/kOmega use no altitude): KR_OVERRIDE=[3;3;0.5],
KOMEGA_OVERRIDE=[0.6;0.6;0.1] (= kRx2) on the RESPONSIVE low-tau keep-s_ddot -> 17/25 -> 22/25; + light
CB_SDDOT_TAU=0.7 -> **24/25, no breach** (only Liss IC3, vel=0.273). So the cycle was NEVER an intrinsic
s_ddot limit — it was a cascade-bandwidth mismatch. Shrinking E instead DIVERGES (chatter) -> bandwidth, not
boundary layer. (kRx3 over-damps -> 21/25; kRx2 is the sweet spot. Old "kR x1.25 failed Linear IC5" caveat
was back-mapped-design-specific; combined-barrier Linear IC5 stays SP.)
**❌ RESPONSIVE-s_ddot 25/25 is a PHASE-LUCK ARTIFACT — NOT a real fix (2026-06-19, decisive).** Two dead ends closed:
- **Asymmetric kR is illusory.** kR_y<3 fixes Liss IC3 ONLY by detuning it into a different limit-cycle basin (sharp bistability near the Hopf point — the exact value matters: ky=2.0 lands in the damped basin, ky=2.25 in the pumped basin -> L3 0.41). But the FULL 5x5 shows kR_y=2.0 breaks Static IC3/IC5 + Circ IC4/IC5 (all slip 0.23-0.32) -> SP 18-19/25. The user's 6-cell probe missed the collateral. kR=[3;3] (symmetric, 24/25, only L3=0.27) is the real optimum.
- **Heavy tau gets deterministic 25/25 but masks the EFFECT, not the cycle.** kR=[3;3], CB_SDDOT_TAU=1.2 (or 1.5) -> SP 25/25 noiseless, no breach. BUT terminal-waveform probe on Liss IC3: over the last 4 s, X-vel peak-to-peak amplitude moves only 0.79->0.71->0.67->0.64 across tau 0.7->1.5 (~15%), period fixed ~1.7 s — **the cycle is fully alive.** What collapses is the SAMPLED terminal velocity (vx 0.125->-0.007, vy 0.214->0.075, 16x) — touchdown just lands near a velocity NULL of the ongoing oscillation. Phase luck. **Noisy run proves the fragility:** 4 seeds -> 25,25,24,9 /25 = **83/100**, Liss IC3 vel jumps to 1.45 (seed3) / 3.78 (seed4) with worstResid 2.014 (funnel breach) + 4 non-landings. Perturbing the terminal phase slides touchdown off the null onto the live cycle.

**Answer to "is the param damping the cycle or its effect": the EFFECT.** tau LPFs the terminal 1/z spike of the s_ddot forcing but does NOT break the optical-flow positive-feedback loop; the cycle amplitude is essentially unchanged. Only removing the forcing entirely (CB_DROP_SDDOT) de-pumps the loop at the source. **CONCLUSION: keep the baked s_ddot-DROP default (25/25 + 75/75 noisy, genuinely cycle-free). Responsive-s_ddot via kR/tau is NOT bakeable.** The 2026-06-19 "raise kR -> scale-free fix overturns drop-only" verdict above is itself overturned: kR helps the cosmetic deterministic number but the cycle (and noisy fragility) remains. Drop-only stands.

**🔬 MECHANISTIC ROOT CAUSE — the FF is ANTI-DAMPING (2026-06-19, measured directly; supersedes the "1/z gain explosion" guess, which is WRONG — the FF magnitude is bounded ~0.4 by dhd_cap+LPF, NOT exploding near ground).** Phase probe on Liss IC3, terminal 4 s, accel command a_x ~ -V_dh_d_x:
- **corr(-FF_x, vx) = +0.64 (KEEP) vs -0.50 (DROP).** The s_ddot FF acceleration is POSITIVELY correlated with lateral velocity -> it injects power IN PHASE with motion (mean(FF_x*vx)=+0.044>0) -> anti-damping. DROP's transport-only term is -correlated -> extracts energy.
- **Net damping collapses: corr(a_actual, vx) = -0.02 (KEEP, ~ZERO net damping -> sustained) vs -0.18 (DROP, genuinely damps -> decays).** The FF anti-damping almost exactly cancels the loop's weak natural damping. THAT zero-net-damping IS the limit cycle. (Underlying loop is lightly damped from cascade-lag + boundary-layer linearization |sigma|<<E=1.0; in DROP it decays, the user's C-IC4 trace confirms.)
- **tau sweep proves no filter setting fixes it:** corr(-FF,vx) falls monotonically +0.77(0.5)->+0.64(0.7)->+0.40(1.0)->+0.18(1.2)->0.00(1.5) — but ONLY by attenuating the FF toward zero (= approaching drop). Even at tau=1.5 (corr~0) net damping is just -0.04, still far from drop's -0.18. There is NO tau with a meaningful FF AND healthy damping. tau<0.5 DIVERGES (noisy s_ddot -> FoV breach).

**WHY intrinsic:** the FF = d/dt of MEASURED optical flow s_dot, and h=v/z contains the UAV's OWN lateral velocity -> differentiating+feeding-forward closes a positive-feedback loop on ego-velocity; the measurement+filter chain rotates it ~90 deg into the velocity-aligned (anti-damping) phase. The target-accel-FF benefit and the anti-damping come from the SAME measured-s_dot derivative -> cannot keep one without the other. **=> Responsive-s_ddot robust elimination is INFEASIBLE.**

**HOW TO ELIMINATE COMPLETELY = remove the s_ddot FF (CB_DROP_SDDOT, the baked default) — UNIQUE complete fix.** It is the anti-damping energy source; removal restores natural damping (-0.18) -> cycle decays to a stable point -> 25/25 + 75/75 noisy, no breach. Partial levers (kR inner-loop speedup, tau, chi_r) only modulate amplitude or touchdown phase, never restore net damping. (A structural alternative — add explicit ego-velocity damping decoupled from the FF — is theoretically possible but unproven/risky; drop already works, so not pursued.)

**🧱 KEEP-s_ddot CAMPAIGN (2026-06-19, user directive "we are NOT dropping s_ddot") — ceiling 92/100, structural wall identified (CB35-40).** Pushed keep-s_ddot as hard as possible:
- Best config: COMBINED_BARRIER+C_SIMPLE, CB_DROP_SDDOT=0, tau=0.7, chi_r=0.65, kR=[4;4;0.5], kOmega=[0.6;0.6;0.1], p2inf_xy=0.5, p2inf_z=1.0 (NEW per-axis descent lever, P2INF_Z_OVERRIDE hook added; default p_2inf(3)=1.5 unchanged) -> NOISELESS 25/25 no breach, NOISY 92/100 (seeds1-3 PERFECT 25/25, seed4=17/25). Trajectory: bare 83 -> 89 (chi_r-low+kR-high stacked damping, CB37 levers) -> 92 (+descent margin).
- Per-axis decomp of the seed-4 wall: ~8 cells fail pure DESCENT vz~0.20 (noise inflates terminal descent 0.10->0.20; p2inf_z=1.0 recovers them); residual = Liss IC3 lateral DIVERGENCE.
- Liss IC3 seed4 TRACE: cycle IGNITES AT ALTITUDE (vy->1.84 @4m t=1.5s), rotates X<->Y (coupled 2-D mode), terminal |v|=2.6 breach. Noise-ignited GROWING oscillation = the s_ddot anti-damping pump under worst-case seed, NOT a terminal-margin miss.
- WHY no lever fixes it (the structural cause, CB40): the residual is an AUTHORITY/DAMPING DEFICIT curable only by RAISING loop gain — and p_20, Gamma, gamma2, kR, chi_r are ALL gain knobs (notably p_20 covertly: barrier gain G_2~1/p_2, so p_20=14 raises flow-loop gain enough to damp Liss IC3 -> seed4 2.76->0.19, but over-drives the cascade-LAG-limited cells -> 65/100, position breaches). Flow funnel occupancy is only 0.01-0.20 (even during divergence) -> NOT containment, it's GAIN. The cascade phase-lag caps global gain -> no single gain damps Liss IC3 AND keeps the lag cells. Yaw is HEALTHY (terminal |e_a|<2.5deg, bounded u_a/kappa_a; a victim of the lateral cycle on the diverging seed, not a cause).
- ~~CONCLUSION: 92/100 is the keep-s_ddot ceiling; only DROP -> 75/75~~ **OVERTURNED (CB45, 2026-06-19):** the DROP "75/75" was seeds 1-3 ONLY (CB16); seed4 never tested. FAIR 4-seed comparison: baked DROP = 82/100 (25/25/24/8) vs keep-s_ddot 92-config = 92/100 (25/25/25/17). **Keep-s_ddot stacked-damping BEATS drop over the hard 4-seed set.** And Static IC5 seed4 (the descent divergence vz~2.19) blows up under EVERY config incl. drop/cap/all-gains -> it is a PATHOLOGICAL NOISE SEED (unrecoverable descent transient), NOT the s_ddot pump and NOT control-tunable -> a perception/flow-outlier problem. So the descent-axis "limit cycle" framing was wrong for seed4: it's a noise outlier, not a pump cycle. SMOOTH DESCENT on all recoverable seeds (1-3: peak|vz| 0.5-0.66, no cycle) IS achieved by the 92-config (chi_r=0.65 + kR_xy=4 + p2inf_z=1.0, + KAPPA_MAX_Z/P_z kappa-cap backstop). NET: keep-s_ddot is NOT a robustness downgrade vs drop -- earlier verdict was a seed-selection artifact.

See [[project_combined_barrier_matlab]], [[feedback_hd_uses_measured_sdot]], [[feedback_combined_surface_divergence]].
