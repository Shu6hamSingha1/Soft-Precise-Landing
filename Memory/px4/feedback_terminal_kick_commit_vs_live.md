---
name: feedback_terminal_kick_commit_vs_live
description: "⭐ TERMINAL-KICK deep-dive (2026-06-27, user-led, GT-FB): the lateral terminal 1/Z kick = the position barrier zeta_r blowing up at the funnel edge (s_e_n=lat/Z breaches p_r via 1/Z) -> infeasible a_u (~1000) -> max tilt -> lateral fly + descent stall -> launch. KEY REFRAME: the loom-commit (open-loop drop) is a STATIONARY-only fix (freeze+fall lands where a moving target WAS); the funnel-ref + k_r (depth-free LIVE terminal recovery) are the MOVING-TARGET (rover) solution. Gate-OFF tests ARE the moving-target regime. Funnel-ref BAKED default-on but REGRESSES stationary gate (8/25->1/25 with k_r=0.3) - un-bake for stationary, keep env-gated for rover."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f1723146-d87f-40d0-9a34-49233fbc4e72
---

**TERMINAL-KICK SESSION (2026-06-27, user-led, GT-FB, h_rd FIXED at -0.42 per user).** Deep structural dive; user has a NEW approach to pursue in a fresh chat.

**The kick mechanism (measured).** Near the deck `s_e_n = lat/(Z·p_10)` blows up (1/Z) → the position barrier `ζ_r = 2·artanh(S_r)` → ∞ at the funnel edge → demands `a_u_xy ≈ 1000` (deliverable ≈17) → actuator answers with **max tilt (→60°)** → tilt simultaneously (a) flings the drone laterally and (b) steals vertical thrust → descent stalls + lateral fly = **launch**. Verified: IC4 reaches lat 0.011 then flies to 0.95 gate-off. It's NOT convergence depth, NOT perception — it's `barrier × 1/Z`.

**Why the barrier breaches:** the lateral DRIFTS out after converging (no velocity feedback — `ζ_h` degenerate, `h_d`=measured ṡ ⇒ `h_e`≈0) + Z shrinks toward the drift ⇒ `lat = Z` ⇒ breach. The lag-floor residual velocity drives the drift.

**The rate race (clean framing):** `ṡ_e_n = (laṫ + |h_rd|·lat)/(Z·p_10)` ⇒ `s_e_n` stays bounded iff **`|h_rd| ≤ k_lat`** (descent rate ≤ lateral convergence rate) ⟺ `ṡ_e_n ≤ 0`. With `h_rd` FIXED (user), the lateral must win it; lag-limited at the deck (`k_lat→0`).

**`σ` ensures `ζ_r → 0` by itself** (combined surface `σ=ζ_h+χ_r·ζ_r`): on `σ=0`, `ζ_h=-χ_r·ζ_r` ⇒ closing flow ⇒ `s_e_n↓`. Stable zero-dynamics `ζ̇_r ≈ -(g_r·χ_r·p_h/2p_10)·ζ_r`, **sign-symmetric** (ζ_r odd, g_r even — works for ζ_r<0 and through zero-crossings). So the convergence-rate dial is **χ_r** (not an explicit k_r), and explicit convergence is **redundant** with the reaching.

**Funnel-ref (un-degenerate ζ_h):** h_d x/y rate `s_dot_meas → p_10·S_r·ṗ_r` (controller.py ~1057). Makes `h_e` a genuine velocity error → recovery authority. Units: `ṡ_e_n = s_dot_meas/p_10` (s_e_n=s_e/p_10, p_10=center/focal CONST) → h_d needs the `p_10` (un-normalize). `dh_d` differentiates the FULL h_d (carries the smooth term, no s_ddot). **BAKED default-on (PLASMC_HD_FUNNEL_REF=1) — but never gate-ON-validated.**

**k_r convergence term (PLASMC_HD_KR, default 0):** `ds_d = p_10·(S_r·ṗ_r − k_r·ζ_r/g_r)` = the back-map `V_ds_e` (prescribe `ζ̇_r=-k_r·ζ_r`, invert via `G_s⁻¹=1/g_r`). User insisted keep it in dh_d. **So this = back-mapping layered on the combined surface** → re-introduces the `G_s⁻¹` starvation (`-k_r·ζ_r/g_r→0` at edge) AND its derivative explodes in dh_d. Gate-OFF n=1: **k_r=0.3 FIXES IC4 (1.18→0.248)** — confirmed by `vlat` flipping +0.30 OUT (k_r=0) → −0.33 IN (k_r=0.3) at 0.5m = active drift-recovery; **but trades IC2 (0.29→0.54), start-height-dependent**; k_r=1.0 DESTABILIZES (s_e_n GROWS +0.2/s, redundant-gain over-drive + dh_d blow-up). REJECTED proportional `Ṡ_r=-k_S·S_r` (drops funnel barrier) and dropping `ζ_r` for `S_r` (user wants the barrier).

**⛔ FULL gate (k_r=0.3, gate-ON, IC1-5 n=5): 1/25 — REGRESSION from frontier 8/25** (destroyed IC1 4→1, IC4 4→0). **The gate-OFF diagnostics MISLED.**

**The loom-commit (LANDING_COMMIT_LOOM=2.8) = the STATIONARY terminal handler.** All 3 commit paths default 0; only loom-commit was ever on. When `∫|h_z|dt≥2.8` AND `s_e_n≤0.35` centered for 3 frames → `in_final_descent=True` → the loop sends **`send_attitude_rate(0,0,0, FINAL_DESCENT_THRUST)`** = ZERO body rates (level) + constant sub-hover thrust = **freeze-and-fall** until ON_GROUND. It AVOIDS the 1/Z kick by exiting before it. Precise = level→no lateral accel→no kick→centered xy frozen (modulo residual lateral velocity coasting); Soft = sub-hover gentle sink. **8/25 comes from converge-then-commit.** The funnel-ref/k_r FIGHT this: they only act in the APPROACH (controller off post-commit), and their aggression spoils the `s_e_n≤0.35` centered commit window.

**⭐⭐ THE REFRAME (user's insight):** open-loop freeze-and-fall is **dumb-but-robust for a STATIONARY target only** — for a MOVING target it lands where the rover WAS (level = no velocity to track it). The rover MUST track live to touchdown (no commit possible). So the terminal 1/Z must be handled IN THE CONTROL (depth-free) = **exactly the funnel-ref/k_r**. **Gate-OFF = the moving-target regime** (run live to touchdown), so the gate-off funnel-ref/k_r wins (IC4 0.248) ARE meaningful for the rover. Verdict splits: **STATIONARY → loom-commit (revert funnel-ref/k_r, restore 8/25); MOVING/rover → funnel-ref + k_r (the live depth-free terminal recovery the commit can't give).**

**CODE STATE at handoff (2026-06-27, cleaned):** `PLASMC_HD_FUNNEL_REF` **UN-BAKED → default-off** (controller.py ~657; restored the `s_dot_meas` 8/25 stationary default), `PLASMC_HD_KR` env-gated **default 0** (back-map convergence term, lines ~1060-1068). So the **stationary default is the working 8/25 config (s_dot_meas + loom-commit)**. For the ROVER: set `PLASMC_HD_FUNNEL_REF=1` (un-degenerate ζ_h) + `PLASMC_HD_KR=0.3` (drift recovery, vlat-IN). New chat = user's NEW terminal-kick approach (these funnel-ref/k_r findings are the prior-art baseline to build on / compare against for the moving target).
