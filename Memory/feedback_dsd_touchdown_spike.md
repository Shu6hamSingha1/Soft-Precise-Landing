---
name: dsd-touchdown-spike
description: "IC1 hard-impact mechanism = outer-PID desired-flow ds_d spiking near touchdown (1/Z), not perception; per-axis gain relaxation only slides the precision-softness frontier"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

> ⛔ SUPERSEDED/CORRECTED 2026-06-26: The ds_d 1/Z touchdown-spike mechanism relates to the terminal residual, but 'gains cannot break the precision-softness frontier / the genuine lever is the DDS lag fix' is obsolete — the frontier was broken by the combined surface + velocity damping. The PX4 lateral "wall" was a gain-parity bug + the velocity-damping lever (tighten the lateral flow funnel XI2_xy), NOT a perception/architecture/inner-loop-velocity limit; the combined sliding surface σ=ζ_h+χ_r·ζ_r is baked default-on and gives 10/10 bounded landings. The residual is a terminal SOFT velocity kick (≈38ms lag), not a precision wall. See [[feedback_flow_funnel_zetah_works]]. Content below kept as history.

**Per-axis gain investigation of IC1 hard impact (2026-06-02).** User directive: "forget the lag, it's a gain issue — relax individual components."

**Mechanism (solid, per-step diagnosed on the funnel×2 run):** the hard impact is driven by the X-axis **outer-PID desired flow `ds_d_x`**, NOT the measured flow `h_x` (which stays ≤1.1 px throughout — so it is NOT a perception/cal problem). Near touchdown the 1/Z growth makes `s_e_n_x` change fast (0.25→0.95 in 0.4 s) → `ds_e_n_x` spikes → the `K_rd_x` derivative term dominates `ds_d_x` (measured **−18** vs proportional `K_rp·s_e_n`≈8.5). That feeds `h_d` (line ~521) → `h_e = h − h_d` blows past funnel `p` → barrier `ratio=h_e/p` clamps (controller.py:536) → `zeta` saturates at 3.66 → `kappa` runaway → `a_u` explodes → hard touchdown. X explodes first, then z, then y (`kap_x` 0.41→14.9 leads).

**Per-axis levers tried (all IC1, multisine M, funnel×2 base, BOARD_ALPHA0=1.23 CTRL_ZERO_WXY=1):**
- funnel `P2INF` uniform: ×2 best single (0.634 m), ×4 over-relaxed (5.2 m), X-only ×4 worse, Y/Z×2+X×4 worse. ×2 is the funnel optimum; relaxing the first-exploding axis (X) *more* loses its lateral enforcement.
- `K_rd_x` (X derivative): ×0.3 → 0.129 m/s (first sub-0.2 SOFT touchdown ever) but 5.16 m drift/TARGET_LOST; ×0.6 → 3.03 m/0.65. Cutting the derivative removes the kick but also strips the legit lateral tracking damping.
- **`PLASMC_DSD_CLAMP`** (NEW env knob, controller.py — per-axis cap on `V_ds_d_xy`, default 0=off, committed): clamp=3 → 0.068 m/s SOFT but 4.34 m drift; clamp=6 → 0.597 m precise but 0.372 m/s hard; clamp=4.5 → 7.64 m/2.33 (variance draw).

**Two conclusions:**
1. **Precision–softness frontier (reconfirms [[feedback-precision-softness-frontier]]):** every relaxation that tames the touchdown `ds_d` spike trades hard-impact for lateral drift, because the 1/Z amplifies the *legitimate* late correction too — you cannot cap the kick without capping the real correction. Tight→soft/drift, loose→precise/hard, monotonic. Gains slide ALONG the frontier; its best-balance point (~0.6 m / ~0.3 m/s) ≈ the unclamped controller. No per-axis gain reaches both corners (0.08 m AND 0.2 m/s).
2. **Variance dominates (reconfirms [[feedback-sensitivity-sweep-methodology]], [[feedback-strict-coord-descent-dry]]):** identical funnel×2 config gave 0.634 AND 2.870 m; clamp configs spanned 0.597→7.637 m. Single-run gain comparisons are statistically meaningless here; need n≥5 to rank.

**Bottom line:** the user's "it's a gain issue" hypothesis was tested exhaustively per-axis. The mechanism is real and now precisely located (`ds_d` touchdown spike), but it is intrinsic to the lagged loop in the short 1/Z window — gains cannot break through the frontier, only slide along it. The genuine lever remains the lag fix ([[dds-lag-fix-blocker]]) so the late lateral correction arrives early enough to both correct AND settle. `PLASMC_DSD_CLAMP` left as a documented env knob (default off, behaviour unchanged).

---

**UPDATE 2026-06-02 (late session): term decomposition + DH_D_MAX lever — hard impacts ELIMINATED.**

Per-term decomposition of `a_u` (tools/analyze_explosion_chain.py, n=6 defaults reps) refined the chain. The catastrophic amplifier is NOT ds_d directly — it's the **κ-ODE runaway fed by the dh_d clamp value**:

1. `ds_d` spike (1/Z, as above) → `dh_d` (smoothed d/dt of h_d) pins at its **±DH_D_MAX=50 clamp**
2. `-dh_d` enters the c-term → `a_u` directly, AND sets `Θ_norm ≈ 50`
3. κ-ODE: `dκ/dt = Θ·N·G·|σ|` with Θ≈50, G≈8 (barrier saturated), |σ|≈3.7 → κ runs to 10–100× κ_0 (decay τ = 1/(N·P) = 33 s — irreversible in-flight)
4. `a_u ⊇ Θ·sat(σ/E)·κ` → **400–7000 m/s²** → hard impact (rel_vel up to 9.5 m/s)

**`PLASMC_DH_D_MAX=5.0`** (50→5, physical level per controller.py's own comment) breaks steps 2-4 without capping the correction authority (h_d / funnel / SMC response intact — unlike DSD_CLAMP which caps the correction itself). n=5 IC1 result vs n=6 defaults baseline:

| | defaults (DH_D_MAX=50) | DH_D_MAX=5 |
|---|---|---|
| rel_vel | mean 4.1, **max 9.5** m/s | mean ~0.3, **max ~0.4** m/s |
| κ end | 4.5–98 (runaway) | 0.16–3.5 (bounded) |
| a_u peak | 373–7051 m/s² | 9–33 m/s² |
| xy_err | mean 1.7 m | mean ~1.9 m (unchanged, within variance) |

Softness fixed; precision still lag-limited (~1–3 m, unchanged). This is the first knob that removes the explosion without the drift penalty.

**IC2-5 gate result (2026-06-03, two-arm, n=8/arm):** DH_D_MAX=5 **passes** — xy 5.84 vs defaults 5.38, vel 1.13 vs 0.74, all within variance. Safe to adopt as default for the multisine-cal era. BUT both arms collapsed at IC2-5 (~5–6 m xy vs historical old-cal 0.7–2 m) → see [[multisine-cal-ic25-collapse]]: the cal regression at off-center starts is a separate, bigger problem that no DH_D_MAX setting affects (the explosion mechanism doesn't even fire there — drone lands too far from marker for the 1/Z spike).
