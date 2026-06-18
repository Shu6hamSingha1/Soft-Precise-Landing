---
name: feedback_inner_loop_velocity_thread
description: "Inner-loop velocity / flow-ceiling thread (2026-06-15/16): the LK flow ceiling (~1 rad/s, saturating sensor) is BEATABLE (centroid-rate -> h/true 0.37->0.96) but accurate velocity alone does NOT cure the lateral wall; both pillars (inner velocity + outer authority) co-bind; a_u points OUTWARD (h_d rotation-FF-dominated); c_h correction smoother+non-regressing but not the wall-breaker. New env knobs + the combined-sliding-variable as the remaining lever."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 3553f974-8018-4a1e-ba00-93060aa8d9e4
---

The deepest narrowing of the lateral wall yet — chains off [[feedback_lateral_overshoot_root]].
Root: `s_e` converges inside `p_1` then OVERSHOOTS; `h_e` converges but `s_e_n` doesn't, because
the SMC tracks the MEASURED flow `h` which UNDER-REPORTS the true velocity → the cascade's
`ṡ_e_n=ds_d` assumption breaks.

## The LK flow ceiling is a SATURATING SENSOR (~1 rad/s) — and it's beatable
`|h|/|true centroid rate|` falls 1.16→0.15 as speed rises (`|h|` plateaus ~1.0) — NOT a constant
bias (which a cascade absorbs) but a hard **dynamic-range ceiling** → positive feedback
(faster→blinder→less brake). Ceiling is at the **LK/lstsq SOURCE**, not the filter (raw p95 ≈
filtered p95; filter only clips noise spikes). `maxLevel=3` LK has ~13 rad/s displacement
capacity, so it's the linearization under-estimate of sustained large motion, not the window.

## Cascade theory (why h_e converges, s_e_n doesn't)
A position-over-velocity cascade converges on TWO pillars: (1) accurate inner velocity sensing,
(2) constant-gain accurate outer position loop. Ours breaks BOTH: (1) LK saturates above ~1 rad/s;
(2) the back-mapped barrier's outer gain COLLAPSES at the boundary (`G_s⁻¹→0`, 4–7×) instead of
staying constant. `h_e` is DIRECTLY actuated (SMC drives the measured `h`→`h_d`) so it always
looks met; `s_e_n` is only reachable THROUGH the corrupted `h`, so it fails.

## SITL A/B RESULTS (IC2, all this thread)
- **`FLOW_CENTROID_RATE` (lever 2, NC85):** blend the accurate detected-centroid rate `d(s[:2])/dt`
  into `h[:2]` (both V-frame). **`CR=1.0` made `h` track truth (h/true 0.37→0.96) — ceiling BEATEN —
  but still 5/5 TL + noisier** (he_std 0.82→1.03, differentiation noise). `CR=0.5` = sweet spot
  (TL 2/4, smoothest). **Accurate velocity alone is NECESSARY but NOT SUFFICIENT.**
- **Both pillars (`CR=0.5`+`SEN_RECOVERY_K`, NC86):** cut TL 5/5→2/5 but NOT cured; arrival vel ~3.0
  + breach @ ~2.2 m unchanged; REC added nothing over CR alone. **a_u-OUTWARD diagnostic:** `a_u`
  points outward ~99% (NOT braking), `w_u` mostly unsaturated → the brake is **MIS-DIRECTED, not
  under-powered**, because `h_d` is dominated by the rotational FF `cross(w_i,s)` in the overshoot.
- **`PLASMC_CH_CLEAN` (consistent c_h, NC87):** drops `w×s` from `h_d` + clean c-term. SMOOTHER
  (he_std 0.61→0.54, w_u sat 22%→7%), xy mean 8.1→5.5, **NON-regressing** (vs MATLAB partial
  C_SIMPLE SP 9→2 → confirms that regression was the convention-mixing). a_u 32%→41% inward (modest).
  **CORRECTION: a_u-outward was specific to the CR+REC config; on the BAKED baseline a_u is already
  32% inward → rotation-FF domination is NOT the binding limit.** c_h is a correctness+smoothness
  keeper, not the wall-breaker.

## STANDING CONCLUSION
Every signal-side fix is now ruled out as a SOLE cure: KP/funnel/KD (NC80-83), N_xy (NC84),
centroid-rate (pillar 1), SEN_RECOVERY_K (pillar 2), c_h (rotation-FF). The lateral wall needs a
STRUCTURAL change. The one untried lever: **the combined sliding variable `σ = ζ_h + λ·ζ_s`** —
gives the accurate POSITION error DIRECT SMC authority (not routed through the corrupted velocity
loop / rotation-dominated `h_d`). Best form: `σ = ζ̇_s + λ·ζ_s` with `ζ̇_s` from the centroid rate
(accurate) — escapes the ceiling AND the cascade-decoupling at once. It's a Lyapunov re-derivation
(do default-off, MATLAB first). See [[feedback_plasmc_two_task_framework]].

## NEW ENV KNOBS (all default-off / behavior-preserving; commits local — see git log)
- `FLOW_CENTROID_RATE` ∈[0,1] (controller, 19604c2) — blend centroid-rate into h[:2]. CR=0.5 best.
- `PLASMC_CH_CLEAN` (controller, 75e8765) — consistent c_h: clean c-term + drop w×s from h_d.
- `PLASMC_SEN_RECOVERY_K` (controller, 6ee0f2c) — escalating funnel-breach recovery (outside G_s⁻¹).
- `PLASMC_DSD_LAT_MAX` (controller, ca920f6) — ds_d magnitude cap (diagnostic; KP is the real lever).
- `FLOW_KF_Q` / `FLOW_KF_R` (img_data, 15cd430) — flow-KF bandwidth (marginal; filter isn't the ceiling).
- `FLOW_COND_REJECT` (img_data, 294bfb4) — condition-aware corner outlier rejection = SMART EKF-flow
  smoothing (kills lstsq garbage spikes WITHOUT bandwidth loss; the right answer to "smooth the noisy
  EKF flow" — blanket FLOW_Q_HTR↓/FLOW_R_CORNER↑ would lower the ceiling).
- `PLASMC_PS0_MARGIN` (controller, e8c2a23) — PS0 now resolution-derived: FoV-edge(=1.0 in s_e_n) + margin.

## REFACTOR: theta_cap moved OUT of the CBF (d3dfbaa)
`theta_cap` (deliverable-tilt saturation) was inside `cbf2_filter` — a deliverability concern, not a
visibility constraint. Moved to controller `_attCtrl` (caps `th_safe` + `I_a[:2]` by one scale,
Phase-1-gated = old behavior). `cbf2_filter` is now a PURE visibility QP. Synced all 11 callers +
`validate_cbf.py` (12/12, parity bit-for-bit) + notebook. Removed validator test 7c. ⚠ touches the
CBF chat's domain. CoG-FF (GAMMA_COG) PX4 port = NOT APPLICABLE (rate-mode; PX4 rate-I rejects it;
see [[feedback_cog_adaptive_feedforward]]).
