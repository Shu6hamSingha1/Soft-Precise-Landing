---
name: feedback_theta_norm_klt_drift
description: "θ_norm fully characterized (2026-06-10). θ_norm=‖Θ‖_F drives dκ/dt; ~99% cross(dw,s). TWO spike sources: (1) sustained off-screen-KLT drift [removed by img_data KLT-bounds guard], (2) DOMINANT residual = frame-jump dw artifact (finite-diff of frame-held w_i ÷ control-dt → 252 rad/s²). θ_norm is CONTAINED DOWNSTREAM by the κ-cap + P-leakage, NOT eliminated — it still spikes ~480 (baked) up to 1226 (narrow E_z). DO NOT source-fix dw: an image-rate dw rewrite was tried + REVERTED (net-negative, dw feeds the load-bearing c feedforward → more κ-runaways)."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a377a083-d63b-447a-908e-12017cf609f1
---

**θ_norm = ‖Θ‖_F drives the κ-ODE growth (`dκ/dt = θ_norm·N·G·|σ| − N·P·κ`).** Regressor: `Θ = [vector | I₃]`, `vector = −c + S·dp − G⁻¹·(Ω·ζ)`, `c = cross(dw, s[:3]) + cross(w, cross(w, s[:3])) + 2·cross(w,h) − dot(…,e3)·h − dh_d`. **Decomposition (2026-06-10) proved `cross(dw, s[:3])` is ~99% of θ_norm at a spike** (e.g. 1209 of 1226).

## θ_norm spikes have TWO sources
1. **Sustained off-screen-KLT drift** (the original θ=616 case). `MARKER_KLT_MAX_STEPS=20` let KLT extrapolate corners off-screen → centroid `s[0]=3.15 rad` (FoV max 0.889) AND a bogus `w_z=4.54 rad/s` (corner lstsq on off-screen pts) → `cross(dw,s)`≈689. **REMOVED** by the `img_data` KLT-bounds guard (stop KLT when any corner exits the image): `_in_bounds` check → reset `_lk_step_count=0`, `_prev_aruco_pts=None`. This only covers the *multi-frame* off-screen path.
2. **Frame-jump `dw` artifact (the DOMINANT residual — present even with s in-frame).** `dw = (w_i[k]−w_i[k−1]) / control_dt`, but `w_i` is **frame-held** (img_data ~42 Hz, control ~125 Hz), so dividing each frame-jump by the ~8 ms control step over-amplifies it ~3× → `|dw|` up to **252 rad/s²** (unphysical). At touchdown (fast close-range flow), `cross(dw,s)` spikes θ_norm to **~480 at the baked config, up to 1226 when E_z is narrowed**. The KLT guard does NOT touch this.

## θ_norm is CONTAINED DOWNSTREAM, not eliminated
At the **baked config** trial-49 rode θ-spikes of 480/319 with **zero κ-runaways** (κ_z≤1.6 < cap). Why the spikes don't cascade: they're **brief** (median θ≈2, so they barely move the integrated κ); `P_z=5` keeps `κ_eq ∝ 1/P` low; and `KAPPA_MAX_Z=3.0` is a **backstop that didn't even bind**. The catastrophic θ=1226 → κ-runaway only appeared *because* E_z=0.5 narrowed the boundary layer. **This corrects the earlier inverted framing** ("κ_max=3 was a band-aid, the KLT check is the real fix"): the **cap + P-leakage downstream containment IS the correct approach**, and source-fixing θ_norm is neither necessary nor safe.

## ⛔ DEAD-END: do NOT source-fix θ_norm via dw / w_i
An **image-rate `dw`** rewrite (differentiate over the real inter-frame interval + **hold between frames**) was implemented + tested 2026-06-10. **Reverted** — it made things WORSE: **3/5 κ-runaways vs 1/5 at E_z=0.5**, mean vel 8.5 vs 4 clean reps. A NOTE is left in `controller.py` at the dw block. Don't re-attempt this, nor an LPF-on-`w_i` source-fix.

**WHY it failed — the precise mechanism (`σ` was UNCHANGED, ~3.7 in both runs):** the κ-ODE `dκ/dt = θ·N·G·|σ| − N·P·κ` is an **INTEGRATOR** — it accumulates *sustained* θ and barely registers brief spikes (a 1-frame spike adds ≈ `θ·N·G·|σ|·8ms` ≈ nothing). The old spiky `dw` gave **brief** θ spikes (high peak ~1226, low **mean ~2.7**) the integrator harmlessly ignored. The image-rate fix's **HOLD** converted those brief spikes into a **sustained-moderate** `cross(dw,s)` → θ **mean jumped ~6× (2.7→17.3)** and frames-with-θ>50 went **0→165**, even as the **peak dropped** (1226→683). It lowered the peak (which the integrator ignores) and **raised the integral** (which it accumulates) → MORE runaways at *lower* peak θ. (Clamping/smoothing `dw` also barely helps: the residual 449→392 is carried by the other c-terms — close-range `w_i`≈5, `s`≈4.9.)

**LESSON (general — applies beyond θ_norm):** for an **integrating** adaptive law (κ-ODE, integral windup, leaky integrators), validate a fix against the **time-INTEGRAL / sustained** quantity (∫θ, θ_mean, frames-elevated) — **NEVER the peak**. And a **feedforward** change (here `dw`→`c`) must be judged in **closed loop**, not by replaying old logs: the offline test (peak 1226→449, "looked great") was a FALSE POSITIVE because it replayed the *old* trajectory and so couldn't see the changed `c` alter the dynamics (descent 17s→11s) and amplify the *sustained* θ. Open-loop-improvement-that-destabilizes-the-closed-loop.

## Status & how to apply
- **Trial 49** (KLT-bounds fix) IC1 n=5: θ_norm contained, but **0 SP, 2 TL + 1 hover** — and those failures are **NOT θ-driven** (lateral-drift TL + descent-bootstrap hover). θ_norm is a contained artifact, not the binding failure.
- If θ_norm spikes again: it's expected at touchdown; **let the cap + P-leakage contain it**. Don't rewrite `dw`. Check the *actual* failure (lateral drift / descent) instead. `DH_D_MAX` stays 50. See [[feedback_descent_bootstrap_fix]], [[feedback_dterm_outer_funnel_analysis]], and `docs/PARAMETER_ANALYSIS.md` §2.
