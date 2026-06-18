---
name: feedback_cbf_theta_cap
description: "cbf2 QP design (2026-06-09): theta_cap post-QP only (removed from loop); two-phase δ implemented; Phase 1 m2=φ_max; Phase 2 ramp on decode-fail."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a377a083-d63b-447a-908e-12017cf609f1
---

**theta_cap in cbf2 — from CBF_visibility.pdf (2026-06-09).**

QP formulation (Eq. 3 of PDF):
```
θ* = argmin_θ ||θ - θ_d||²   s.t.   |Cr̂_k + [L_ω(θ - θ_curr)]_k + τd_k| ≤ m_k,   |θ| ≤ θ_cap
```

Two constraints, two jobs:
- **FoV box `m_k`**: geometric — keeps marker inside predicted margin (visibility)
- **`|θ| ≤ θ_cap`**: deliverability — PDF Assumption 5: *"θ_cap is the true deliverable tilt (thrust unsaturated)"*

**theta_cap makes the QP "input-aware"** (PDF Section 3): the solution θ* is always within what the SO(3) inner loop can physically track. It is NOT primarily about LK breakage or hover authority — those are downstream consequences. The formal reason is that θ_cap is the maximum tilt the attitude controller can deliver without thrust saturation.

**theta_cap also defines the feasibility boundary** (PDF Section 5): QP declared infeasible if "even the full L_ω θ_cap tilt cannot bring Cr̂ inside" — triggers ring-flow fallback. **With theta_cap moved post-QP (2026-06-09), the QP itself has only the FoV box constraint; the Phase 1 m2=φ_max means the box is always feasible (m_k > 0 guaranteed). The theta_cap clause still applies implicitly: if the QP box solution already exceeds theta_cap, the post-QP clip is the fallback. The `not ok` branch (decode failure → Phase 2) is now the effective "perception impossible" gate.**

**Two-phase δ (PDF Section 4 — IMPLEMENTED 2026-06-09):**
- **Phase 1** (central marker decodes → QP path): `δ_eff = 0`. `m2 = φ_max` only — centroid-only barrier. The marker is deliberately allowed to grow and overflow; using its large half-extent would fire the barrier early during close approach. τδ̇ also excluded (δ held at 0, not the physical fill rate). Phase 2 ramp counters reset on every successful decode.
- **Phase 2** (decode-fail, hysteresis-gated): ramps `δ_eff` from 0 → last-measured ½ptp over `CBF_PHASE2_RAMP_FRAMES=5` frames, after `CBF_PHASE2_HYSTERESIS=3` consecutive fails. Tightens the magnitude-clamp fallback using per-axis headroom `m2_p2 = φ_max − δ_eff`. PDF constraint: ramp must be fast enough that δ_eff never lags the actual fill rate (`min(alpha + 1/ramp_frames, 1)`). `_lw_delta_prev` / `_lw_Lw2_prev` stashed from Phase 1 for Phase 2 use.
- **δ̇ (loom rate)** is still tracked in Phase 1 for the Phase 2 reference but NOT included in `m2` during Phase 1 (excluded per the two-phase spec).

**Current defaults:** `theta_floor = theta_cap = 60°` → FoV box is the only per-step shaper. floor=cap=60° disables d_min collapse (see [[feedback_fov_cone_clamp_deadlock]]).

**2026-06-09 CHANGE: theta_cap removed from QP iteration loop.**
Previously the cap was applied inside the 10-iteration alternating-projections loop (`for _ in range(10)`), interleaved with the FoV box projections. This caused the cap to interfere with the box projection's convergence: the cap could clip `th` away from the feasible box region, forcing the next iteration to fight back, creating oscillation or stalling convergence in corner cases where the two constraints intersect near-tangentially.

Fix: cap is now applied **once, after the loop**, as post-QP saturation:
```python
# post-QP deliverability cap: applied after box projection so it never
# interacts with the FoV constraint and cannot create QP infeasibility
tn = float(np.linalg.norm(th))
if tn > self._theta_cap:
    th = th * (self._theta_cap / tn)
```
The QP loop is now a pure FoV-box alternating projection (always converges when `m_k > 0`). The theta_cap clip preserves the deliverability guarantee without polluting the convergence of the box projection.

**How to apply:** Don't lower theta_cap below 45° (LK starts breaking). Don't raise above 75° (hover authority collapses). If CBF triggers frequently in normal ops, fix the underlying control, not theta_cap.
