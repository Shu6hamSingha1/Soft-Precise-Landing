---
name: feedback_cbf_lw_rotation_bug
description: "cbf2 target-visibility CBF — current implementation: pure cbf2_filter, L_eff=L_w·M coupling, theta_safe->rd3 direct attitude, two-phase δ. Validated offline 13/13 + SITL visibility."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 8ca75061-5bff-412a-a7d6-dfb5d6cb945a
---

**cbf2 target-visibility CBF — current implementation (the approach to port / reuse).**
The `FUNNEL_MODE=cbf2` block lives in the pure function `src/cbf_visibility.py::cbf2_filter` (the live
controller calls it). It is a camera-plane QP over body tilt that keeps the marker on the sensor.

**Per cycle**, given measured camera centroid `cr=(x,y)` (tangent `(px−c)/f`), attitude `R`, yaw `ψ`,
desired lateral accel `a_xy`/`a_z`:
- `L_w = [[xy,−(1+x²)],[1+y²,−xy]]` (rotational interaction matrix, depth-free).
- `L_eff = L_w @ [[0,1],[-1,0]]` — the tilt→feature coupling. `L_w` couples the body **rotation-axis
  rate**, but the QP variable `θ` is the **lean-direction** vector (`θ_d=Rz(−ψ)·a_xy/a_z`); a lean +x is
  a rotation about +y, so `ω=Mθ`, `M=[[0,1],[-1,0]]`. Use `L_eff`, not `L_w`.
- `θ_cur = Rz(−ψ)·(−R[:2,2]/R33)`.
- QP: `θ* = argmin‖θ−θ_d‖² s.t. |cr + L_eff·(θ−θ_cur) + τd|_k ≤ m_k`, solved by alternating projection
  onto the rows of `L_eff`; then post-QP deliverability cap `θ* ← θ*·min(1, θ_cap/‖θ*‖)`, θ_cap=60°.
- Two-phase δ: Phase 1 (marker decodes) `m=φ_max` centroid-only (always feasible); Phase 2 (decode-fail,
  hysteresis 3, ramp 5 frames) tightens `m=φ_max−δ_eff−τδ̇_eff`.
- **Apply θ* directly to the desired attitude:** `rd3=[−Rz(ψ)·θ*, 1]/‖·‖`, build `R_d` from it (thrust
  magnitude from the descent/loom loop). `a_z` cancels in `−I_a/‖I_a‖`, so this skips the accel
  round-trip + its LPF and lands the barrier's hard bound exactly on the commanded attitude.

Env: `CBF_LW_ROT` (the L_eff coupling, default 1), `CBF_RD3_DIRECT` (direct attitude, default 1),
`CBF_TAU`, `CBF_DMIN_EMA`, `CBF_PHASE2_HYSTERESIS`, `CBF_PHASE2_RAMP_FRAMES`, `PLASMC_THETACAP_DEG`.

**Validated.** Offline `tools/validate_cbf.py` 13/13 (parity, L_eff fidelity vs an independent
pinhole+attitude camera <8% near hover, barrier-in-box ~1e-16, no-strangle, conventions, two-phase,
direct-attitude). SITL `tools/analyze_cbf_visibility.py` (IC2 4-cell, n=5): on the CBF's OWN metric —
**target visibility, NOT fly-away** — the CBF keeps the marker in-FoV under tilt and bounds body tilt to
the cap. **Fly-away is a control-tuning issue the CBF can't fix; judge the CBF by visibility only.**

Isolated demo: `notebooks/cbf_validation.ipynb` (theta_unsafe→CBF→theta_safe + plots). MATLAB-port map:
`docs/CBF_SEN_MATLAB_PORT.md`. Theory: `docs/CBF_visibility.pdf`, `docs/FUNNEL_CBF_DESIGN.md` §0.
Related: [[project_cbf_visibility_design]], [[feedback_cbf_theta_cap]], [[feedback_lateral_overshoot_root]].
