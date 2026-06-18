---
name: project_cbf_visibility_design
description: "Target-visibility CBF design (L_omega camera-plane tilt-QP) — IMPLEMENTED in controller.py as FUNNEL_MODE=cone0|cbf1|cbf2 (default cone); gated behind THETA_FLOOR<60"
metadata: 
  node_type: memory
  type: project
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

The target-visibility guarantee was redesigned (2026-06-06, committed `4e885f9`). Converged design lives in **`PX4_Gazebo/docs/CBF_visibility.pdf`** (+ `.tex`) and **`FUNNEL_CBF_DESIGN.md §0`**.

**The design:** a QP/clamp on the desired body tilt, on the REAL camera plane.
- Barrier `h_k = phi_max,k - |^C r_hat_k| - delta_k` on the measured camera image point.
- Tilt→feature coupling = the **rotational part of the IBVS interaction matrix** `L_omega(^C r_hat) = [[xy, -(1+x^2)],[1+y^2, -xy]]` — exact, **depth-independent**, no virtual frame.
- **Two-phase delta:** 0 while the central (target) marker decodes (centroid-only barrier; lets it overflow as you close in); ramps in from the visible markers on its overflow. Forward-invariant look-ahead (target drift `d` + loom `delta_dot`); `theta_cap` input-awareness.

**Decisions reached along the way (don't re-derive):**
- The optic-flow-dynamics CBF was **abandoned** — calibration vs GT showed the flow (esp. divergence `h_z`) is poorly observed and carries `beta`/`d_h`. See [[feedback-wxy-unobservable-imu-fusion]].
- A V-frame barrier (`|^V r_hat|`) does NOT ensure visibility (a tilt with `^V r_hat=0` gives `^C r_hat = tan theta` off-sensor). Barrier must be on the camera plane.
- `^C r_hat = ^V r_hat + tan theta` is WRONG (de-rotation is a homography, not additive) → deleted; use `L_omega`.
- Optic flow stays on the **3D soft-landing velocity loop** (`h = v/z → 0`, all axes), NOT the visibility barrier.

**Status: `FUNNEL_MODE=cone0` IMPLEMENTED 2026-06-06** in `src/controller.py` at the cone-clamp application (~line 875). It's the deg.-0 directional clamp: limits ONLY the outward (away-from-marker) accel, frees the inward/tangential; marker dir = V-frame centroid `s[:2]` mapped to NED via `Rz(yaw)`. Default `"cone"` (magnitude clamp) is UNCHANGED — cone0 is opt-in. **The image→NED sign/axis map is UNCALIBRATED** — env knobs `CONE0_SWAP`/`CONE0_SIGN_X`/`CONE0_SIGN_Y` (default no-swap,+1); a wrong sign drives the marker OUT — but the default map `SWAP=0,+1,+1` is VALIDATED offline (`tools/calibrate_cone0_sign.py` aligns t_hat with the controller's re-centering I_a, cos=0.84; GT disagrees ~90° but that's the compass start-yaw drift mode 8b, irrelevant to the controller-frame cone0). Live "small accel, watch centroid" test still recommended as the gold check.

**cone0 & cbf1 MATCHED TO THE L_omega DOC 2026-06-06** (controller.py ~line 850): the tilt headroom now comes from the **rotational interaction matrix L_omega at the measured camera centroid** (tangent units, depth-free) — `headroom = min_k m_k/‖L_omega[k]‖`, `m = phi_max − |cr + τ·d| − δ − τ·δ̇`. cone0 = τ=0 (static); cbf1 = τ>0 with drift `d = cr_dot_obs − L_omega·ω` (body-rate-stripped via self._w[:2], EMA-filtered). Reduces to atan(d_min/f) when centred; refines the edge case. The directional a_xy clamp (t̂) still supplies directionality — a calibrated-inertial hybrid; a fully camera-frame θ-QP needs the a_xy↔image-tilt convention (2nd cal gate, deferred). Env: `CBF_TAU`≈0.3, `CBF_DMIN_EMA`≈0.3. **Usage gate (cone0/cbf1):** the headroom only matters with `PLASMC_THETA_FLOOR_DEG < 60` (default 60 pins θ_cone=θ_cap). Test: `FUNNEL_MODE=cone0|cbf1 PLASMC_THETA_FLOOR_DEG=15..30`.

**`FUNNEL_MODE=cbf2` = the EXACT camera-frame θ-QP (2026-06-06, controller.py ~line 913)** — the literal doc QP, retires the lean-magnitude approximation. Solves in image-axis tilt on the **measured C-frame feature anchored at θ_curr**: `θ* = argmin‖θ−θ_d‖² s.t. |ᶜr̂ + L_ω·(θ−θ_curr) + τd| ≤ m, |θ|≤θ_cap` (ᶜr̂=measured camera centroid — NOT V-frame s; θ_curr=Rz(−yaw)·(−R[:2,2]/R₃₃) from attitude; m=φ_max−δ; θ_d=Rz(−yaw)·(a_xy/a_z); a_xy*=a_z·Rz(yaw)·θ*), iterative-projection solve. [Fixed 2026-06-06: first cut wrongly anchored on V-frame s; barrier is C-frame.] **No new cal gate**: both cone0 AND the L_ω·ω drift calibrated to the IDENTITY map (`calibrate_cone0_sign.py` cos·I_a=0.84, `calibrate_cbf1_drift_sign.py` cos 0.85), so body→image is no-swap. cbf2 differs from cbf1 mainly off-centre (binding case near touchdown is ~centred). Two sign-cal tools committed. Supersedes [[fov-cone-clamp-deadlock]].
