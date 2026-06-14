# Porting brief — CBF (cbf2) + SEN_FUNNEL: PX4 Python → MATLAB

**For:** a Claude session on the Windows machine, tasked with porting the target-visibility
**CBF** and the **SEN_FUNNEL** (position-error PPC funnel) from the PX4/Gazebo Python pipeline
(`PX4_Gazebo/`) into the MATLAB Phase-1 controller (`MATLAB/`).

> ✅ **Status (read first).** The CBF is **validated** — offline (`tools/validate_cbf.py`, 13/13) and in
> SITL on its own metric, *target visibility* (`tools/analyze_cbf_visibility.py`): it keeps the marker
> on the sensor under tilt and bounds the body tilt to its deliverability cap. **Port the current
> implementation described below.** Judge a MATLAB CBF port by visibility (marker stays in frame under
> tilt), never by landing xy — the lateral fly-away is a control-tuning matter the CBF does not own.
> SEN_FUNNEL is implemented + default-on in Python but **still needs tuning** (`FUNNEL_CBF_DESIGN.md` §9).

> ℹ️ **Memory note.** The campaign auto-memory (`~/.claude/.../memory/*.md`) is machine-local and did
> NOT travel with git. This brief + the docs below are the transferred context.

---
## 1. What to read (in this order)

**Design (the "why"):**
1. `PX4_Gazebo/docs/CBF_visibility.pdf` (+ `.tex`) — the CBF theory: the camera-plane QP over body tilt,
   the rotational interaction matrix `L_ω`, the lean→rotation coupling `L̃_ω = L_ω M` (`M=[[0,1],[-1,0]]`),
   the post-QP deliverability cap, the two-phase `δ`, and the direct-to-attitude application.
2. `PX4_Gazebo/docs/FUNNEL_CBF_DESIGN.md` — §0 the converged CBF design; **§9** the SEN_FUNNEL spec
   (back-mapped PPC on `s_e_n`).
3. `PX4_Gazebo/docs/CONTROLLER_PARITY.md` — the **MATLAB↔Python parity map**. The Rosetta stone for the
   port (read its top addendum for the intentional divergences).

**Code (the "what"):**
4. `PX4_Gazebo/src/cbf_visibility.py` — `cbf2_filter(...)`, the **pure, isolated CBF**. Port THIS
   function; no controller-state coupling beyond an explicit `state` dict. Returns `th_safe`.
5. `PX4_Gazebo/src/controller.py` — the call site + integration points:
   - cbf2 call: `~controller.py:1094-1106` (marshals state → `cbf2_filter`).
   - **θ_safe → desired attitude:** the `R_d` block `~:1201-1230` — build `rd3 = [-Rz(yaw)·th_safe, 1]/‖·‖`
     directly (thrust magnitude from the descent loop).
   - **SEN_FUNNEL:** `~:551-583` (the `self._sen_funnel` branch) — `S_s=s_e_n/p_s` →
     `ζ_s=log((1+S)/(1−S))` → `G_s` → `ζ̇_sd = −K_rp·ζ_s − K_ri·∫ζ_s − K_rd·ζ̇_s` →
     `V_ds_d = G_s⁻¹·ζ̇_sd + S_s·ṗ_s`. Params: `gamma_s` (`XIS`), `p_s_0` (`PS0`), `p_s_inf` (`PSINF`).
6. `PX4_Gazebo/tools/validate_cbf.py` — the offline unit harness (parity, `L_ω` fidelity, barrier,
   no-strangle, conventions, Phase-2, direct-attitude). **Mirror its checks in MATLAB** after porting.
7. `PX4_Gazebo/notebooks/cbf_validation.ipynb` — isolated `theta_unsafe → CBF → theta_safe` demo + plots.

**MATLAB targets (the "where"):**
8. `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m` — the **canonical single-run** PLASMC sim.
   Neither the CBF nor the SEN_FUNNEL exists here yet; both are additions to make.
9. `MATLAB/Common/image_feature.m`, `kappa_Solver.m`, `Constants.m`, `InitVar.m`, `UAVDyn.m` —
   feature model, adaptive-gain ODE, camera params, ICs, dynamics.

---
## 2. The CBF, distilled (for the MATLAB port)

Per control cycle, given the measured camera-frame board centroid `cr=(x,y)` (tangent units
`(px−centre)/f`), the current tilt, the yaw `ψ`, and the desired lateral accel `a_xy` (+ `a_z`):

```
L_w   = [[x*y, -(1+x^2)], [1+y^2, -x*y]]          % rotational interaction matrix (DEPTH-FREE)
L_eff = L_w * [[0,1],[-1,0]]                       % tilt->feature coupling (lean->rotation map M)
θ_d   = Rz(-ψ) * (a_xy / a_z)                      % desired LEAN vector (image axes)
θ_cur = Rz(-ψ) * (-R(1:2,3) / R(3,3))              % current lean vector from attitude
m     = φ_max (= centre/focal)                     % Phase-1 box half-width (two-phase δ, §4)
% QP: θ* = argmin ||θ-θ_d||^2  s.t. |cr + L_eff*(θ-θ_cur) + τ d|_k <= m_k
%   solve by alternating projection onto rows of L_eff (10 iters); then cap:
θ*    = θ* * min(1, θ_cap/||θ*||)                  % post-QP deliverability cap (θ_cap=60°)
% apply θ* to the desired attitude DIRECTLY (no a_xy round-trip / LPF):
rd3   = [-Rz(ψ)*θ*, 1];  rd3 = rd3/||rd3||;  build R_d from rd3 (+ heading), thrust mag from descent loop
```

Properties to reproduce in MATLAB (port `validate_cbf.py`): with `L_eff` the predicted feature matches
the true pinhole+attitude projection to <8% near hover; the QP's predicted feature lands inside the box
to ~1e-16; inward/recovery accel is left free (no strangling).

---
## 3. Port gotchas (MATLAB ≠ Python here)

- **Frames.** Python is NED/FRD with `Rz`, downward camera, tangent `(px−c)/f`. MATLAB Phase-1 uses its
  own conventions (`Constants.m`, `image_feature.m`). **Re-derive `θ_cur`, `Rz(±ψ)`, and the `cr` sign
  in MATLAB's frame** — do NOT copy signs blindly. Re-run the `L_ω`-fidelity check (port `validate_cbf.py`
  test 1) against MATLAB's own forward projection to confirm `M`/sign before trusting the barrier.
- **MATLAB has the true desired attitude available** (it builds `R_d` / desired tilt directly), so
  applying `θ*` to the desired tilt is natural — don't reconstruct accel.
- **SEN_FUNNEL replaces the outer PID**, not the SMC. In MATLAB it sits where the pixel-error → `V_ds_d`
  map is. Watch the `S_s` clip / `p_s` floor (§9 notes a back-mapped PPC goes *gentle* at the bound →
  needs hard outlier-containment, not the soft clip).
- **Judge the port by visibility**, not by landing xy: the CBF's only job is keeping the marker on the
  sensor under tilt.

---
## 4. Transfer mechanics (reference)

One repo (`github.com/Shu6hamSingha1/Soft-Precise-Landing`). On Windows: `git pull origin main`
(Git Bash / MSYS2; `export PATH=$PATH:"/c/Program Files/GitHub CLI"` if `gh` is missing). All files
above arrive together. Commit MATLAB changes back from Windows and `git pull` here to sync.
