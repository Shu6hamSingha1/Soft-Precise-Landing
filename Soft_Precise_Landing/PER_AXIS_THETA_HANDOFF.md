# Per-Axis θ — Windows Transfer (MATLAB port + manuscript edits)

**Purpose.** PX4/Python validated the per-axis regressor-norm ASMC change (`PLASMC_THETA_PER_AXIS`).
This doc is the turnkey spec to (A) port it to the MATLAB canonical controller and (B) update the
manuscript — both done on Windows. The Lyapunov proof is already written:
`Soft_Precise_Landing/Drafts/PER_AXIS_THETA_PROOF.md`.

**What changed (one line).** The switching gain and κ-ODE driver use the **per-axis regressor row-norm**
`θ_k = ‖row_k(Θ)‖ = √(v_k²+1)` (a 3-vector) instead of the shared scalar Frobenius norm `‖Θ‖_F`.
`θ_k ≡ ‖Θ‖_F` recovers the published law exactly → strict generalization. It is the **tight** per-axis
disturbance bound; the shared scalar over-bounds every axis by the worst row, which is what let the
lateral position-barrier `ζ_r` explosion detonate the z switching term (the collateral z over-brake).

**PX4 validation (the evidence to cite).** GT-FB IC1–5: per-axis θ → **9/10 sub-meter, 0 fly-aways,
0 stalls** (best of campaign; shared-θ baseline 8/10, 1 stall). Mechanism confirmed: terminal `|a_u_z|`
stays 3.6–7.8 even when `|a_u_xy|` explodes to 2995 (shared-θ fly-aways detonated z to 92–320).
[n≥3 confirmation: PENDING — fill in from `run_logs/ic_gate_thetaperaxis_n3.log` before baking the paper.]

---

## A. MATLAB port — `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m`

The change is 3 edits + 1 in `kappa_Solver.m`. Mirror the Python exactly (always pass a 3-vector θ so
the scalar case is just `Theta_norm*ones(3,1)` — no branching needed downstream).

### Edit 1 — after the `Theta` build (currently lines 802–804)
```matlab
    Theta = [- c + S_2(:,:,idx)*dp_2(:,idx) ...
        - G_2(:,:,idx)\chi_zeta_aug, eye(3)];
    Theta_norm = norm(Theta,'fro');                 % keep: scalar, for logging/compat
    % PER-AXIS regressor norm: theta_k = ||row_k(Theta)|| = sqrt(v_k^2 + 1).  Tight per-axis bound;
    % decouples the axes so the lateral zeta_r blow-up can't detonate z (see PER_AXIS_THETA_PROOF.md).
    Theta_perax = sqrt(sum(Theta.^2, 2));           % 3x1
    if THETA_PER_AXIS                                % global flag (mirror PLASMC_THETA_PER_AXIS)
        theta_ctrl = Theta_perax;                   % 3x1 per-axis
    else
        theta_ctrl = Theta_norm * ones(3,1);        % 3x1 replicated scalar == published law
    end
```
(Declare `global THETA_PER_AXIS;` near the other override globals, default `false`/`[]` → scalar path.)

### Edit 2 — κ-ODE driver (currently line 808)
```matlab
    u_kappa = [sigma(:,idx); theta_ctrl];           % was [sigma; Theta_norm] (4x1) -> now 6x1
```

### Edit 3 — switching term (currently line 819)
```matlab
    u_sw = -K_ctrl.Gamma*sigma(:,idx) ...
           - theta_ctrl .* (diag(sat(K_ctrl.E\sigma(:,idx)))*G_2(:,:,idx)*kappa(:,idx+1));
```
(`diag(sat(...))*G*kappa` is a 3-vector since `G_2` is diagonal; `theta_ctrl .*` applies it per-axis.
With `theta_ctrl = Theta_norm*ones(3,1)` this is identical to the old `Theta_norm*diag(sat)*G*kappa`.)

### Edit 4 — `MATLAB/Common/kappa_Solver.m`
```matlab
function [dkappadt] = kappa_Solver(~, kappa, X, K, G)
    N = K(1:3,:);  P = K(4:6,:);
    sigma = X(1:3);
    theta = X(4:6);                                  % was X(4) scalar -> now 3x1 per-axis
    dkappadt = theta .* (N * G * abs(sigma)) - N * P * kappa;   % element-wise theta
end
```
(`N,G` diagonal → `N*G*abs(sigma)` is 3x1; `theta .*` makes it per-axis. Scalar-replicated θ reproduces
the old `Theta_norm * N*G*abs(sigma)` bit-for-bit.)

### Validate the MATLAB port
- **Parity check:** with `THETA_PER_AXIS=false`, the run must be identical to current `main` (scalar path).
- **Effect check:** with `THETA_PER_AXIS=true`, re-run the IC1–5 / vdf_params validation; expect the z
  switching to decouple from lateral ζ_r (cleaner terminal, no regression on the 25/25 SP set). The
  proof guarantees the SAME UUB, so this is a no-regression + decoupling test, not a re-tune.

---

## B. Manuscript edits — `control_formulation.tex` (+ supplemental if it restates the law)

The proof in `Drafts/PER_AXIS_THETA_PROOF.md` is written in manuscript notation; lift directly.

1. **Define the row-norm + bound** (near eq. `sigma derivative: equation 2`, where `θ·d̄` appears): add
   `θ_k ≜ ‖e_k^⊤θ‖`, and the tight per-axis bound `|(θd̄)_k| ≤ θ_k‖d̄‖ ≤ θ_k d̃`  (★).
2. **Control law** (eq. `adaptive control law: equation`): `‖θ‖ → Θ_⋆ ≜ diag(θ_1,θ_2,θ_3)`:
   `u_sw = −Γσ − Θ_⋆ sat(E⁻¹σ) G_h κ`.
3. **Adaptive law** (eq. `adaptive law: equation`): `κ̇ = Θ_⋆ N G_h |σ| − N P κ`.
4. **Theorem 1 proof:** replace the scalar `‖θ‖` by `θ_k` and collect **per axis** (Section 3 of the
   proof doc). The decisive point: `θ_k` multiplies BOTH the axis-k switching dissipation AND the axis-k
   κ-driving term, so the cross-cancellation in `V̇` holds per-axis and the inequality collapses
   line-for-line to the current one — SAME `V`, SAME `φ₁`, SAME UUB radius `ϑ`. The published scalar law
   is the conservative special case `θ_k ← ‖θ‖_F ≥ θ_k`.
5. **Remark (new):** per-axis regressor norm is the tight disturbance bound and decouples cross-axis
   switching; cite the empirical decoupling (terminal `a_u_z` bounded while `a_u_xy` explodes) and the
   `0 fly / 0 stall` GT-FB result. Note it removes the *collateral* z over-brake but not the lateral
   `s_e_n` 1/Z runaway (separate lever) — keep the claim honest (Corollary 1 / Theorem 3 unchanged: they
   use only σ-UUB + CBF forward-invariance, neither depends on the switching-gain form).
6. **Supplemental:** if S2 restates `u_sw`/`κ̇`, apply the same `‖θ‖→Θ_⋆` substitution there.

---

## Order of operations
1. PX4: finish n≥3 confirmation → bake `PLASMC_THETA_PER_AXIS=1` default-on → commit/push (Ubuntu).
2. Windows `git pull`. Apply Part A to MATLAB; parity-check (flag off ≡ main), then effect-check (flag on,
   IC1–5 / 25-SP). 3. Apply Part B to the manuscript using `Drafts/PER_AXIS_THETA_PROOF.md`. 4. Fill the
   n≥3 number into "PX4 validation" above and into the manuscript remark.

Cross-refs: proof `Drafts/PER_AXIS_THETA_PROOF.md`; memory `Memory/px4/feedback_theta_per_axis_decoupling.md`,
`Memory/px4/feedback_terminal_root_lateral_zeta_r.md`.
