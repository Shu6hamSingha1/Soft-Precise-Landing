---
name: Yaw ASMC has POSITIVE sign pattern (opposite of 3D outer-loop ASMC)
description: The yaw plant $\dot{\alpha}_e = -\dot{\psi}_d + d_\alpha$ carries the control with coefficient $-1$ (vs. the 3D plant's $+\beta$), so the Theorem 2 yaw ASMC law has ALL-POSITIVE signs — unlike the Theorem 1 law's $-\Gamma\sigma$ pattern
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Yaw control law (correct, 2026-04-24):**
$$\dot{\psi}_d = +\gamma_\alpha\sigma_\alpha + \text{sat}(\sigma_\alpha/\varepsilon_\alpha)\kappa_\alpha + \chi_\alpha \alpha_e$$

Matches MATLAB `visualControl_IBVS_adaptive.m:494-495`:
```matlab
u_a = K_ctrl.Gamma_a*sigma_a + sat(sigma_a/K_ctrl.E_a)*kappa_a(idx+1) + K_ctrl.Omega_a*e_a(idx);
```
with name map $\gamma_\alpha ↔$ `Gamma_a`, $\chi_\alpha ↔$ `Omega_a`, $\varepsilon_\alpha ↔$ `E_a`.

**Why the signs flip vs. the 3D law (Theorem 1):**
- 3D plant: $\dot{\boldsymbol{\zeta}}_2 = \mathcal{G}_2[+\beta\boldsymbol{u}_h + \ldots]$, control enters with $+\beta > 0$ → law has $\boldsymbol{u}_\text{sw} = -\Gamma\boldsymbol{\sigma} - \ldots$ (negative).
- Yaw plant: $\dot{\alpha}_e = -\dot{\psi}_d + d_\alpha$, control enters with $-1$ → law must be $\dot{\psi}_d = +\gamma_\alpha\sigma_\alpha + \ldots$ (positive) for negative-feedback.

**Why the $-\dot{\psi}_d$ coefficient in the plant:** `I_R_V = rotz(yaw)` (MATLAB line 200) makes $\mathcal{V}$ yaw-aligned with the UAV. A static target's inertial orientation $\phi_t$ maps to $\alpha = \phi_t - \psi$ in the virtual frame, so $\dot{\alpha} = -\dot{\psi} + \omega_{t,z}$. The $\omega_{t,z}$ term plus body roll/pitch cross-coupling become $d_\alpha$.

**Earlier bug (fixed 2026-04-24):** The tex originally had NEGATIVE signs in `yaw control law: equation` (copied from the 3D pattern); this is an unstable (positive-feedback) Lyapunov derivative. Flipped to positive signs across `control_formulation.tex` and the proof in `supplemental.tex` §S2 "Full Proof of Theorem~2".

**How to apply:**
- Any future edits to the yaw control law must keep all three terms positive. Do NOT "harmonize" the sign with Theorem 1's $-\Gamma\sigma$ pattern — they differ for good physical reason.
- If a reviewer asks why signs differ between the 3D and yaw ASMC laws, the short answer is: the 3D plant has $+\beta$ scaling, the yaw plant has $-1$ scaling; sign pattern must compensate. The main paper does NOT cross-reference the 3D law from the yaw law (we removed "similar to \eqref{adaptive control law: equation}--\eqref{adaptive law: equation}" on 2026-04-24 precisely to avoid this misleading comparison).
