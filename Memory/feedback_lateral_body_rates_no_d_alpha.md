---
name: Lateral body rates do NOT appear in d_alpha
description: After de-rotation through `^V R_C`, the virtual image plane is parallel to the inertial X–Y plane. The orientation α is invariant to body roll/pitch, so its time derivative depends only on the body yaw rate and target angular velocity. Lateral body rates are absorbed by construction and do NOT reappear as a coupling in d_α. Locked 2026-05-04.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** In the MDF-ASMC virtual-frame formulation, the lateral body rates $\boldsymbol{\omega}_{\text{b},xy}$ (roll + pitch rates) are *absorbed by construction* in the de-rotation $\,^\mathcal{V}R_\mathcal{C}$ that defines the virtual frame $\mathcal{V}$. They do NOT reappear in:

- $\dot{\boldsymbol{w}}$ (rotational optic flow derivative)
- $\dot{\alpha}$ (orientation kinematics)
- $d_\alpha$ (residual coupling in the orientation kinematics)

Therefore the residual coupling is $d_\alpha = f(\,^\mathcal{V}\boldsymbol{\omega}_\text{t})$ — function of the **target's** angular velocity only. There is no $\boldsymbol{\omega}_{\text{b},xy}$ term.

**Why.** The virtual frame $\mathcal{V}$ rotates only with the body's yaw (the de-rotation removes roll and pitch). The virtual image plane is consequently always parallel to the inertial X–Y plane. The orientation $\alpha$ is computed from second-order centered moments of the virtual feature points $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$ on this plane. Since the plane's orientation in space depends only on body yaw (and not on roll/pitch), differentiating $\alpha(t)$ produces dependence on:

1. Body yaw rate $\dot\psi_\text{b}$ (= virtual frame's rotation rate).
2. Target angular velocity $\,^\mathcal{V}\boldsymbol{\omega}_\text{t}$.

NOT on body roll/pitch rates $\boldsymbol{\omega}_{\text{b},xy}$. The lateral body rates do change the *camera* orientation but the de-rotation cancels them; they cannot reappear in any quantity computed *post-de-rotation*.

**MATLAB confirmation.** `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m` L292:
```matlab
V_w_a = transpose(I_R_V) * (dx_t(4:6,idx) - [0;0;I_w_c(3)]);
```
Only the z-component of body angular velocity (yaw rate) is subtracted. Lateral body rates are not in the formula. The analytical computation directly verifies that $\boldsymbol{w}$ does not depend on $\boldsymbol{\omega}_{\text{b},xy}$.

**Where this came up (2026-05-04).** A draft prose at §II.B.1 (control_formulation.tex L68) claimed: "The lateral body rates $\boldsymbol{\omega}_{\text{b},xy}$ are absorbed by construction in the de-rotation $\,^\mathcal{V}R_\mathcal{C}$ and reappear separately as a residual coupling in $d_\alpha$ (defined below)." This was wrong. Also §II.B.2 L79 had stale wording: $d_{\alpha}=f(\,^\mathcal{V}\boldsymbol{\omega}_{\text{t}},\,^\mathcal{V}\boldsymbol{\omega}_{\text{b},xy})$ — the second argument is incorrect. Both fixed: L68 stops at "absorbed by construction in the de-rotation"; L79 reads $d_{\alpha}=f(\,^\mathcal{V}\boldsymbol{\omega}_{\text{t}})$.

**Detection (regression check).** Should NEVER appear in active tex:

```bash
grep -nE "reappear.*coupling.*d_\\\\alpha" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "d_\\{?\\\\alpha\\}? *=.*\\\\boldsymbol\\{\\\\omega\\}_\\{\\\\text\\{b,xy\\}\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "lateral body rates.*coupling.*d_\\\\alpha" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
```

Any hit is a regression: lateral body rates do not appear in $d_\alpha$.
