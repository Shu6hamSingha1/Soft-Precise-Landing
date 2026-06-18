---
name: $h_\text{rd}$ sign convention — must be negative for descent
description: Locked 2026-05-09. The constant desired descent optic flow $h_\text{rd}<0$ in this paper's convention. With $\,^V z_t>0$ (depth from camera to target, positive in NED V-frame) and descending UAV ($\,^V v_{b,z}>0$ in NED), the optic flow $h_z = \,^V v_{t/b,z}/\,^V z_t = -\,^V v_{b,z}/\,^V z_t < 0$. So $h_\text{rd}<0$ commands descent; $h_\text{rd}>0$ would command ascent. Table S1 lists $h_\text{rd}=-0.42$ s$^{-1}$. Don't claim "$h_\text{rd}>0$" or invoke NED/ENU as the cause — the sign follows from the optic-flow definition $h=v/z$ and the descent direction, not from frame labeling.
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

**$h_\text{rd}<0$ achieves descent in MDF-ASMC.** Listed value: $h_\text{rd}=-0.42$ s$^{-1}$ in Table~S1.

## Why

Optic flow definition: $\boldsymbol{h} = \,^\mathcal{V}\boldsymbol{v}_{t/b}/\,^\mathcal{V}z_t$.

With:
- $\,^V z_t>0$ (depth from camera to target, positive when UAV is above target)
- UAV descending: $\,^V v_{b,z}>0$ (NED, V-frame Z-axis aligned with NED Z = down)
- Target stationary at touchdown: $\,^V v_{t,z}=0$
- Therefore: $\,^V v_{t/b,z} = 0 - \,^V v_{b,z} < 0$
- Thus: $h_z = \,^V v_{t/b,z}/\,^V z_t = (\text{negative})/(\text{positive}) < 0$

So $h_\text{rd}<0$ is the descent command; $h_\text{rd}>0$ would command ascent.

## How to apply

1. **Main paper §III-A1**: write "$h_\text{rd}<0$" (corrected 2026-05-09 from prior "$h_\text{rd}>0$").
2. **Table S1**: $h_\text{rd}=-0.42$ s$^{-1}$ matches the descent direction.
3. **Don't justify the sign via NED-vs-ENU**: optic flow $h=v/z$ is a definition; the sign of $h_\text{rd}$ for descent is determined by the descent kinematics, not the frame label.
4. **Don't invent post-hoc justification text** like "negative because UAV approaches from above" — the user explicitly rejected that framing 2026-05-09. Just state the sign.

## Related conventions

- `feedback_hat_boldsymbol_ordering.md` — notation conventions.
- `feedback_image_parameter_driven_terminology.md` — three image parameters $\boldsymbol{s},\alpha,\boldsymbol{h}$.
- `project_2026-05-06_section_ii_iii_lock.md` — §II/§III layout where $h_\text{rd}$ first appears.
