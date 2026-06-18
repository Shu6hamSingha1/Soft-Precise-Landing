---
name: MDF-ASMC dual-funnel architecture (no "parallel" wording in paper)
description: The target image funnel and optic-flow funnel are architecturally distinct channels that meet at the cone projection; 2026-04-23 paper-wide cleanup DROPPED the word "parallel" from abstract/contributions — the architecture is described without that adjective
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Architectural facts (unchanged):**

- **Target image funnel** — PPC envelope $\boldsymbol{p}_1(t)$ on feature-point position in the image plane (pixel units). Its role is to **size** the state-dependent attitude cone of the funnel-margin cone clamp, which clips the commanded acceleration.
- **Optic-flow funnel** — PPC envelope $\boldsymbol{p}_2(t)$ on the optic-flow error $\boldsymbol{h}_\text{e}$ (rad/s, 3D). Stabilised by the leakage-type adaptive SMC. Its role is to **generate** the commanded acceleration.
- **Signal flow (accurate):** optic-flow funnel → ASMC law → commanded acceleration → cone projection (cone sized by the target image funnel via $d_\text{min}^\text{fov}$) → clipped acceleration → SO(3) inner loop. The two funnels are independent — the target image funnel never sees the ASMC state, the ASMC never sees the cone size directly.
- **Avoid the phrasing "the two funnels meet at the cone projection"** — it is misleading. The optic-flow funnel does not terminate at the cone projection; only its downstream ASMC output does. Prefer: "the ASMC-generated acceleration is clipped in closed form by the cone sized by the target image funnel".

**Paper-wording rule (2026-04-23 lock):**

- **Never use "parallel" to describe the architecture** anywhere in `manuscript.tex`, `control_formulation.tex`, `results.tex`, `supplemental.tex`, or figure/table captions. The word was stripped from the abstract and contribution (a) on 2026-04-23 and must not be reintroduced.
- **Why the change:** "Parallel" overclaims independence. The two funnels do not run as independent signal channels in the block diagram — the cone half-angle reads $d_\text{min}^\text{fov}$ from the target image funnel and the ASMC-generated acceleration feeds into that same cone clamp. A reviewer reading "parallel" will see the coupling at the cone and flag the word as inaccurate.
- **Preferred phrasing:** "dual-funnel architecture", "the optic-flow funnel generates the commanded acceleration, and the target image funnel defines a state-dependent attitude cone that clips it", or simply describe the two roles by what they do without an architectural adjective.

**The cone clamp is the enforcement mechanism, NOT the funnel.**
- $\boldsymbol{p}_1(t)$ is a PPC bound — purely time-driven, no feedback.
- The **funnel-margin cone clamp** is the projection that reads $d_\text{min}^\text{fov}(t) = \min_{i,k}(p_{1_k}(t) - |\,^\mathcal{C}n_{i,k}|)$ and converts it to $\theta_\text{cone}(t)$ used in the geometric clip. (Term locked 2026-04-22, superseding the intermediate "FoV-adaptive cone clamp" label.)
- Do not conflate them. Section III.C should be titled "Target Image Funnel and Funnel-Margin Cone Clamp" (two architecturally distinct ideas), not the old "Visibility Funnel: FoV-Adaptive Cone Clamp" which fused them.

**NOT how it works (legacy Approach-1 framing to avoid):**
- "Target image funnel synthesises a feasible optic-flow command that the optic-flow funnel drives to zero" — that was Approach 1's series chain via barrier-transformed $\boldsymbol{\zeta}_1$.
- "Two funnels coupled through a normalized-error PID" — wrong; the PID is just the $\boldsymbol{h}_\text{d}$ reference generator for the optic-flow funnel. It does not couple the funnels.

**The Normalized Virtual Feature PID is an ancillary component, not the coupling:**
- Takes $\hat{\boldsymbol{r}}_\text{e}$ (virtual image feature error) → produces $\boldsymbol{h}_\text{d}$ (desired optic flow).
- Feeds the optic-flow funnel as its setpoint.
- Is separate from the target image funnel / cone clamp.

**Key terminology to use for the optic-flow funnel:**
- "Three-dimensional optic-flow funnel" or "optic-flow funnel on $\boldsymbol{h}_\text{e}$" — NOT "optic-flow funnel on the descent-rate error" (that suggests 1-D only; the funnel has $p_{2_k}$ for $k\in\{1,2,3\}$ covering lateral-chase AND descent simultaneously).

**How to apply:**
- Use names **target image funnel** (not "visibility funnel") and **optic-flow funnel** (not "soft-landing funnel").
- Describe the architecture WITHOUT the word "parallel". The block diagram and prose convey the two-channel structure on their own.
- Never say "coupled through", "through a reference generator", "synthesises a feasible command for the other", or "the two funnels meet at the cone projection".
