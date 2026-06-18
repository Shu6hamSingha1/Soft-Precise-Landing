---
name: MDF-ASMC block diagram structure (block_diagram_v3.pdf)
description: Reference for the active block diagram in the manuscript. Documents the colour-coded groups, every block with its equation reference, and the signal flow. Source of truth for prose that describes the controller architecture; updated 2026-05-04 to reflect epoch-2 vocabulary, new Virtual Image Position block, and rotational optic flow `w` output.
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Active file in manuscript:** `Soft_Precise_Landing/Figures/block_diagram_v3.pdf` (static PDF, included by `control_formulation.tex`). Latest regen: 2026-05-04. The legacy TikZ source `Soft_Precise_Landing/block_diagram.tex` is in `Obsolete/` and is no longer compiled.

## Colour-coded groups

| Colour | Group | Role |
|---|---|---|
| Light teal | **Quadrotor Plant** | Camera + state estimator + rigid-body dynamics |
| Yellow / dashed border | **Image Parameters** | Computational stack from raw image to the controller-input parameters |
| Pink | **Outer-Loop Control** | Image-error formation + Virtual Image Point Control (PID) + Optic Flow PPC |
| Orange | **Optic Flow Funnel** | (sub-group inside Outer-Loop) PPC + Error Transformation |
| Green | **Optic Flow ASMC** | Sliding surface + adaptive gain + regressor + control law |
| Light blue | **Acceleration Conditioning** | Target image funnel + funnel-margin cone clamp + low-pass filter |
| Red / salmon | **Yaw ASMC** | Sliding surface + adaptive gain + control law + virtual-compass integrator |
| Dark blue | **Inner-Loop Control** | Geometric SO(3) attitude tracker |

**Sub-group labels visible inside the diagram** (group-of-groups structure):
- "Optic Flow Control" — sub-label inside Outer-Loop Control (covers Virtual Image Point Control + Optic Flow Funnel + Optic Flow ASMC)
- "Yaw Control" — sub-label inside Yaw ASMC (covers Yaw Sliding/Adaptive/Control)

## Blocks and their equation references (v3, 2026-05-04 numbering)

### Quadrotor Plant (right side)
- **Camera Pinhole Model** — emits raw image data; feeds the Image Parameters stack.
- **Flight State Estimator** — emits the rotation matrix `R` for the inner loop and inputs to the `^V`-frame projections.
- **Quadrotor Dynamics (1)** — receives `^B T_u` and `^B τ_u`; integrates the rigid-body equation.

### Image Parameters (top, four sub-blocks left-to-right; epoch-2 names)
1. **Image Feature Points** → emits `^C r̂_i` (raw camera-frame feature points; was "Target Image Feature Points")
2. **Virtual Image Orientation (9)** → emits `α` (was "Target Virtual Orientation (7)")
3. **Virtual Image Point** → emits `^V r̂` (centroid; was "Target Virtual Image Points (3)")
4. **Virtual Image Position (2)** → emits `s` (homogeneous augmentation; **NEW block** added in this regen)
5. **Optic Flow (5)** → emits **`h` AND `w`** (translational + rotational optic flow; rotational component `w` is **new** vs. earlier diagrams; was "Optic Flow (2)")

These are *computationally* five sub-blocks. Conceptually the controller-input parameters are still **three** (`s`, `α`, `h`) plus the rotational optic flow `w` (which couples kinematically into the position-error and optic-flow equations); see `feedback_target_image_parameters.md`.

### Outer-Loop Control (pink, middle)
- **α_d ⊖ α** summing junction → `α_e` (orientation error)
- **r̂_d ⊖ ^V r̂** summing junction → `r̂_e` (virtual image point error)
- **Virtual Image Point Control (13)** — input `r̂_e` → output `h_d` (was "Normalized Virtual Feature PID (11)")
- **h ⊖ h_d** summing junction → `h_e` (optic-flow error)
- **Optic Flow PPC (14)** — input `h_e` → output `ζ_2` (was eq 12)
- **Error Transformation (15)** — provides `S_2`, `ṗ_2`, `G_2` to the ASMC (was eq 13)

### Optic Flow ASMC (green, bottom-middle)
- **Optic Flow Sliding Surface (16)** — input `ζ_2`, `G_2`, `S_2`, `ṗ_2` → output `σ` (was eq 14)
- **Regressor Estimation (17)** — input `S_2`, `ṗ_2` → output `θ` (was eq 15)
- **Optic Flow Control Law (18)** — input `ζ_2`, `G_2`, `σ`, `κ`, `θ` → output `^I a_d` (was eq 16)
- **Optic Flow Adaptive Gain (19)** — input `σ` (and `θ` from regressor) → output `κ` (was eq 17)

### Acceleration Conditioning (light blue, right)
- **Target Image Funnel (22)** — input `^C r̂_i` → output `θ_cone` (was eq 20)
- **Funnel-Margin Cone Clamp (23)** — inputs `θ_cone`, `^I a_d` → output `^I a_{d,xy}` (was eq 21)
- **Low-Pass Filter (24)** — combines `^I a_{d,xy}` and `^I a_{d,z}` (z-component bypasses the clamp) → output `^I ã_d` and `^B T_u` (was eq 22)

### Yaw ASMC (red/salmon, left)
- **Yaw Sliding Surface (25)** — input `α_e` → output `σ_α` (was eq 23)
- **Yaw Adaptive Gain (25)** — input `σ_α` → output `κ_α` (was eq 23)
- **Yaw Control Law (25)** — inputs `σ_α`, `κ_α` → output `ψ̇_d` (desired yaw rate; was eq 23)
- **Virtual-Compass Integrator (26)** — integrates `ψ̇_d` → output `ψ_d` (was eq 24)

### Inner-Loop Control (dark blue, bottom)
- **Geometric SO(3) (29)** — inputs `ψ_d` (from Yaw ASMC), `^I ã_d` (from Acceleration Conditioning), `R` (from Plant) → output `^B τ_u` (back to Plant; was eq 27)

## End-to-end signal flow (summary)

```
Camera ──► [Image Parameters]
                │
                ├── ^C r̂_i ─────────────────────► Target Image Funnel ─► θ_cone ─► Cone Clamp
                ├── ^V r̂  ──► r̂_e ─► Virtual Image Point Control ─► h_d ─►(merge w/ h)─► h_e ─► PPC ─► ζ_2 ─► ASMC ─► ^I a_d
                ├── s     ──► (state-feedback into outer-loop blocks via ζ_2/G_2/S_2)
                ├── α     ──► α_e ─► Yaw Sliding/Adaptive ─► Control Law ─► ψ̇_d ─► VC Integrator ─► ψ_d
                └── h, w  ──► h_e (above), w (kinematic coupling in c_h)

Cone Clamp(^I a_d, θ_cone) ─► ^I a_{d,xy} ─► LPF ─► ^I ã_d, ^B T_u
                                                       │            │
                                                       │            └─► Plant
                                                       │
ψ_d, ^I ã_d, R ──► Geometric SO(3) ─► ^B τ_u ─► Plant
```

## Key signals (with units)

| Signal | Meaning | Frame / units |
|---|---|---|
| `^C r̂_i, i∈{1..4}` | image feature points | camera frame, pixels |
| `^V r̂` | virtual image point (centroid of four de-rotated points) | virtual frame, pixels |
| `s` | virtual image position (homogeneous augmentation `[^V r̂; 1]`) | virtual frame, dimensionless |
| `α, α_d, α_e` | virtual image orientation, desired, error | radians |
| `h, h_d, h_e` | translational optic flow, desired, error | rad/s |
| `w` | rotational optic flow (target ω in V minus body yaw rate) | rad/s |
| `ζ_2, G_2, S_2, ṗ_2` | PPC-transformed error and helpers | dimensionless |
| `σ, κ` | translational sliding surface and adaptive gain | dimensionless |
| `θ` | regressor (uncertainty collected for SMC) | dimensionless |
| `σ_α, κ_α` | yaw sliding surface and adaptive gain | dimensionless |
| `ψ̇_d, ψ_d` | desired yaw rate, desired yaw | rad/s, rad |
| `^I a_d` | inertial commanded acceleration (raw) | m/s² |
| `^I a_{d,xy}, ^I a_{d,z}` | lateral / vertical components of `^I a_d` | m/s² |
| `^I ã_d` | filtered + cone-clipped commanded acceleration | m/s² |
| `θ_cone` | funnel-margin cone half-angle | radians |
| `^B T_u` | total rotor thrust | N (along `-Z_b`) |
| `^B τ_u` | body-frame control torque | N·m |
| `R` | UAV body-to-inertial rotation matrix | SO(3) |

## What changed in the 2026-05-04 regen

1. **Vocabulary epoch-2 propagated.** All sub-block labels in the Image Parameters group dropped the "Target" prefix:
   - "Target Image Feature Points" → "Image Feature Points"
   - "Target Virtual Image Points" → "Virtual Image Point" (singular — centroid emphasis)
   - "Target Virtual Orientation" → "Virtual Image Orientation"
2. **New "Virtual Image Position (2)" block** added — emits `s` (the homogeneous-augmented form). Previously `s` was implicit in the diagram; now it has a dedicated computational block.
3. **Optic Flow now emits both `h` and `w`** — the rotational optic flow `w` (introduced in the §II.B reorder + epoch-3 naming) is now a visible signal output of the Optic Flow block.
4. **Outer-loop PID renamed** — "Normalized Virtual Feature PID (11)" → "Virtual Image Point Control (13)" matching epoch-3 of `feedback_image_feature_naming.md`.
5. **Equation numbers shifted +2** across most blocks due to the new equations introduced when §II.B and §III were restructured this session.

## Conventions to maintain in prose

- **Three image parameters** (s, α, h) plus rotational optic flow `w` — see `feedback_target_image_parameters.md`. The diagram shows five sub-blocks computationally but only three controller-input parameters conceptually.
- **Avoid "channel"** when describing signal flow (`feedback_no_channel_word.md`); use "actuation", "command", "loop", or "axis".
- **Image-plane axes** use $\hat{X}_\text{c}, \hat{Y}_\text{c}$ (with hats); 3-D frame axes use $X_\text{c}, Y_\text{c}, Z_\text{c}$ (no hats) (`feedback_image_plane_axis_vs_component.md`).
- **Funnel naming**: target image funnel = $\boldsymbol{p}_1$ on `^C r̂_i`; optic-flow funnel = $\boldsymbol{p}_2$ on `h_e` (`feedback_funnel_naming.md`).
- **Inner loop**: geometric SO(3) tracker (Lee 2010), NOT the Euler-angle PID baseline of §S1 (`supplemental.tex`'s §S1 was rewritten 2026-04-30 to reflect this).
- **No "target" prefix** on feature-point or centroid labels (epoch-2 vocabulary, `feedback_no_landmark_term.md`).

## Related memories

- `feedback_target_image_parameters.md` — three-parameter convention.
- `feedback_no_landmark_term.md` — full feature-point naming table (epoch-2).
- `feedback_image_feature_naming.md` — virtual image point error / Virtual Image Point PID naming (epoch-3).
- `feedback_funnel_naming.md`, `feedback_funnel_symbol_convention.md` — funnel symbol conventions.
- `feedback_no_channel_word.md` — avoid "channel".
- `Obsolete/Soft_Precise_Landing/block_diagram.tex` — legacy TikZ source (no longer compiled).
