---
name: Mathematical-notation definition audit (main paper + supplement)
description: Complete list of mathematical symbols used in manuscript.tex/control_formulation.tex/results.tex/supplemental.tex with their definition status — reference for future notation-consistency passes
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Audit baseline: 2026-04-24 (post-Theorem 2 addition, post-signs-flip, post-abstract/conclusion Theorem 2 cross-refs).

**Frames and rotations.** Fully defined: $\mathcal{I}$, $\mathcal{B}$, $\mathcal{C}$, $\mathcal{V}$, $\,^\mathcal{V}R_\mathcal{C}$ (Fig. 2 caption, supplement §S1-A). Implicitly defined (conventional): $\,^\mathcal{I}R_\mathcal{V}$, $R$ (body-to-inertial), $R_{33}$, $R_d$.

**Body-frame quantities (line 45 of control_formulation.tex).** Fully defined: $\,^\mathcal{B}\boldsymbol{v}_\text{b}$, $\,^\mathcal{B}\boldsymbol{\omega}_\text{b}$, $\,^\mathcal{B}\boldsymbol{F}_u$, $\,^\mathcal{B}T_u$, $\,^\mathcal{B}\boldsymbol{F}_\text{g}$, $\,^\mathcal{B}\boldsymbol{\tau}_u$, $\,^\mathcal{B}\boldsymbol{F}_\text{d}$, $\,^\mathcal{B}\boldsymbol{\tau}_\text{d}$, $m$, $J$, $B_v$, $B_\omega$, $\boldsymbol{c}_v$, $\boldsymbol{c}_\omega$.

**Virtual-frame image features (lines 62–67, 127, 129).** Defined: $\boldsymbol{h}$, $\boldsymbol{s}$, $\,^\mathcal{V}\hat{\boldsymbol{r}}$, $\,^\mathcal{V}\hat{x}_k/\hat{y}_k$, $\,^\mathcal{V}\boldsymbol{r}_\text{t}$, $\,^\mathcal{V}z_\text{t}$, $B_h$, $\boldsymbol{c}_h$, $\boldsymbol{d}_h$, $\,^\mathcal{V}\boldsymbol{v}_\text{t/b}$, $\,^\mathcal{V}\boldsymbol{\omega}_\text{t/b}$, $N$, $\boldsymbol{\hat{r}}_k$.

**Implicit (conventional but not formally introduced — "low" priority for rigor pass):**
- $\,^\mathcal{V}\boldsymbol{\omega}_b$, $\,^\mathcal{V}\boldsymbol{v}_b$, $\,^\mathcal{V}\boldsymbol{a}_\text{t}$ (line 65 inside $\boldsymbol{d}_h$)
- $\,^\mathcal{V}\boldsymbol{\omega}_\text{t}$, $\,^\mathcal{V}\boldsymbol{\omega}_{\text{b},xy}$ (line 154 as arguments of $d_\alpha$)
- $\,^\mathcal{I}\boldsymbol{a}_\text{d}$, $\,^\mathcal{I}\boldsymbol{r}_\text{b}$, $\,^\mathcal{I}\boldsymbol{v}_\text{b}$, $\,^\mathcal{I}\boldsymbol{a}_{\text{d},xy/z}$
- $\boldsymbol{s}_\text{d}$, $\hat{\boldsymbol{r}}_\text{d}$, $\alpha_\text{d}$ (desired values; only their derivatives are given)
- $\boldsymbol{d}_\text{max}$ (Assumption 1)
- $\Pi_{[-\pi,\pi]}$ (angle-wrapping projection)
- $^\vee$ (vee operator, SO(3) isomorphism — supplement line 166)

**Known gaps fixed 2026-04-24:**
1. **$\mathcal{R}$** (camera-sensor resolution) and **$f$** (focal length) — now defined inline at first use in `control_formulation.tex:175` ("$\mathcal{R}=[r_h,r_w]^\top$ the camera-sensor resolution (in pixels) and $f$ the focal length (in pixels)").
2. **$\varepsilon$ vs $\varepsilon_k$ symbol clash** — cone-clamp numerical floor renamed $\varepsilon \to \epsilon_\text{c}$ (`control_formulation.tex:237, 239`), with inline note disambiguating from ASMC boundary-layer widths.
3. **Stale `Assumption~2` cross-reference** — line 222 changed to "Property~1".
4. **Pixel-axis $(u, v)$ notation** — three uses eliminated in favour of dual $(\hat{X}_\text{c}/\hat{Y}_\text{c})$ axes vs $(\,^\mathcal{C}\hat{x}/\,^\mathcal{C}\hat{y})$ components convention; see `feedback_image_plane_axis_vs_component.md`.

**Still unfixed (low priority):**
- **$\hat{\boldsymbol{e}}_3$** — $[0,0,1]^\top$ unit vector. Used in $\boldsymbol{c}_h$, $\dot{\boldsymbol{s}}_e$, $\boldsymbol{h}_\text{d}$, cone-clamp equation. Never defined; mild rigor gap.

**Additional gaps fixed 2026-04-24:**
- **Camera resolution $\mathcal{R}$** now has a numerical value in Table~II: $[320,240]^\top$ px (was referenced but unquantified).
- **Stale numerical value in supplement §S3-B** disturbance recap: "35° attitude-cone projection" → "60° attitude-cone projection" (matches Property 1).
- **Yaw-ASMC symbol mismatch** Table~S1 used `_a` subscript, main paper uses `_α`. Unified to α-subscript; dropped redundant $\Omega_a$ row (was a duplicate of $\chi_\alpha$ per MATLAB).
- **Legacy figure-caption symbols** $\boldsymbol{E}_2 \to \hat{\boldsymbol{r}}_\text{e}$, $\boldsymbol{w}_e \to \boldsymbol{h}_\text{e}$.
- **Boundary-layer claim overstatement**: "0 < ε_k ≪ 1" and "0 < ε_α ≪ 1" → "$\mathcal{E}\succ 0$" and "$\varepsilon_\alpha>0$" (actual locked values are $\varepsilon_k=1.0$ and $\varepsilon_\alpha=3.0$).
- **Figure labels in `make_plasmc_plots.py`**: (u,v) → $\,^\mathcal{C}\hat{x}/\hat{y}$; $g\tan\phi_{\max}$ → $g\tan\theta_\text{cap}$; 35° → 60°; $w_{e,k}$ → $h_{e,k}$.
- **Figure labels in `make_multi_init_plots.py`**: bare $\hat{x}/\hat{y}$ → $\,^\mathcal{C}\hat{x}/\hat{y}$.
- **Worst-case xy error unified** to 5.7 cm across abstract, §IV-B, supplement summary (validated against MATLAB: $Linear$_multi_init.mat IC5 = 5.7126 cm; matches Table IV's 5.71).
- **Frames/planes figure** (`frames_planes.tex`): added image-plane axis labels $\hat{X}_\text{c}/\hat{Y}_\text{c}$ and $\hat{X}_\text{v}/\hat{Y}_\text{v}$; moved $\boldsymbol{p}_1(t)$ envelope from virtual plane to camera plane; added 4-corner cluster; added $\,^\mathcal{V}R_\mathcal{C}$ rotation indicator; corrected caption from "axes stay parallel to $\mathcal{I}$" (overreach) to "$X_v$-$Y_v$ plane stays parallel to the inertial $X_i$-$Y_i$ plane".

**2026-04-25 additions:**
- **Idealised case removed.** Supplement §S3-A deleted entirely; surviving multi-init subsection framed as single-regime "under the disturbance model". All "realistic"/"full-disturbance"/"full robustness" contrast language stripped. §S3-X cross-references bumped up by one letter.
- **IC notation unified** to `IC$_N$` (subscript form, N∈{1..5}) for indexed references, bare `IC` for generic; `IC$N$` (non-subscripted), `\text{IC}_N`, and `IC $[\ldots]$` (coordinates without index) all banned.
- **Table IV restructured.** Caption: "under realistic disturbances (5 ICs per trajectory)" → "under the simulated disturbance model (Table~II); 5 ICs per case". Column header "Trajectory" → "Case". Row labels dropped the repeated "Case" prefix (just "1, 2, 3, 4, 5").

**Image-plane notation dual convention (locked 2026-04-24):**
- Axes / directions: $\hat{X}_\text{c}, \hat{Y}_\text{c}$ (capital with camera-frame subscript).
- Feature-point components: $\,^\mathcal{C}\hat{x}, \,^\mathcal{C}\hat{y}$ (lowercase with camera-frame superscript).
- Feature-point vector: $\,^\mathcal{C}\boldsymbol{\hat{r}} = [\,^\mathcal{C}\hat{x}, \,^\mathcal{C}\hat{y}]^\top$ (supplement §S1-A).
- Virtual-frame counterparts: $\,^\mathcal{V}\hat{x}_k, \,^\mathcal{V}\hat{y}_k$ (line 129 definition for the $k$-th feature point).

**Theorems/Corollary summary (post-2026-04-24):**
- Theorem 1: Adaptive Optic-Flow Funnel Invariance (3D outer-loop ASMC).
- Theorem 2: Adaptive Yaw ASMC Ultimate Boundedness (scalar yaw). Placed after Corollary 1.
- Corollary 1: Closed-loop target visibility (cone clamp + PID cascade).
- Proofs: §S2 "Full Proof of Theorem~1" and "Full Proof of Theorem~2".
- Abstract, contribution (c), conclusion all cite all three.

**How to apply:**
- Before declaring a manuscript-review pass complete, grep every math symbol against its definition site. Use this file as the coverage baseline.
- When making new edits that introduce symbols, also update this file to keep the audit current.
- The "low priority" implicit list above is the reviewer-flag watchlist — rigor pass #2 should tackle these if an IEEE TAES referee demands strict first-use definitions.
