---
name: prose-audit
description: Mechanical writing-style audit across the VDF-ASMC manuscript (manuscript.tex, control_formulation.tex, results.tex, supplemental.tex, block_diagram.tex, figure captions). Enforces the writing rules accumulated in feedback memory — 16 rules covering bare-demonstrative vagueness, controller-name staleness, physical-vs-math visibility phrasing, variable-based funnel naming, legacy-design comparisons, image-feature-point naming / drop "physical" + "target" qualifiers + unify subscript to `_i`, plant-input subscript conventions, trajectory naming, virtual-image-point error symbol naming + "Control Design" subsection-title pattern, funnel/FoV symbol spelling, no overclaim of unproven properties, optic-flow "bounded h" not "constant/zero", §I qualitative / §IV numerical anchoring + locked Para 1-7 wording regression checks, citation classification (lin2023 IBVS landing baseline), Oxford/US spelling consistency, and hardcoded section-ref composition for sub/sub-sub-section refs. Pure grep-based pass; flags defects with file:line, does not rewrite.
---

# /prose-audit

Read-only writing-style audit. Flags every violation of the project's documented prose rules so they can be fixed in a single rewrite pass. Complements `/manuscript-review` (which is content-focused) with a mechanical style sweep.

## Usage

```
/prose-audit                        -- full audit across all manuscript tex
/prose-audit <file>                 -- restrict to one tex file
/prose-audit <rule>                 -- restrict to one rule id (see below)
```

Rule ids: `this`, `name`, `visibility`, `funnel-naming`, `legacy`, `corner`, `subscript`, `trajectory`, `image-feature`, `funnel-symbol`, `overclaim`, `optic-flow-principle`, `numerical-anchoring`, `citation-classification`, `spelling`, `section-ref`.

## Scope

All `.tex` files under `Soft_Precise_Landing/`:
- `manuscript.tex`
- `control_formulation.tex`
- `results.tex`
- `supplemental.tex`
- `block_diagram.tex`
- any figure-caption tex under `Figures/generated/`

Skip `Drafts/` unless explicitly requested — those are frozen backups.

## Rules and detection

### Rule 1 — `this`: no bare demonstrative without an attached noun

Every `this`, `these`, `that`, `those` in technical prose must be followed by an explicit noun. Memory: `feedback_no_vague_this.md`.

**Detection.** Case-insensitive, word-boundaried, in-prose (outside math mode):

```bash
grep -nEi "\b(this|these|those) (is|are|was|were|will|would|should|can|could|may|might|leads|implies|means|gives|shows|provides|follows|allows|yields|makes|ensures|results|motivates|suggests|justifies|explains|requires|indicates|captures|exploits|reflects|guarantees|establishes|enables|holds|avoids|eliminates|prevents|preserves|compensates|solves|addresses|resolves|requires|uses|relies|comes|has|have)\b"
```

Also flag `\\bthis ~\\cite` / `\\bthis \\eqref` / `\\bthis~\\cite` constructions (bare `this` pointing at a citation or equation).

**Allowed form:** `this + <noun>` — e.g., `this funnel`, `this bound`, `this residual`, `this saturation guard`, `this architecture`. When a noun is present, the line is fine.

### Rule 2 — `name`: current controller name is MDF-ASMC; current paper title is T2

Memory: `project_naming_decisions.md`.

**Detection.**

```bash
grep -nE "\bDF-ASMC\b|\bPLASMC\b" <scope>
```

Any hit outside a code-literal MATLAB identifier (`K_PLASMC`, filename strings, .mat-file names) is stale. In tex prose, all controller references should read `MDF-ASMC`.

Also flag the old paper title fragments if they appear in tex:
- `Prescribed Performance` as a title word (T1 legacy)
- `optic[- ]flow adaptive sliding[- ]mode` (T1 legacy)
- `Dual-Funnel` without the leading `Monocular` in the *paper title line* of `manuscript.tex` is OK (dual-funnel is still the architecture name), but the controller acronym must be MDF-ASMC.

### Rule 3 — `visibility`: prose uses physical language, not feature-point math

Memory: `feedback_visibility_phrasing.md`.

**In reader-facing text** (abstract, contribution list, theorem/corollary statements, conclusion, section intros, figure captions): use "target remains visible" / "target stays inside FoV" / "target is always visible".

**Detection of mis-scoped feature-point language:**

```bash
grep -nE "feature point" manuscript.tex            # every hit in manuscript.tex is a candidate
grep -nE "feature point" control_formulation.tex   # only the *proof proper* and set-definition blocks may use this; intro/theorem/corollary statements may not
```

For each hit in `manuscript.tex`, `results.tex`, figure captions, and section intros: the line is a defect unless the surrounding paragraph is a *proof* or an *equation-set definition*. Flag with reason "feature-point math leaked into reader-facing prose".

Also flag `stays inside the pixel-box` / `remain in $\boldsymbol{\rho}_\text{fov}` / `remain in $\boldsymbol{p}_1` in reader-facing prose — same class of defect.

### Rule 4 — `funnel-naming`: use variable-based names (target image funnel + optic-flow funnel)

Memory: `feedback_funnel_naming.md` + `project_dual_funnel_parallel_architecture.md`.

**Correct (locked 2026-04-22):** `target image funnel` (for $\boldsymbol{p}_1(t)$, pixels), `optic-flow funnel` (for $\boldsymbol{p}_2(t)$, rad/s).

**Forbidden (all superseded):**
- `visibility funnel` / `Visibility Funnel` — old name for $\boldsymbol{p}_1(t)$; visibility is a closed-loop effect, not the bound itself.
- `soft-landing funnel` / `Soft-Landing Funnel` — rejected purpose-based name for $\boldsymbol{p}_2(t)$.
- `optic-flow performance funnel` — redundant (a funnel is already PPC).
- `inner funnel`, `outer funnel`, `the inner/outer funnels` (aliases with inner/outer *loop*).

Also forbidden phrasings that imply series-through-PID or direct funnel coupling:
- `the two funnels meet at`
- `funnels coupled through`
- `funnels couple via`
- `target image funnel synthesises a feasible optic-flow command`
- `visibility funnel synthesises a feasible optic-flow command` (legacy)
- `[target image|optic-flow] funnel drives the other`

**Detection.**

```bash
grep -nEi "\b[Vv]isibility [Ff]unnel\b"
grep -nEi "\b[Ss]oft-?landing [Ff]unnel\b"
grep -nEi "\boptic-flow performance funnel\b"
grep -nEi "\b(inner|outer) funnel\b"
grep -nEi "two funnels meet|funnels coupled|funnels couple|funnel synthesises a feasible|funnel drives the other"
```

**Note on subsection heading:** `\section`/`\subsection` lines containing `Visibility Funnel: FoV-Adaptive Cone Clamp` must be retitled to `Target Image Funnel and FoV-Adaptive Cone Clamp` — the funnel is the bound, the cone clamp is the enforcement mechanism; keep them architecturally distinct.

### Rule 5 — `legacy`: no comparisons to superseded internal design variants

Memory: `feedback_no_legacy_comparisons.md`.

Tex should read as-if-designed, never as-if-migrated from an earlier draft.

**Detection.**

```bash
grep -nEi "earlier design|previous design|legacy approach|Approach 1 vs|replacing .* of earlier|in contrast to earlier drafts|unlike the earlier formulation"
```

Also flag `Approach 1` / `Approach 2` literal text in any tex file — these were internal working names and should not appear in the finished manuscript.

### Rule 6 — `corner`: tracked features are "image feature points"; drop "physical" and "target" qualifiers

Memory: `feedback_corner_points_naming.md` + `feedback_no_landmark_term.md` (vocabulary epoch-2 locked 2026-05-04).

**Canonical naming ontology (epoch-2, 2026-05-04):**

| Quantity | Long form | Short form | Symbol |
|---|---|---|---|
| 3-D point on the target | target point | — | $\,^\mathcal{V}\boldsymbol{r}_\text{t}$ |
| 2-D individual point in camera image plane, $i\in\{1,\dots,N\}$ | **image feature point** | — | $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ |
| 2-D individual point in virtual image plane, $i\in\{1,\dots,N\}$ | **virtual image feature point** | virtual feature point | $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$ |
| 2-D mean of camera-plane feature points | **image point** | — | $\,^\mathcal{C}\boldsymbol{\hat{r}}$ |
| 2-D mean of virtual-plane feature points | **virtual image point** | — | $\,^\mathcal{V}\boldsymbol{\hat{r}}$ |

**Subscript role lock (epoch-2, 2026-05-04):**
- Per-feature index = dummy summation index ($1..N$): `i` (e.g., $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$, $\sum_{i=1}^{N}$).
- 3-D-vector component index ($1..3$): `k` (e.g., $h_{e_k}$, $k\in\{1,2,3\}$).
- Moment exponents: `p, q` (e.g., $\mu_{pq}$, $(\cdot)^p(\cdot)^q$).

**Frame qualifiers:** "physical" is dropped globally. The unqualified `image plane` / `camera frame $\mathcal{C}$` IS the physical (real) camera; only `virtual` marks the derived variant. The "target" qualifier is also dropped (epoch-2 2026-05-04) since the camera observes only the target; "image feature point" is unambiguous.

**Forbidden phrasings:**
- `corner point` / `corner points` / `target image corner point(s)` — superseded.
- `physical corner(s)` / `physical image corner(s)` / `physical (image )?feature point(s)` — superseded.
- `physical camera frame` / `physical image plane` — drop "physical".
- **(epoch-2)** `target image feature point(s)` — drop "target" qualifier.
- **(epoch-2)** `target image point` (for the centroid) — use "image point" / "virtual image point".
- **(epoch-2)** `the $k$-th feature point` (when referring to the per-feature virtual-frame projection) — use "the $i$-th virtual image feature point".
- **(epoch-2)** `\,^\mathcal{V}\boldsymbol{\hat{r}}_k` — use `\,^\mathcal{V}\boldsymbol{\hat{r}}_i` (subscript unified to per-feature `_i`).
- **(epoch-2)** `\mu_{ij}` (with `^i`/`^j` exponents in the moment definition) — use `\mu_{pq}` with `^p`/`^q` exponents.

**Detection.**

```bash
grep -nEi "\\bcorner point(s)?\\b"
grep -nEi "\\btarget image corner point(s)?\\b"
grep -nEi "\\bphysical corner(s)?\\b"
grep -nEi "\\bphysical image corner(s)?\\b"
grep -nEi "\\bphysical (image )?feature point(s)?\\b"
grep -nEi "\\bphysical camera frame\\b"
grep -nEi "\\bphysical image plane\\b"

# Epoch-2 forbidden — drop "target" qualifier:
grep -nE "\\btarget image feature point(s)?\\b"
grep -nE "\\btarget image point\\b"

# Epoch-2 subscript regression:
grep -nE "\\\\,\\^\\\\mathcal\\{V\\}\\\\boldsymbol\\{\\\\hat\\{r\\}\\}_k"   # virtual frame per-feature must be `_i`
grep -nE "\\\\,\\^\\\\mathcal\\{V\\}\\\\hat\\{x\\}_k"
grep -nE "\\\\,\\^\\\\mathcal\\{V\\}\\\\hat\\{y\\}_k"
grep -nE "the \\$k\\$-th feature point"   # epoch-1 prose form

# Epoch-2 moment-exponent regression (must be `pq` not `ij`):
grep -nE "\\\\mu_\\{ij\\}"
```

Any match is a defect. Preferred terms: `image feature point(s)` (camera plane), `virtual image feature point(s)` (virtual plane); `image point` / `virtual image point` for the centroid; `target point` for the real-world 3-D landmark.

### Rule 7 — `subscript`: plant inputs use `_u`, body frame is mandatory

Memory: `feedback_notation_subscript_convention.md`.

Torque `τ` and thrust `T` are plant control inputs — must be `_u`, never `_d`. Body-frame prefix `\,^\mathcal{B}` is mandatory on both.

**Detection.**

```bash
grep -nE "\\\\tau_\\\\text\{d\}|T_\\\\text\{d\}|\\\\boldsymbol\{\\\\tau\}_\\\\text\{d\}"
grep -nE "(?<!\\^\\\\mathcal\\{B\\})\\\\tau_u|(?<!\\^\\\\mathcal\\{B\\})T_u"
```

Second pattern flags `\tau_u` or `T_u` missing the body-frame prefix.

### Rule 8 — `trajectory`: use Case 1–5 numbering; specific phrasing rules

Memory: `feedback_trajectory_naming.md`.

**Detection.**

```bash
grep -nEi "linear target(?! trajectory)"   # "linear target" without "trajectory" suffix
grep -nEi "\bRun [1-9]|\bRun~[1-9]"        # "Run" label instead of "IC"
```

The phrase "static target" is OK; "linear target" alone is wrong and must be "linear target trajectory". "Run" must be "IC" when referring to initial conditions in the multi-init sweep.

### Rule 9 — `image-feature`: virtual image point error + PID naming (epoch-3, locked 2026-05-04)

Memory: `feedback_image_feature_naming.md`.

Canonical names (epoch-3, locked 2026-05-04). The variables describe error in the **virtual image point** $\,^\mathcal{V}\hat{\boldsymbol{r}}$ (centroid in $\mathcal{V}$), not per-feature errors:

- `\hat{\boldsymbol{r}}_\text{e}` → "virtual image point error" (long, at definition); "image point error" (short, in body).
- `\bar{\boldsymbol{r}}_\text{e}` → "normalized virtual image point error" (long, at definition); "normalized image point error" (short, in body).
- `\hat{\boldsymbol{r}}_\text{d}` → "desired image point" (long form only — no short form).
- Outer-loop PID short tag → **"Virtual Image Point PID"** (used in body, tables, block-diagram labels, Corollary 1 cite).

Subsection / control-law titles (parallel "Control Design" pattern):
- §III.A.1: `\subsubsection{Virtual Image Point Control Design}`
- §III.A.2: `\subsubsection{Optic Flow Control Design}`
- §III.B.1: `\subsubsection{Yaw Control Design}`

**Detection — epoch-1/-2 stale variants that should be zero:**

```bash
# Variable names (epoch-2 forms — superseded 2026-05-04):
grep -nE "virtual image feature error\\b"              # epoch-2 long form
grep -nE "normalized virtual image feature error\\b"   # epoch-2 long form
grep -nE "\\bvirtual feature error\\b"                 # epoch-2 short form (drop "image")
grep -nE "\\bnormalized virtual feature error\\b"      # epoch-2 short form

# PID tag (epoch-2 forms — superseded):
grep -nE "[Nn]ormalized [Vv]irtual [Ff]eature PID"     # epoch-2 PID tag
grep -nE "[Nn]ormalized [Vv]irtual [Ii]mage [Ff]eature PID"  # epoch-2 abstract-form PID tag

# Subsection titles (must use epoch-3 "Control Design" pattern):
grep -nE "subsubsection\\{Normalized Virtual Image Feature Control Law\\}"  # epoch-2 §III.A.1
grep -nE "subsubsection\\{Optic Flow Control Law\\}"                        # epoch-2 §III.A.2
grep -nE "subsubsection\\{Yaw ASMC with Virtual-Compass Integrator\\}"      # epoch-2 §III.B.1

# Even older epoch-1 forms (should also be zero):
grep -nE "[Nn]ormalized-[Ee]rror PID"           # epoch-1 tag
grep -nE "image-feature error +PID"             # epoch-1 abstract tag
grep -nE "virtual image-feature error"          # hyphen variant
grep -nE "feature-error PID"                    # hyphenated short form
grep -nE "normalized image feature error\\b"    # missing "virtual"
grep -nE "outer image[- ]feature error"         # supplemental §S3 intro variant
```

**Detection — locked epoch-3 forms (must hit):**

```bash
grep -nE "subsubsection\\{Virtual Image Point Control Design\\}" Soft_Precise_Landing/control_formulation.tex
grep -nE "subsubsection\\{Optic Flow Control Design\\}" Soft_Precise_Landing/control_formulation.tex
grep -nE "subsubsection\\{Yaw Control Design\\}" Soft_Precise_Landing/control_formulation.tex
grep -nE "Virtual Image Point PID" Soft_Precise_Landing/control_formulation.tex
grep -nE "virtual image point error" Soft_Precise_Landing/control_formulation.tex
```

Allowed occurrences:
- `\label{normalized error: equation}` — label key for the normalization equation; do not rename (would cascade through `\eqref`).
- Definition paragraph at `control_formulation.tex` §III.A.1 — one full-form definition of each symbol (`\emph{virtual image point error}`, `\emph{normalized virtual image point error}`).
- Body / table / block-diagram tag is "Virtual Image Point PID" (no qualifier prefix).

### Rule 10 — `funnel-symbol`: target image funnel uses $\boldsymbol{p}_1$ / $\xi_1$ / $\boldsymbol{\varphi}_\text{max}$

Memory: `feedback_funnel_symbol_convention.md`.

Canonical symbols (locked 2026-04-22):
- Target image funnel time-varying bound: `\boldsymbol{p}_1(t)`.
- Target image funnel initial/terminal values: `\boldsymbol{p}_{1_0}`, `\boldsymbol{p}_{1_\infty}` (pixel units).
- Target image funnel decay rate: `\xi_1`.
- Target image funnel scalar-$k$ component: `p_{1_k}`.
- Camera half-FoV normalization constant: `\boldsymbol{\varphi}_\text{max} = \mathcal{R}/(2f)` (feature/tangent units).
- Matrix form in proofs: `\Phi_\text{max} = \text{diag}(\boldsymbol{\varphi}_\text{max})`.

**Detection** — forbidden old spellings:

```bash
grep -nE "\\\\rho_\\\\text\\{fov\\}|\\\\rho_\\{\\\\text\\{fov"       # old target-image-funnel vector
grep -nE "\\\\boldsymbol\\{\\\\rho\\}_\\\\text\\{fov\\}"             # old boldsymbol form
grep -nE "\\\\boldsymbol\\{\\\\rho\\}_\\{\\\\text\\{fov"             # old boldsymbol initial/inf form
grep -nE "\\\\ell_\\\\text\\{fov\\}"                                 # old decay rate
grep -nE "rho_\\{\\\\text\\{fov,0|rho_\\{\\\\text\\{fov,\\\\infty"   # initial/terminal fragments
grep -nE "P_\\{1_0\\}"                                               # old matrix form in supplement (should be Phi_max)
```

**Subtle trap:** `\boldsymbol{p}_{1_0}` is now the target-image-funnel initial pixel-box value. Inside the PID / normalization definition it MUST be `\boldsymbol{\varphi}_\text{max}`, not `\boldsymbol{p}_{1_0}`. Detection of the collision:

```bash
grep -nE "\\\\oslash\\s*\\\\boldsymbol\\{p\\}_\\{1_0\\}"             # normalization using wrong symbol
grep -nE "sensor-half|sensor half.?width.*\\\\boldsymbol\\{p\\}_\\{1_0\\}"
```

Any hit = defect: rewrite to use `\boldsymbol{\varphi}_\text{max}`.

### Rule 11 — `overclaim`: don't claim properties no theorem in §III actually proves

Memory: `feedback_no_overclaim_proven_properties.md`.

Theorems we have: Theorem 1 (`h_e` funnel invariance), Theorem 2 (yaw UUB), Corollary 1 (kinematic visibility). Soft-touchdown (`‖v_rel(t_touch)‖ ≤ 0.20 m/s`) is *empirical* (25/25 in §IV), not a closed-form bound.

**Detection — forbidden phrasings (anywhere in main paper or abstract):**

```bash
grep -niE "transient (specification|specifications|bound) (required|for|of)" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -niE "finite[- ]time (bound|convergence|stability|guarantee).*(v_?rel|relative velocity)" Soft_Precise_Landing/*.tex
grep -niE "guarantee.*soft touchdown|prove.*soft touchdown" Soft_Precise_Landing/*.tex
grep -niE "drives the relative velocity to zero in finite time" Soft_Precise_Landing/*.tex
```

Any hit is a defect: replace with the empirically-validated phrasing ("demonstrates a 25/25 soft-precise landing rate", "delivers ... in simulation", etc.) or with a guarantee that *is* proven (UUB, funnel invariance, visibility).

### Rule 12 — `optic-flow-principle`: bounded h, not "constant" or "zero"

Memory: `reference_optic_flow_soft_landing_principle.md`.

The §I optic-flow soft-landing argument must use **bounded** wording, not "constant" (1-D Hérissé only) or "zero" (would prevent descent).

**Detection — forbidden phrasings in §I:**

```bash
grep -niE "(force|forcing|drive|driving)\s+(it|optic flow|the optic flow)\s+to (zero|0)" Soft_Precise_Landing/manuscript.tex
grep -niE "regulat(e|ing)\s+(it|optic flow|the optic flow)\s+to a constant" Soft_Precise_Landing/manuscript.tex
grep -niE "descent velocity" Soft_Precise_Landing/manuscript.tex   # should be "relative velocity" (3-D)
```

Any hit in §I introduction is a defect (1-D-vertical framing leaking into the 3-D narrative). Replace with "keeping it bounded" / "bounded reference" / "bounding" + "UAV–target relative velocity".

### Rule 13 — `numerical-anchoring`: §I qualitative, §IV numerical

Memory: `feedback_functional_requirement_framing.md`, `feedback_soft_precise_definition_location.md`.

§I Para 1 must carry the **qualitative two-clause functional requirement**, not numerical thresholds. Numerical thresholds (`0.08 m`, `0.20 m/s`, `z_f = 0.20 m`) live in `results.tex:4` and Table II only.

**Detection — numbers leaking into §I (manuscript.tex line range ~98–120):**

```bash
sed -n '95,120p' Soft_Precise_Landing/manuscript.tex | grep -nE "0\.08|0\.20|z_\\\\text\\{f\\}|xy_\\\\text\\{prec\\}|v_\\\\text\\{soft\\}"
```

Any hit is a defect (numerical threshold has migrated back into §I). Move to §IV.A or remove.

**Detection — functional requirement missing from §I Para 1 (locked 2026-05-02 wording):**

```bash
grep -niE "lateral approach of the UAV to a close vicinity of the target.*before touchdown" Soft_Precise_Landing/manuscript.tex
grep -niE "simultaneous convergence of the UAV's altitude and velocity" Soft_Precise_Landing/manuscript.tex
grep -niE "the functional requirements are \(a\)" Soft_Precise_Landing/manuscript.tex
```

If any fails to hit, §I Para 1 has regressed away from the locked functional-requirement statement.

**Detection — Para 1 motivation chain regression (S2+S3 collapsed):**

```bash
# S2 must name two distinct failure modes with concrete consequences:
grep -niE "imprecise landing risks missing the platform.*grazing its edge" Soft_Precise_Landing/manuscript.tex
grep -niE "hard landing risks airframe and payload damage on contact" Soft_Precise_Landing/manuscript.tex

# S3 lead "Hence, the terminal landing phase demands both ... precise positioning and a ... soft touchdown":
grep -niE "Hence, the terminal landing phase demands both" Soft_Precise_Landing/manuscript.tex

# Old collapsed form should NOT appear (regression check):
grep -niE "is particularly demanding because it couples tight positioning" Soft_Precise_Landing/manuscript.tex
grep -niE "imprecise or hard landings risk damage" Soft_Precise_Landing/manuscript.tex
```

The first two should hit; the last two should NOT hit (they are the superseded lumped wording).

**Detection — Para 2 closing gap statement (locked 2026-05-02):**

```bash
# Para 2 closing must contain the negative attribution form:
grep -niE "None of these frameworks achieve the functional requirement of the soft touchdown" Soft_Precise_Landing/manuscript.tex

# Para 2 must contain the positive IBVS achievement claim earlier:
grep -niE "These IBVS frameworks achieve the functional requirement of the precise touchdown through asymptotic image-feature convergence" Soft_Precise_Landing/manuscript.tex

# Para 2 S1 must scope to autonomous landing (locked 2026-05-02):
grep -niE "Existing solutions for autonomous landing fall into" Soft_Precise_Landing/manuscript.tex

# Para 2 S2 PBVS sensor framing must use "visual measurements" (locked 2026-05-02):
grep -niE "PBVS approaches rely on explicit 3-D pose reconstruction from visual measurements" Soft_Precise_Landing/manuscript.tex

# Old Para 2 critique form should NOT appear (regression check — was scoped to IBVS only and used the indirect-velocity mechanism):
grep -niE "Though IBVS frameworks achieve the functional requirement of the precise touchdown" Soft_Precise_Landing/manuscript.tex
grep -niE "asymptotic convergence of the normalized target position" Soft_Precise_Landing/manuscript.tex
grep -niE "explicit depth or implicit scale information" Soft_Precise_Landing/manuscript.tex   # missing "metric"

# Old Para 2 sensor-list should NOT appear (regression check — was definitionally wrong):
grep -niE "PBVS approaches rely on explicit 3-D pose reconstruction from LiDAR" Soft_Precise_Landing/manuscript.tex
grep -niE "pose reconstruction from LiDAR, stereo cameras, or GPS" Soft_Precise_Landing/manuscript.tex
grep -niE "salehi2021.*lin2022.*zhang2026" Soft_Precise_Landing/manuscript.tex   # salehi2021 must NOT be in PBVS list

# Anti-pattern: "None of these frameworks ATTEMPT" is too strong — lin2022 attempts velocity-PPC. Should be "achieve":
grep -niE "None of these frameworks attempt to achieve" Soft_Precise_Landing/manuscript.tex
```

**Detection — Para 3 optic-flow compressed form (locked 2026-05-02):**

Para 3 is compressed to 2 sentences for page-limit:

```bash
# S1 must position optic flow as natural sensing modality for soft touchdown WITH the three-cite list:
grep -niE "Optic flow.* inherently regulates both relative altitude and relative velocity to the target, making it a natural sensing modality for the soft touchdown \\\\cite\\{herisse2012, izzo2011, singhal2025\\}" Soft_Precise_Landing/manuscript.tex

# S2 must use the per-paper split with singhal2025 in the vertical-stationary group:
grep -niE "either restrict to purely vertical descent on stationary targets \\\\cite\\{izzo2011, singhal2025\\}.*or rely on an external heading reference and assume an unrestricted field of view during the descent \\\\cite\\{herisse2012\\}" Soft_Precise_Landing/manuscript.tex

# REGRESSION checks — old wordings that should NOT appear:
grep -niE "Existing optic-flow schemes, however, typically address purely vertical descent on stationary targets" Soft_Precise_Landing/manuscript.tex   # lumped form
grep -niE "rely on IMU-aided attitude" Soft_Precise_Landing/manuscript.tex                                                                              # self-inconsistent (we use IMU too)
grep -niE "drives the relative velocity to zero as altitude vanishes" Soft_Precise_Landing/manuscript.tex                                              # violates "no zero" rule (reference_optic_flow_soft_landing_principle.md)
grep -niE "Regulating .emph\\{optic flow\\} rather than metric position circumvents scale ambiguity entirely" Soft_Precise_Landing/manuscript.tex      # superseded — scale-ambiguity tail dropped from Para 3
grep -niE "Our own prior work \\\\cite\\{singhal2025\\} was restricted to a single vertical output on a stationary surface" Soft_Precise_Landing/manuscript.tex   # superseded — singhal2025 merged into S2 vertical-stationary cite group
grep -niE "three-dimensional image-kinematic coupling" Soft_Precise_Landing/manuscript.tex                                                              # technical jargon dropped
grep -niE "approach distance and relative velocity" Soft_Precise_Landing/manuscript.tex                                                                 # was singhal2025 phrasing; replaced with "relative altitude" for Para 1 parallelism
```

The first two should hit; all the regression checks should NOT hit.

**Detection — Para 4 visibility-constraint DIRECT mechanisms (locked 2026-05-03):**

Para 4 covers two direct visibility-constraint mechanisms (barrier on image features, input saturation on attitude). lin2022's PPC was moved to §III where the PPC paradigm is foundational for the optic-flow funnel.

```bash
# Para 4 S1 must use "directly enforces" scope marker:
grep -niE "An orthogonal line of work directly enforces .emph\\{visibility constraints\\}" Soft_Precise_Landing/manuscript.tex

# Para 4 must contain the two direct-mechanism sentences:
grep -niE "Barrier-function constructions \\\\cite\\{salehi2021\\} impose hard inequality constraints but render the control law unbounded near the FoV boundary" Soft_Precise_Landing/manuscript.tex
grep -niE "Input-saturated visual servoing \\\\cite\\{xie2016\\} bounds the vehicle's roll and pitch to keep features inside the FoV, but the fixed bound cannot tighten as features approach the FoV edge" Soft_Precise_Landing/manuscript.tex

# Para 4 must end with the plural closing covering both mechanisms:
grep -niE "These methods answer the visibility question but not the soft-touchdown question\\." Soft_Precise_Landing/manuscript.tex

# §III must cite lin2022 as PPC paradigm foundation:
grep -niE "We follow the prescribed-performance control \\(PPC\\) paradigm \\\\cite\\{lin2022\\}" Soft_Precise_Landing/control_formulation.tex

# §III must cite xie2016 in the cone-clamp introduction:
grep -niE "generalising the fixed roll/pitch saturation of \\\\cite\\{xie2016\\} to a state-dependent constraint" Soft_Precise_Landing/control_formulation.tex

# REGRESSION checks — Para 4 should NOT contain PPC, adaptive/learning content, or older wordings:
grep -niE "Prescribed-performance control \\(PPC\\) \\\\cite\\{lin2022\\}.*bounds the tracking error" Soft_Precise_Landing/manuscript.tex                # lin2022 PPC moved out of Para 4
grep -niE "but the bound is fixed rather than state-dependent" Soft_Precise_Landing/manuscript.tex                                                       # "state-dependent" framing in §I (reserved for §III)
grep -niE "Recent state-estimation pipelines \\\\cite\\{bouazza2025\\}.*while adaptive" Soft_Precise_Landing/manuscript.tex                              # adaptive/learning in Para 4 (now in Para 5)
grep -niE "with no mechanism to prescribe the rate at which the UAV--target relative velocity decays" Soft_Precise_Landing/manuscript.tex                # PPC critique sentence removed entirely
grep -niE "the rate at which the UAV.target relative velocity vanishes" Soft_Precise_Landing/manuscript.tex                                              # "vanishes" overclaim
grep -niE "PPC.*\\\\cite\\{lin2022\\} instead forces the image error" Soft_Precise_Landing/manuscript.tex                                                # "image error" inaccurate (lin2022 is PBVS-PPC)
grep -niE "^[^%]*It answers the visibility question but not the soft-touchdown question\\." Soft_Precise_Landing/manuscript.tex                          # singular "It" pre-xie2016-addition
```

The first five should hit; all regression checks should NOT hit.

**Detection — Para 6 research-gap-level reframing (locked 2026-05-03):**

Para 6 carries problem-statement-level research gaps (not solution-relative critiques). Each gap is named in terms of an unsolved problem, not in terms of what existing methods don't do.

```bash
# S1 must use the "to the best of the authors' knowledge" hedge:
grep -niE "To the best of the authors' knowledge, the literature exhibits three research gaps\\." Soft_Precise_Landing/manuscript.tex

# Gap (i): soft-precise on moving target using only monocular vision:
grep -niE "The soft-precise touchdown on a moving target using only monocular vision remains an open problem" Soft_Precise_Landing/manuscript.tex

# Gap (ii): online uncertainty identification on the optic-flow dynamics (not "image-kinematic plant"):
grep -niE "Online identification of the full uncertainty on the optic-flow dynamics" Soft_Precise_Landing/manuscript.tex

# Gap (iii): closed-loop stability guarantee for soft + precise + visibility:
grep -niE "A closed-loop stability guarantee that simultaneously certifies the soft touchdown, the precise touchdown, and target visibility is missing" Soft_Precise_Landing/manuscript.tex

# Closing must restore the Table~\ref{tab:comparison} cross-reference:
grep -niE "These gaps, summarised qualitatively in Table~\\\\ref\\{tab:comparison\\}, motivate the MDF-ASMC framework" Soft_Precise_Landing/manuscript.tex

# REGRESSION checks — solution-implicit gap framings should NOT appear:
grep -niE "Extending the above critique to the broader literature, no existing image-based landing controller jointly constrains" Soft_Precise_Landing/manuscript.tex   # old gap (i)
grep -niE "no existing image-based landing controller jointly constrains the image-plane and optic-flow errors" Soft_Precise_Landing/manuscript.tex                  # old gap (i) shortened
grep -niE "image-kinematic plant" Soft_Precise_Landing/manuscript.tex                                                                                                 # superseded by "optic-flow dynamics"
grep -niE "no existing adaptive visual-servoing law identifies the full sliding-mode gain online" Soft_Precise_Landing/manuscript.tex                                # old gap (ii)
grep -niE "No prior work has established a layered stability certificate combining a Lyapunov argument" Soft_Precise_Landing/manuscript.tex                         # old gap (iii) (solution-implicit)
```

The first five should hit; all regression checks should NOT hit.

**Detection — §II.B section-title parallelism (locked 2026-05-04, epoch-3):**

§II.B subsection title is "Image Parameters" (no "target" qualifier; locked 2026-05-04). §II.B.1 / §II.B.2 / §II.B.3 use the parallel "Virtual Image [Property]" / "Optic Flow" pattern.

```bash
# §II.B subsection title and subsubsection titles must use the locked 2026-05-04 forms:
grep -nE "subsection\\{Image Parameters\\}" Soft_Precise_Landing/control_formulation.tex
grep -nE "subsubsection\\{Virtual Image Position\\}" Soft_Precise_Landing/control_formulation.tex
grep -nE "subsubsection\\{Virtual Image Orientation\\}" Soft_Precise_Landing/control_formulation.tex
grep -nE "subsubsection\\{Optic Flow\\}" Soft_Precise_Landing/control_formulation.tex

# REGRESSION checks — superseded forms should NOT appear:
grep -nE "subsection\\{Target Image Parameters\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "subsubsection\\{Normalised Target Position\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "subsubsection\\{Normalized Target Position\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "subsubsection\\{Target Virtual Orientation\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "subsubsection\\{Virtual Target Orientation\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -niE "the virtual target orientation" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -niE "the normali[sz]ed target position" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -niE "the target virtual orientation" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -niE "target image parameters?\\b" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex   # subsection rename — drop "target"
```

The first four should hit; all regression checks should NOT hit (except the LaTeX label `\label{...target image parameters...}` if any internal label name was deliberately preserved — verify case-by-case).

**Detection — Para 5 final 5-sentence form with parallel inline critiques (locked 2026-05-03):**

Para 5 has clean S1↔S2/S3a/S3b 1:1:1 category mapping; every cite has an inline critique; S4 sweeping closing is a clean recap.

```bash
# Lead-in S1 must use "Beyond the above frameworks" bridge with three categories:
grep -niE "Beyond the above frameworks, recent work introduces vision--IMU sensor fusion, adaptive control, and learning-based control" Soft_Precise_Landing/manuscript.tex

# S2 must include inline critique for bouazza2025:
grep -niE "State-estimation pipelines \\\\cite\\{bouazza2025\\} reconstruct relative pose through vision--IMU fusion but require an inter-platform communication channel and known target geometry" Soft_Precise_Landing/manuscript.tex

# S3a must split adaptive control:
grep -niE "Adaptive control schemes \\\\cite\\{paris2020, lin2023, zhang2026\\} presuppose an .emph\\{a priori\\} characterization of the disturbance class or switching gain" Soft_Precise_Landing/manuscript.tex

# S3b must split learning-based control:
grep -niE "Learning-based control schemes \\\\cite\\{kamath2026\\} depend on heavy perception pipelines and pre-identify an aerodynamic model offline" Soft_Precise_Landing/manuscript.tex

# S4 sweeping closing must remain:
grep -niE "None of these is available at flight time on an unknown target with unknown depth scaling" Soft_Precise_Landing/manuscript.tex

# REGRESSION checks — old wordings that should NOT appear:
grep -niE "Recent extensions introduce vision" Soft_Precise_Landing/manuscript.tex                                                                        # superseded "Recent extensions" lead-in
grep -niE "Other relevant work (covers|in autonomous landing addresses)" Soft_Precise_Landing/manuscript.tex                                              # superseded misleading lead-in
grep -niE "Adaptive and learning-based control schemes \\\\cite\\{paris2020, lin2023, kamath2026, zhang2026\\}" Soft_Precise_Landing/manuscript.tex      # superseded merged form
grep -niE "Recent state-estimation pipelines \\\\cite\\{bouazza2025\\} reconstruct relative pose through vision--IMU fusion[.]\\s+Adaptive" Soft_Precise_Landing/manuscript.tex   # bouazza2025 sentence missing inline critique
```

The first five should hit; all regression checks should NOT hit.

**Detection — Para 5 (NEW paragraph for vision-IMU + adaptive/learning extensions; locked 2026-05-02):**

```bash
# Lead-in must use the locked phrasing:
grep -niE "Recent extensions introduce vision--IMU sensor fusion, adaptive observers, and learning-based control" Soft_Precise_Landing/manuscript.tex

# State-estimation sentence must cite bouazza2025 alone:
grep -niE "State-estimation pipelines \\\\cite\\{bouazza2025\\} reconstruct relative pose through vision--IMU fusion" Soft_Precise_Landing/manuscript.tex

# Adaptive/learning critique must use the broadened phrasing:
grep -niE "Adaptive and learning-based control schemes \\\\cite\\{paris2020, lin2023, kamath2026, zhang2026\\} either depend on heavy perception pipelines, presuppose an .emph\\{a priori\\} characterization of the disturbance class or switching gain, or pre-identify an aerodynamic model offline" Soft_Precise_Landing/manuscript.tex

# Para 5 closing must contain the gap statement:
grep -niE "None of these is available at flight time on an unknown target with unknown depth scaling" Soft_Precise_Landing/manuscript.tex

# REGRESSION — old "Other relevant work covers" lead-in (was misleading):
grep -niE "Other relevant work (covers|in autonomous landing addresses) relative-state estimation and adaptive control" Soft_Precise_Landing/manuscript.tex
```

The first four should hit; the last should NOT hit.

**Detection — Para 5 gap (ii) phrasing must be synced with Para 4:**

```bash
# Para 5 gap (ii) must use the broadened "characterization of the disturbance class or switching gain":
grep -niE "require an .emph\\{a priori\\} characterization of the disturbance class or switching gain" Soft_Precise_Landing/manuscript.tex

# REGRESSION check — old SMC-only "bound on switching gain" wording:
grep -niE "require an .emph\\{a priori\\} bound on the switching gain or disturbance" Soft_Precise_Landing/manuscript.tex
```

The first should hit; the second should NOT hit.

**Detection — Para 7 contribution-summary order (a, b, c, d):**

```bash
# Para 7 must enumerate contributions in (a, b, c, d) order to match the contributions list:
grep -niE "Contribution.\\(a\\) closes gaps.\\(i\\).-.\\(ii\\) by fusing.*\\(b\\) removes.*\\(c\\) closes gap.\\(iii\\).*\\(d\\) characterises" Soft_Precise_Landing/manuscript.tex

# REGRESSION check — old (a, c, b, d) order:
grep -niE "Contribution.\\(a\\).*\\(c\\) closes gap.\\(iii\\).*\\(b\\) removes" Soft_Precise_Landing/manuscript.tex
```

The first should hit; the second should NOT hit.

**Detection — Para 4 critique split + tightened scope (locked 2026-05-02):**

```bash
# Para 4 must split state-estimation (bouazza2025) from adaptive control schemes:
grep -niE "Recent state-estimation pipelines \\\\cite\\{bouazza2025\\} reconstruct relative pose through vision" Soft_Precise_Landing/manuscript.tex
grep -niE "adaptive and learning-based control schemes \\\\cite\\{paris2020, lin2023, kamath2026, zhang2026\\}" Soft_Precise_Landing/manuscript.tex

# Para 4 critique must use the broadened "characterization of the disturbance class or switching gain":
grep -niE "presuppose an .emph\\{a priori\\} characterization of the disturbance class or switching gain" Soft_Precise_Landing/manuscript.tex

# Old Para 4 critique form should NOT appear (regression check):
grep -niE "Recent adaptive and learning-based schemes \\\\cite\\{paris2020, lin2023, bouazza2025" Soft_Precise_Landing/manuscript.tex   # bouazza2025 lumped with controllers
grep -niE "presuppose an .emph\\{a priori\\} upper bound on the switching gain or disturbance" Soft_Precise_Landing/manuscript.tex   # SMC-only phrasing
```

The first three should hit; the last two should NOT hit.

**Detection — clause (a) using "relative position" (3-D) instead of "lateral":**

```bash
grep -niE "relative position to a (small|close) vicinity of (the origin|the target)" Soft_Precise_Landing/manuscript.tex
```

Any hit is a defect: clause (a) is **lateral only** because the precise criterion is `‖^I e_xy‖`, not 3-D. The vertical position component lives in clause (b) ("altitude and velocity converge to those of the target at touchdown").

**Detection — clause (b) missing the altitude–velocity simultaneity:**

```bash
grep -niE "(b)\s+the.*convergence of (the UAV's )?velocity to (that|those) of the target" Soft_Precise_Landing/manuscript.tex
```

If hit AND no "altitude" mention in the same enumeration: defect. Clause (b) must couple altitude and velocity (the simultaneity rules out incomplete-landing and crash failure modes the previous wording could not exclude).

### Rule 14 — `citation-classification`: every PBVS / IBVS / optic-flow citation must match the paper's actual framework

Memory: `feedback_citation_classification_audit.md`.

Every citation in a sentence that asserts a framework label (PBVS, IBVS, optic-flow regulation, learning-based, adaptive-observer) must match the paper's actual framework. Caught real defect 2026-05-02: `\cite{cho2022}` was placed under "PBVS approaches rely on explicit 3-D pose reconstruction" when cho2022 is feed-forward IBVS.

**Detection — known mis-placements:**

```bash
# cho2022 should NEVER appear in a PBVS-asserting sentence:
grep -nE "PBVS.*\\\\cite\\{[^}]*cho2022" Soft_Precise_Landing/manuscript.tex
grep -nE "pose reconstruction.*\\\\cite\\{[^}]*cho2022" Soft_Precise_Landing/manuscript.tex
grep -nE "position-based.*\\\\cite\\{[^}]*cho2022" Soft_Precise_Landing/manuscript.tex

# lin2023 (IBVS) should NEVER appear in a PBVS-asserting sentence:
grep -nE "PBVS.*\\\\cite\\{[^}]*lin2023" Soft_Precise_Landing/manuscript.tex
grep -nE "pose reconstruction.*\\\\cite\\{[^}]*lin2023" Soft_Precise_Landing/manuscript.tex

# salehi2021 should NEVER appear in a PBVS-asserting sentence (it is IBVS — verified 2026-05-02):
grep -nE "PBVS.*\\\\cite\\{[^}]*salehi2021" Soft_Precise_Landing/manuscript.tex
grep -nE "pose reconstruction.*\\\\cite\\{[^}]*salehi2021" Soft_Precise_Landing/manuscript.tex
grep -nE "position-based.*\\\\cite\\{[^}]*salehi2021" Soft_Precise_Landing/manuscript.tex

# salehi2021 should NEVER appear in an autonomous-landing-asserting sentence (it is robot-manipulator IBVS):
grep -nE "autonomous landing.*\\\\cite\\{[^}]*salehi2021" Soft_Precise_Landing/manuscript.tex
grep -nE "extensions to aerial platforms.*\\\\cite\\{[^}]*salehi2021" Soft_Precise_Landing/manuscript.tex

# Conversely, lin2022 / zhang2026 should NEVER appear in an IBVS-asserting sentence:
grep -nE "IBVS methods.*\\\\cite\\{[^}]*(lin2022|zhang2026)" Soft_Precise_Landing/manuscript.tex
grep -nE "image features directly.*\\\\cite\\{[^}]*(lin2022|zhang2026)" Soft_Precise_Landing/manuscript.tex
```

Any hit is a defect — fix by moving the cite to the correct framework's sentence or restructuring.

**Detection — definitional sensor-modality errors (visual servoing is VISION-based):**

```bash
# PBVS does NOT use LiDAR or GPS — those are outside visual servoing:
grep -niE "PBVS.*(LiDAR|GPS)" Soft_Precise_Landing/manuscript.tex
grep -niE "pose reconstruction from .*(LiDAR|GPS)" Soft_Precise_Landing/manuscript.tex
grep -niE "position-based visual servoing.*(LiDAR|GPS)" Soft_Precise_Landing/manuscript.tex

# Likewise IBVS shouldn't be associated with LiDAR/GPS as sensors:
grep -niE "IBVS.*(LiDAR|GPS) sensor" Soft_Precise_Landing/manuscript.tex
```

Any hit is a definitional error: the "V" in PBVS/IBVS = VISUAL. Sensors are vision (monocular pose estimation, stereo depth). LiDAR/GPS are sensor-fusion augmentations outside the visual-servoing definition.

**Reference table** (canonical PBVS / IBVS labels — see `feedback_citation_classification_audit.md` for the full verified classification table):

- PBVS (autonomous-landing, vision-based pose): `lin2022, zhang2026`
- IBVS (autonomous-landing, aerial): `chaumette2006, jabbari2014, lee2012, fink2017, xie2016_2, xie2020, lin2023, cho2022` — `lin2023` (robust prescribed-performance IBVS, circle features) replaced `chen2025` as the IBVS landing baseline 2026-06-01; `lin2023` is purely IBVS (NOT adaptive — it fixes its funnel a priori), so unlike the old `chen2025` it is NOT in the Adaptive list.
- IBVS (manipulator, barrier-function visibility): `salehi2021` — only in Para 4, NEVER in Para 2 autonomous-landing taxonomy
- Optic-flow on **moving platforms** (not "stationary"): `herisse2012` — handles moving platform via external heading reference + no FoV constraint
- Optic-flow on stationary 1-D vertical: `izzo2011` (lunar landing), `singhal2025` (own work)
- Adaptive **control**: `paris2020, zhang2026` (`chen2025` removed 2026-06-01 with the baseline swap; its replacement `lin2023` is a fixed-funnel IBVS design, not adaptive)
- Learning-based **control**: `kamath2026`
- Vision-aided **state estimation** (separate class): `bouazza2025` — NOT a control framework, NEVER lump with adaptive/learning controllers

### Rule 15 — `spelling`: paper-wide US/Oxford `-ize`/`-or` convention (locked 2026-05-03)

Memory: implicit (paper-wide spelling pass applied 2026-05-03). The paper uses **Oxford/US spelling**: `-ize`/`-ization` for verb forms and noun derivatives; `-or` (not `-our`) for words like "color"/"behavior"/"favor"; `-er` (not `-re`) for "center"/"meter"; `-zed` for "analyzed".

**Detection — British forms that should be zero in active tex files:**

```bash
# British -ised/-ising/-isation verb forms:
grep -nE "\\b(summarise|stabilise|normalise|characterise|recognise|emphasise|realise|minimise|maximise|linearise|discretise|parameteri[sz]e|regularise|optimise|initialise|specialise|generalise|formalise|penalise|finalise|standardise|categorise|prioritise|synthesise|organise|polarise|customise|theorise|harmonise|legalise|legitimise|materialise|memorise|mobilise|modernise|moralise|neutralise|patronise|personalise|popularise|publicise|randomise|rationalise|sensitise|sterilise|subsidise|symbolise|sympathise|systematise|utilise|conceptualise)(s|d|sing|sed|ses|sation|sations)?\\b" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex

# Common British nouns/spellings:
grep -nE "\\b(behaviour|colour|favour|favourable|fibre|metre|metres|centimetre|centimetres|centred|catalogue|dialogue|defence|offence|practise|practised|licence|licensed)\\b" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex

# "centre" — but PRESERVE "Robert Bosch Centre" (proper noun on author affiliation):
grep -nE "\\bcentre\\b" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
# Filter: any match that is NOT inside "Robert Bosch Centre" is a defect.

# British "analysed":
grep -nE "\\banalys(ed|ing|es)\\b" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
```

Any match (other than "Robert Bosch Centre" proper noun) is a defect. Replace with US/Oxford form. Comments inside TikZ source (`%`-prefixed lines in `frames_planes.tex`, etc.) don't render and may be skipped, but still worth fixing for source consistency.

### Rule 16 — `section-ref`: hardcoded composition pattern for sub/sub-sub-section refs (locked 2026-05-03)

Memory: `feedback_section_ref_composition.md`.

The IEEE TAES class returns only the local counter from `\ref{}` to a subsection / sub-subsection (e.g., "C" instead of "III-C"). To render fully-qualified section numbers, use **hardcoded composition** of `\ref{}` calls:

| Target | Composition pattern | Renders to |
|---|---|---|
| Top-level section | `Section~\ref{section_label}` | "III" |
| Subsection | `Section~\ref{section_label}-\ref{subsection_label}` | "III-A" |
| Sub-subsection | `Section~\ref{section_label}-\ref{subsection_label}\ref{subsubsection_label}` | "III-A2" |

Plus the parent-collapse rule: when a sentence cites multiple sibling subsections (or sub-subsections) of one parent, **cite the parent** instead of composing a sibling range.

**Detection — bare subsection / subsubsection refs that need composition:**

Subsection labels (must be composed with their parent section):
- `outer loop: section` → must be prefixed with `\ref{control design: section}-`
- `inner loop: section` → must be prefixed with `\ref{control design: section}-`
- `stability: section` → must be prefixed with `\ref{control design: section}-`
- `image parameters: section` → must be prefixed with `\ref{background: section}-`
- `quadrotor dynamics: section` → must be prefixed with `\ref{background: section}-`
- `problem statement: section` → must be prefixed with `\ref{background: section}-`
- `multi init: subsec`, `test conditions: subsec`, `speed envelope: subsec`, `comparison: subsec` → must be prefixed with `\ref{experimentation: section}-`

Subsubsection labels (must be composed with parent subsection AND grandparent section):
- `virtual image point control: section` → `\ref{control design: section}-\ref{outer loop: section}\ref{virtual image point control: section}`
- `optic flow control: section` → `\ref{control design: section}-\ref{outer loop: section}\ref{optic flow control: section}`
- `acceleration conditioning: section` → `\ref{control design: section}-\ref{outer loop: section}\ref{acceleration conditioning: section}`
- `yaw control: section` → `\ref{control design: section}-\ref{inner loop: section}\ref{yaw control: section}`
- `so3 inner loop: section` → `\ref{control design: section}-\ref{inner loop: section}\ref{so3 inner loop: section}`

```bash
# Bare subsection ref (must be composed) — these labels should NEVER appear standalone after `Section~\ref{`:
grep -nE "Section~\\\\ref\\{(stability|outer loop|inner loop|image parameters|quadrotor dynamics|problem statement|virtual image point control|optic flow control|acceleration conditioning|yaw control|so3 inner loop|multi init: subsec|test conditions: subsec|speed envelope: subsec|comparison: subsec): (section|subsec)\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex

# Old stale labels (must be zero — renamed 2026-05-04):
grep -nE "\\\\ref\\{control strategy: section\\}|\\\\label\\{control strategy: section\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "\\\\ref\\{normalized pid: section\\}|\\\\label\\{normalized pid: section\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "\\\\ref\\{ppc optic flow: section\\}|\\\\label\\{ppc optic flow: section\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "\\\\ref\\{fov cone: section\\}|\\\\label\\{fov cone: section\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
grep -nE "\\\\ref\\{target image funnel: section\\}|\\\\label\\{target image funnel: section\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex

# Sibling-range that should collapse to parent (pattern: Sections~\ref{X}-\ref{A}--\ref{B}):
grep -nE "Sections?~\\\\ref\\{[^}]+\\}-\\\\ref\\{[^}]+\\}--\\\\ref\\{[^}]+\\}" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
# Manually verify: if the two endpoints share a parent that is *most* of the range, prefer parent reference.

# Plural mismatch (Sections~ with single ref):
grep -nE "Sections~\\\\ref\\{[^}]+\\}(?![^a-zA-Z]*\\\\ref|\\}--)" Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
# Manually verify: if only one ref follows, change "Sections~" → "Section~".
```

Allowed top-level refs (no composition needed — `\thesection` is roman numeral): `\ref{background: section}`, `\ref{control design: section}`, `\ref{experimentation: section}`, `\ref{conclusion: section}`. These render as "II", "III", "IV", "V" directly.

## Output format

Group violations by rule id; within each rule, list `file:line: <offending text>` lines. End with a one-line summary and overall verdict.

```
== Rule 1 (this) — bare demonstrative ==
manuscript.tex:67: this leads to a parallel two-funnel architecture
control_formulation.tex:189: This bound implies the zeta_2 envelope...
...

== Rule 2 (name) — stale controller name ==
supplemental.tex:142: the DF-ASMC framework's leakage-adaptive...
results.tex:67: PLASMC outperforms all four baselines...
...

== Rule 3 (visibility) — corner-point in reader-facing prose ==
manuscript.tex:114: the corner points stay inside the FoV throughout descent...
...

== Rule 4 (funnel-naming) ==
(none)

== Rule 5 (legacy) ==
(none)

== Rule 6 (corner) ==
(none)

== Rule 7 (subscript) ==
(none)

== Rule 8 (trajectory) ==
(none)

=========================================================
SUMMARY: 3 rules triggered, 14 total defects across 4 files.
VERDICT: REVISION REQUIRED — address Rules 1/2/3 before compile.
```

If zero defects: `VERDICT: CLEAN — prose conforms to all 10 style rules.`

## Fix workflow (manual)

`/prose-audit` does NOT rewrite. For each flagged line:
1. Read the surrounding paragraph.
2. Decide between: (a) attach the missing noun (Rule 1), (b) rename the term (Rules 2/4/6), (c) recast the sentence in physical language (Rule 3), (d) drop the legacy comparison (Rule 5), (e) correct the subscript/frame (Rule 7), or (f) rename Run→IC / add "trajectory" suffix (Rule 8).
3. Apply via `Edit`.

Batch all fixes into a single commit titled `"Prose-audit pass: Rule <N> cleanup"`.

## When to use

- Before any manuscript submission/resubmission — style defects are free points for a reviewer.
- After a controller/paper rename (Rules 2–4 will flag all stale references).
- After a large rewrite pass — catches residual legacy phrasing.
- As the final step before compile, after `/manuscript-review` content-pass is clean.

## When NOT to use

- During active drafting — premature style nitpicks slow down content work.
- On `Drafts/` files — those are frozen pre-edit backups; mutating them defeats the backup.
- On hook files, skill files, or code comments — this skill is manuscript-scope only.

## Notes

- This skill is purely mechanical. A line that grep flags but is actually correct in context (e.g., `this funnel architecture` — legitimate `this + noun`) must be verified by the caller before acting. The audit errs on the side of over-reporting.
- When adding a new writing rule to memory, extend this skill's rule list in the same commit so enforcement does not drift from the documented rule set.
- Grep patterns may need escaping tweaks depending on shell; on Git Bash / MSYS2 the patterns above should work as written with the Grep tool.
