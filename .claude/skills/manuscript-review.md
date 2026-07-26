---
name: manuscript-review
description: Critical-reviewer pass over the MDF-ASMC manuscript (manuscript.tex + control_formulation.tex + results.tex + supplemental.tex + block_diagram.tex), producing IEEE-TAES-style feedback covering novelty, theoretical rigor, experimental validation, comparison fairness, numerical consistency against live .mat data, IC consistency, supplement cross-references, uniformity, paper flow, and writing quality.
---

# /manuscript-review

Walk the manuscript as a hostile IEEE TAES reviewer would: assume every numerical claim is stale until proven otherwise, and every novelty claim is someone else's prior work until the literature is checked.

## What it reviews

All manuscript sources in `Soft_Precise_Landing/`:

- `manuscript.tex` — abstract, introduction, contribution list, qualitative comparison table, conclusion
- `control_formulation.tex` — preliminaries, assumptions, control synthesis, remarks, Lyapunov proofs
- `results.tex` — multi-init table (V), deep-sweep Pareto analysis, speed envelope, comparison table, figure captions
- `supplemental.tex` — proofs, derivations, deep-sweep details, extended results (S-prefixed numbering)
- `block_diagram.tex` — TikZ block diagram with signal labels (notation must match body text)

And the live MATLAB data that the manuscript claims to report:

- `MATLAB/Multi_init_cond/Datasets/*_multi_init.mat` — multi-init sweep results (Table V source)
- `MATLAB/Comparison/Datasets/result_ctrl_{1-5}.mat` — comparison results (Table VI/VIII source)
- `MATLAB/Sweeps/Datasets/sweep_deep.mat` — deep-sweep Pareto data (supplement S3-D source)
- `MATLAB/Common/bestParam.mat` — locked-in gains (Table III source)
- `MATLAB/Multi_init_cond/multi_Init_Var.m` — canonical ICs (Table IV source)

## Review methodology

Work in this order. Each step is a real check, not a formality.

### 1. Data freshness — verify manuscript numbers against live .mat files (mandatory first pass)

The #1 source of embarrassment is stale numbers. Every table and claim must be checked against the actual MATLAB output, not just cross-referenced within the tex files.

**Step 1a: Load live data.** Use Python + scipy.io to read the .mat files listed above. Extract:
- Multi-init: per-trajectory landing count, mean/worst t_f and p_xy for each trajectory type
- Comparison: per-controller per-trajectory t_f and p_xy (use 39.99s or similar cutoff for crash detection)
- Deep sweep: baseline aggregate, Pareto candidate count, top candidate metrics
- bestParam.mat: current gain values
- multi_Init_Var.m: current IC definitions

**Step 1b: Cross-check results.tex tables.** For each table row, compare the manuscript number to the .mat value. Flag any mismatch > 1% as STALE. Common drift patterns:
- Table updated for old IC set but .mat reflects new ICs
- Comparison rerun after gain/trajectory changes but table not refreshed
- Deep-sweep rerun with new ICs invalidating old Pareto counts

**Step 1c: Cross-check manuscript.tex claims.** Every worst-case / mean / envelope number in the abstract, contribution list, and conclusion must match the current results.tex tables. Grep:

```bash
grep -nE "cm|1\.[0-9]×|7\.[0-9]|8\.[0-9]|32|25/25|50/50|49/50" manuscript.tex
```

**Step 1d: Cross-check supplement.** The supplement's deep-sweep section (S3-D) claims specific Pareto/fragile/inert counts and parameter names. Verify against sweep_deep.mat.

**Step 1e: Gain table.** Table III gains must match bestParam.mat exactly. Extract each field and compare.

Also verify: the parameter count in "deep sweep over N parameters" claim matches the actual row count of the sweep log.

### 2. IC and cross-reference consistency

**Initial conditions.** The canonical ICs are defined in `MATLAB/Multi_init_cond/multi_Init_Var.m`. Every IC reference in the manuscript must match:

- Table IV (results.tex): IC numbering, position vectors, and description labels
- Supplement text: any "IC_k = [x,y,z]" inline references
- Figure captions: any IC-specific figure must reference the correct IC number
- Comparison harness IC (MATLAB/Comparison/InitVar.m): must match one of the canonical ICs

Use "IC" not "Run" for initial conditions throughout. Grep:
```bash
grep -nEi "Run [0-9]|Run~[0-9]" results.tex supplemental.tex manuscript.tex
```
Any match is a defect — replace with "IC".

**Supplement cross-references.** The supplement references main-paper sections and tables by hardcoded numbers (e.g., "Section~IV-F", "Table~V"). These break silently when results.tex restructures. Grep:
```bash
grep -nE "Section~[IVX]+-[A-G]|Table~[IVX]+|Fig\.~[0-9]" supplemental.tex
```
For each match, verify the target still exists with that number in the main paper. Common failures:
- Subsection letters shift when a subsection is added/removed/reordered
- Table numbers shift when a table is added/removed
- "Section~IV-F" → should be "Section~IV-E" after restructuring

**Block diagram signal labels.** Every signal name in block_diagram.tex must match the notation in control_formulation.tex. Grep block_diagram.tex for `tau`, `T_`, `F_` and verify subscripts and frame prefixes match.

**Termination convention.** `visualControl_comparison.m:383,812` terminates on `alt_above = abs(I_p_c(3) - x_t(3,idx)); if alt_above <= zf` — i.e., UAV altitude ABOVE THE TARGET, not absolute altitude. `results.tex:4` defines `z_f` as "UAV altitude above the target"; `results.tex:21` labels it "Above-target gap". On Cases 2/5 (ship-deck heave) absolute altitude at touchdown can be ~0.40 m while above-target gap is 0.20 m. Grep any prose claiming "altitude falls below" or "absolute altitude at touchdown":

```bash
grep -nE "absolute altitude|altitude falls|altitude first" results.tex supplemental.tex manuscript.tex
```

Any match outside the `results.tex:4` definition is a defect — the reader must resolve every `z_f` reference to above-target gap.

### 3. Novelty and contribution claims

- "To the best of the authors' knowledge, the first ..." claims — spot-check the reference list for Bechlioulis/Rovithakis funnel/PPC constructions, and for prior dual-funnel or cascaded-PPC schemes in visual servoing.
- Table I (qualitative comparison) — the Proposed row's checkmarks must be defensible; a reviewer will attack any column where all prior works are marked × (especially "Scale-Free", "Soft Landing", and any novelty axis).
- Contribution list in the introduction must be tight: each bullet should be a *technical* contribution, not a restatement of the results section.
- **Validate sweeping critiques against the cited literature.** Per `feedback_validate_critiques_against_cited_works.md`, any §I claim of the form "no prior work does X", "the literature lacks Y", or "leaves Z unconstrained" must be cross-checked against each cited work — especially `\cite{herisse2012}` and `\cite{singhal2025}`, both of which regulate optic flow. A critique that's correct against one subset of citations and false against another must be **scoped** ("the cited IBVS extensions ... regulate image-position features only" rather than "no prior work regulates optic flow").
- **§I Para 1 carries the qualitative functional requirement; §IV.A carries the numerical thresholds.** Per `feedback_functional_requirement_framing.md` and `feedback_soft_precise_definition_location.md`, §I Para 1 should contain the two-clause functional-requirement statement (smooth convergence of position before landing + velocity as approaching the touchdown altitude). Numerical thresholds (`0.08 m / 0.20 m/s / z_f = 0.20 m`) live in `results.tex:4` (§IV.A intro) and Table II only. Flag any drift: numbers leaking into §I, or the qualitative statement being missing.
- **Distinguish "evaluation instant" from "convergence window" in temporal-dynamics prose.** MATLAB evaluates `precise` and `soft` at the *same* `idx` (when `alt_above ≤ z_f`), but the underlying convergences operate over *different* phases (position during descent; velocity in the terminal phase). Prose that conflates these — e.g., "both criteria are enforced at the touchdown instant" — obscures the dual-funnel temporal structure.
- **No overclaim of proven properties.** Per `feedback_no_overclaim_proven_properties.md`, the §I narrative must not claim properties that no theorem in §III actually proves. The theorems we have: Theorem 1 (`h_e` funnel invariance), Theorem 2 (yaw UUB), Corollary 1 (kinematic visibility). Soft-touchdown (`‖v_rel(t_touch)‖ ≤ 0.20`) is **empirical** (25/25 in §IV), not a closed-form bound. Banned phrasings in §I and abstract: "transient specification required for soft touchdown", "finite-time bound on the relative velocity", "guarantees soft touchdown", "drives the relative velocity to zero in finite time".

### 4. Theoretical rigor (control_formulation.tex)

Flag the following classes of issue:

- **Approximation sneaking into proofs.** Any `≈` or "negligible" inside a Lyapunov derivative. MDF-ASMC's Theorem 1 uses `σ^⊤ sat(E^{-1}σ) ≈ |σ|^⊤` inside the boundary layer — this is an approximation, not an equality, and a reviewer will push.
- **Saturation guards that weaken guarantees.** Remark 1's funnel-saturation clamp `ε_S = 0.05` on S_2 is necessary for implementation but formally weakens the funnel envelope; the proof text should acknowledge this explicitly, not silently rely on an unsaturated S_2.
- **Cone-clamp claims.** Any "never activates on any run" sentence must be verified against the current (post-ship-deck) multi-init sweep, not an older heave-only run.
- **Assumption sharpness.** Assumption 1's unknown-but-existing bounds (β_min, β_max) — are they uniform in time? What happens at the ground-effect transition?
- **Yaw-channel ASMC.** If the stability analysis is claimed to "apply verbatim" with some parameter identification, demand the explicit substitution rather than accepting the handwave.
- **Integrator anti-windup, cone-clamp projection, ε_S clamp, attitude cone** — each is a hard nonlinearity in the closed loop. The proof should either pass through them or explicitly bound their contribution.

### 5. Experimental validation

- Hardware: is there *any* hardware result, HIL, or even Gazebo/AirSim run? TAES reviewers routinely demand at least HIL for a landing paper. If absent, the future-work sentence must be prominent.
- Disturbance realism: pixel noise SNR, ground-effect model, actuator delay, wind — are all enabled simultaneously in the reported results?
- Statistical significance: 25/25 is a binary count; a reviewer may ask for mean ± std across an rng sweep, not a single seed.
- Deck-motion harshness: document the roll/pitch amplitudes + heave explicitly where the Linear/Circular results are introduced, so a reviewer can't mistake them for trivial kinematic targets.

### 6. Comparison fairness

Each baseline (`Lin 2022`, `Zhang 2026`, `Lin 2023`, `Cho 2022`) must pass all of:

- Gains either author-reported or sourced from author correspondence; if retuned, say so and bound the retuning effort (e.g., "matched MDF-ASMC's deep-sweep budget").
- Disturbance profile identical to MDF-ASMC case 1 (noise, delay, GE, saturation) — footnote should state this.
- Feed-forward / observer terms from the original paper are either enabled or the exclusion is justified (Lin 2022's target-velocity FF exclusion is a known attack surface — own it in the text).
- Failure modes that look catastrophic (Zhang's free-fall, Chen's structural violation) must be diagnosed in the text, not just dagger-footnoted in the table.

### 7. Uniformity and paper flow

A uniformity pass catches drift that a content-focused reviewer will miss but a copy-editor will not. Run each of these greps and reconcile any outlier style.

**Cross-reference style** — pick one form per reference type and apply globally. IEEE TAES convention:

```
Table~\ref{}    Fig.~\ref{}    Figs.~\ref{}    Section~\ref{}    Theorem~1    Remark~1
```

Flag outliers with:

```bash
grep -nE "\\\\textit\{Table\}|\\\\textit\{Fig|Figure~\\\\ref|Sec\.~\\\\ref" manuscript.tex control_formulation.tex results.tex supplemental.tex
```

Note: theorem-environment *block headers* at the start of a paragraph (`\textit{Theorem 1}:`, `\textit{Remark 1 (...)}.`) are intentional and should be left alone. The fix is only for *inline back-references* in running text.

**Hyphenation** — compound nouns must be hyphenated when used prenominally as adjectives and unhyphenated when used as nouns. Core terms in this paper:

| Noun form | Adjective form |
|---|---|
| regulates optic flow | optic-flow error, optic-flow formulation |
| achieves a soft landing | soft-landing controller, soft-landing rate |
| visual servoing (literature) | visual-servoing law, visual-servoing dynamics |
| ship deck (surface) | ship-deck motion, ship-deck Linear |

Grep with:

```bash
for term in "optic flow" "soft landing" "soft touchdown" "visual servoing" "ship deck"; do
  grep -c "$term" manuscript.tex control_formulation.tex results.tex supplemental.tex
  grep -c "${term// /-}" manuscript.tex control_formulation.tex results.tex supplemental.tex
done
```

Any file with a non-zero count on *both* forms is a candidate for audit. The rule of thumb: if the next word is a noun (`error`, `law`, `rate`, `controller`, `motion`), hyphenate the compound; otherwise don't.

**Abbreviation first-use expansion** — every acronym must be expanded on first appearance in the abstract *and* on first appearance in the body (these are separate introductions). Audit list:

```
MDF-ASMC, PPC, IBVS, PBVS, SO(3), GUUB, UUB, AEDO, ASMC, AFOSMC, AWGN, EKF, HIL, VTOL
```

**Number/unit consistency** — pick one unit per physical quantity across the whole paper:

| Quantity | Prose unit | Table unit |
|---|---|---|
| touchdown error xy_e | cm | m (4 decimal places) |
| relative speed v_rel | m/s | m/s |
| landing time t_f | s | s |
| altitude z | m | m |

The xy_e split (cm in prose, m in tables) is the only allowed cross-unit mismatch, and it must be *consistent*: every prose number in cm, every table number in m, never the reverse.

**Notation consistency** — bold for vectors, plain for scalar components:

```
\boldsymbol{p}_2   \boldsymbol{\sigma}   \boldsymbol{\kappa}   \boldsymbol{\zeta}_2
p_{2_k}            \sigma_k              \kappa_k              \zeta_{2_k}
```

Audit with `grep -nE "p_\{2_0\}|\\\\boldsymbol\{p\}_\{2_0\}"` and reconcile drift.

**Mathematical-notation first-use audit.** Every math symbol used in the main paper must be either (i) defined inline at first use, (ii) defined in the supplement with a forward-cite at first main-paper use, or (iii) a universally-recognized convention whose exclusion would not raise a reviewer eyebrow. The coverage baseline is `reference_notation_audit.md`; high-priority watchlist (as of 2026-04-24):

- `\mathcal{R}` (camera-sensor resolution), `f` (focal length) — used in the main paper but formally defined only in supplement §S1-A.
- `\hat{\boldsymbol{e}}_3` ($[0,0,1]^\top$) — never defined; used in `\boldsymbol{c}_h`, `\dot{\boldsymbol{s}}_e`, `\boldsymbol{h}_\text{d}`.
- Symbol clashes: same glyph with two meanings in nearby equations. Example caught 2026-04-24: $\varepsilon$ is the cone-clamp numerical floor (line 239 of `control_formulation.tex`) AND the entry-wise value of $\mathcal{E}=\text{diag}(\varepsilon_k)$ (line 215, ASMC boundary-layer width). Two different $\varepsilon$'s in the same section — rename one.
- Stale cross-references to renamed/restructured Assumptions and Theorems. Example caught 2026-04-24: `\textit{Assumption~2}` in `control_formulation.tex:222` survived a split that renamed it to "Property 1" everywhere else. Grep `Assumption~[0-9]|Property~[0-9]|Theorem~[0-9]|Corollary~[0-9]|Remark~[0-9]` and reconcile with the actual statement labels.
- Assumption/Property/Theorem numbering consistency after insertions. When adding a new Theorem, every *later* statement that cites "Theorem~N" must be bumped if inserted before; when splitting an Assumption, every cross-reference must be audited.

Implicit conventional symbols ($R$, $R_{33}$, $\Pi_{[-\pi,\pi]}$, $^\vee$, $\,^\mathcal{V}\boldsymbol{v}_b$, $\,^\mathcal{V}\boldsymbol{\omega}_b$, etc.) are on the rigor-pass watchlist — flag them only if doing a strict IEEE-TAES first-use audit.

**Word-choice audit (2026-04-25 additions).** Grep the active tex for these banned/discouraged words and reconcile each hit:

- `channel` — banned. Replace with "axis"/"axes" (per-coordinate), "loop"/"control loop" (closed-loop structure), or "component" (vector-element index). Memory: `feedback_no_channel_word.md`.
- `(u, v)` as pixel-axis labels — banned. Use $(\hat{X}_\text{c}, \hat{Y}_\text{c})$ for axes / directions, $(\,^\mathcal{C}\hat{x}, \,^\mathcal{C}\hat{y})$ for feature-point components. Memory: `feedback_image_plane_axis_vs_component.md`.
- "harness", "implementation code", `\texttt{...}` containing a MATLAB function call (`rng(...)`, `traj_Gen`, `awgn(...)`) — implementation-detail leaks. Memory: `feedback_trim_implementation_details.md`.
- "image-orientation-error dynamics" / "α_e dynamics" / improvised variants — use "image orientation kinematics" / "image position kinematics" only. Memory: `feedback_kinematics_phrasing.md`.
- "descent velocity" in general-landing prose (not classical-1D-Herissé context) — use "UAV–target relative velocity" instead. Soft-touchdown criterion is on the 3-D relative speed.
- "realistic", "idealised"/"idealized", "noiseless", "disturbance-free", "full-disturbance", "full robustness model" — banned. Paper presents a single regime "under the disturbance model (Table II)". Memory: `feedback_single_regime_presentation.md`.
- `IC$N$` (unsubscripted), `\text{IC}_N`, or `IC $[\ldots]$` (coordinates with no index) — banned. Use `IC$_N$` for indexed ICs, bare `IC` for generic references. Memory: `feedback_ic_notation.md`. Grep recipe:
  ```bash
  grep -nE 'IC\$[0-9]|\\text\{IC\}|IC \$\[' Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
  ```

**Before any structural edit (subsection deletion, large rewrite, notation-wide rename):** back up all 6 active tex files to `Drafts/<name>_vN.tex` per `feedback_v2_backups_before_structural_edits.md`. Current highest version in use: `_v2` (2026-04-25 pre-idealised-removal snapshot).

**Supplement → main paper cross-references.** Supplement compiles separately, so any `\eqref{...}` or `\ref{...}` to a main-paper label resolves as `??`. Grep `supplemental.tex` for `\eqref{` and `\ref{` and verify each label exists in `supplemental.tex` itself; main-paper references must be rewritten as text ("Section~III-A2 of the main paper", "Theorem~2 of the main paper", "Assumption~1 of the main paper"). Memory: `feedback_supplement_cross_refs.md`.

**Figure-script ↔ caption sync.** When a caption or notation changes, the matching figure-generating script must also change and the PDF must be regenerated. Grep `scripts/make_*.py` for the affected symbols/values:
- Notation leaks to watch: `set_xlabel.*\$u\$|\$v\$` (image-plane axes should use `\,^\mathcal{C}\hat{x}`, `\,^\mathcal{C}\hat{y}`); `w_e` (should be `h_e`); `phi_max|deg2rad\(35` (should be `theta_cap = deg2rad(60)`, matching Property 1).
- After fixing, run `python scripts/make_plasmc_plots.py` / `make_multi_init_plots.py` etc. and verify PDF timestamps. Memory: `feedback_figure_label_sync.md`.

**Section-sequence audit (supplement vs main paper).** The supplement subsection order should follow the main paper's §III/§IV flow:
- §S1 (Virtual-Camera + Baseline) → main-paper preliminaries
- §S2 (Detailed Derivations): Optic-Flow Remarks → SO(3) Construction → Theorem 1 Proof → Theorem 2 Proof, matching main-paper §III.A.2 → §III.B.2 → §III.C
- §S3 (Simulation Results): Idealised → Realistic → Internals → Locked Params → Deep Sweep → Speed Envelope → Per-Controller Failure → Summary, matching main-paper §IV.B → §IV.C → summary

Reorder if a new subsection has been inserted out of place.

**Frame superscripts** — the Nomenclature table lists bare symbols (`\boldsymbol{r}`, `\boldsymbol{v}`) but the body uses frame-decorated forms (`^\mathcal{I}\boldsymbol{r}`, `^\mathcal{B}\boldsymbol{\omega}`, `^\mathcal{C}\boldsymbol{\hat{r}}`). Ensure the Nomenclature carries a one-line legend explaining the leading superscript convention.

**Subscript semantic consistency** — the same subscript letter must not carry two meanings across the manuscript. Known convention:

| Subscript | Meaning | Example |
|---|---|---|
| `_u` | control input (plant actuator command) | `\,^\mathcal{B}\boldsymbol{\tau}_u`, `\,^\mathcal{B}T_u`, `\,^\mathcal{B}\boldsymbol{F}_u` |
| `_d` (on disturbance terms) | disturbance | `\boldsymbol{d}_h`, `\,^\mathcal{V}\boldsymbol{F}_\text{d}` |
| `_d` (on reference signals) | desired | `\,^\mathcal{I}\boldsymbol{a}_\text{d}`, `\psi_\text{d}`, `R_\text{d}` |
| `_e` | error | `\boldsymbol{e}_R`, `\boldsymbol{h}_\text{e}`, `\boldsymbol{s}_\text{e}` |
| `_b` | body-frame quantity | `\boldsymbol{v}_\text{b}`, `\boldsymbol{\omega}_\text{b}` |
| `_t` | target | `\boldsymbol{r}_\text{t}`, `z_\text{t}` |

**Critical rule:** torque (`\tau`) and thrust (`T`) are plant control inputs — they must always use `_u`, never `_d`. The body-frame prefix `\,^\mathcal{B}` is mandatory on both. Desired acceleration (`a_d`) and heading (`\psi_d`) may use `_d` because no disturbance counterpart exists for those symbols.

Audit with:

```bash
grep -nE "\\\\tau_\\\\text\{d\}|T_\\\\text\{d\}" control_formulation.tex results.tex supplemental.tex block_diagram.tex
```

Any match is a defect — these should be `\tau_u` and `T_u` respectively.

**Paper-flow checks** — read the following transitions specifically and flag any that feels abrupt:

1. End of introduction → first formal section (Section II): there must be a bridge sentence, not a hard cut into notation.
2. Dual-funnel exposure (single-run internals) → multi-init subsection (statistical aggregation): a transition sentence is required.
3. Abstract, introduction contribution list, and conclusion: these three restate the same headline claims. Pick **canonical phrasing for each of the five core claims** and reuse verbatim across all three locations. Drift here is the #1 source of stale-number embarrassment.
4. Results §summary vs manuscript §conclusion: differentiate function — the results summary should be *numerical* (every sentence has a number), the conclusion should be *methodological* (every sentence has a claim about the method). If both carry the same numbers, one of them can be tightened.
5. Long bucket paragraphs in the introduction (>250 words): split on topic transitions, especially where self-citation appears alongside external literature.

### 8. Writing, figures, references

- Every `\ref{}` / `\cite{}` resolves (run `/verify-tex` for a mechanical check).
- Figure captions state axes, units, and the data source run.
- No figure caption quotes a number that Table V contradicts.
- Reference list includes the *latest* versions of the baseline papers (journal if available, not preprint).
- Acronym on first use, consistent notation between `control_formulation.tex` and `results.tex`.
- **Citation style (author-less):** the manuscript refers to prior work by citation number only — no author surnames, no "X et al.", no "X and Y", and no "X 2022" year tags. Grep patterns to flag:
  `[A-Z][a-z]+ et al\.\s*\\cite`, `[A-Z][a-z]+ and [A-Z][a-z]+\s*\\cite`, `[A-Z][a-z]+~\\cite`, `\b(Lin|Zhang|Chen|Cho|Xie|Herisse|Fink|Izzo|Lee|Leok|McClamroch) (19|20)\d\d`.
  Replace with `\cite{key}`, `the work in \cite{key}`, `\cite{key1,key2}`, or a Table-row `\cite{key}` cell as appropriate. Table I and the comparison table rows must use `\cite{key}` as the leading label, not `Author~\cite{key}`.
- **Hard-coded baseline citation numbers in comparison figures.** The 5-controller comparison figures embed bracketed IEEE citation numbers in the figure legends/titles ("Baseline A [1]", "Baseline B [2]", etc.). Numbers are hard-coded — matplotlib mathtext does not render `\cite{}`, and `text.usetex=True` requires a LaTeX install on PATH that the user does not have. Two scripts carry four dicts:
  - `scripts/make_comparison_plots.py`: `CTRL_DISPLAY_MAIN` (for `comparison_combined_circular.pdf`, main paper) + `CTRL_DISPLAY_SUPP` (for `comparison_traj3d_combined.pdf`, supplement).
  - `scripts/make_comparison_multi_speed_plots.py`: `CTRL_TITLE_MAIN` (for `comparison_multi_speed_circular.pdf`, main paper) + `CTRL_TITLE_SUPP` (for the other three trajectories, supplement).

  Drift trigger: any new `\cite{}` added before `lin2022`/`zhang2026`/`lin2023`/`cho2022` in either bibliography, any cite removed, or any cite moved earlier (IEEE ordering is by first occurrence). Audit recipe:
  1. Compile `manuscript.tex` and `supplemental.tex`.
  2. Grep their rendered PDFs for the bracketed numbers next to each baseline and confirm they match the four cite-keys' actual citation numbers in the reference list.
  3. If they drift, update the four dicts in the two scripts, regenerate the affected PDFs (run both scripts), and update `project_baseline_citation_numbers.md` to record the new numbers.

### 9. Compiled-PDF pass (blank space, overflow, overwritten text)

After a compile, open the rendered PDF (`IBVS_Manuscript.pdf`) and check issues that only surface in the typeset output. Use `pymupdf` (`import fitz`) — the PDF is too large for plain text extraction.

**Blank-space audit.** Walk every page and record top/bottom margins plus the largest internal vertical gap between text blocks:

```python
import fitz
d = fitz.open('IBVS_Manuscript.pdf')
for i, p in enumerate(d):
    blocks = [(b[1], b[3]) for b in p.get_text('blocks') if b[4].strip()]
    blocks.sort()
    merged = []
    for y0, y1 in blocks:
        if merged and y0 <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], y1))
        else:
            merged.append((y0, y1))
    gaps = sorted(((merged[k+1][0]-merged[k][1], merged[k][1], merged[k+1][0])
                   for k in range(len(merged)-1)), reverse=True)
    top_blank = merged[0][0] if merged else 0
    bot_blank = p.rect.height - (merged[-1][1] if merged else 0)
    print(f"p{i+1} top={top_blank:.0f} bot={bot_blank:.0f} max_gap={gaps[0] if gaps else ''}")
```

Flag any page with `top_blank > 60`, `bot_blank > 80`, or `max_gap > 60` pt. Distinguish:

- **End-of-paper figure spill** (last 3–5 pages with >200 pt blank): floats have been pushed past the text. Fix by converting one or more `figure*` → `figure` (single-column), reducing `width` from `0.95\textwidth` to `0.75–0.85`, pairing two related plots into one subfigure grid, or moving low-value figures to an appendix.
- **Mid-body gap** (internal gap >60 pt on a text page): an oversized float broke LaTeX's packing. Try `[!tbp]` placement, shrink the offending figure by ~15%, or move it to the adjacent column.
- **Oversized table** eating >⅓ page: collapse duplicate header rows, drop decimal precision where the table unit already implies it, combine split columns (`mean / worst` → `mean (worst)`), or switch to `\small` / `\footnotesize`.

**Overwritten / overlapping text.** Use `rawdict` span bboxes to find pairs whose rectangles overlap in both x and y by more than 2 pt. Filter out math accents — a diacritic (ˆ, ˜, ˙, ¯, ´) over its base glyph is correct typography, not an overlap. Real defects look like:

- Legend entries stacked at the same coordinate: `('(0, 0, 5)', '(0, 0, 7)', 40 pt, 2 pt)` — adjacent legend labels colliding horizontally. Fix in the figure-generation script, not LaTeX.
- Axis tick labels colliding (`2.0` over `1.5`) — axis is too cramped; enlarge the subplot or thin the ticks.
- Figure-internal axis labels (`x [m]`) duplicated at the same coordinate — subplot grids re-emitting shared labels. Use `sharex=True` / `sharey=True`.
- Table cells where `×` / `✓` glyphs overlap neighbouring column text — column is too narrow; widen or switch to `tabularx`.
- Body text (not math) where two spans overlap substantially — an `\hspace` collision or a rogue `\raisebox`. Grep for those macros near the reported page.

**Stale content baked into figure PDFs.** After an author-less citation rewrite (§8), the LaTeX is clean but the figure legends often still carry `Lin 2022`, `Zhang 2026`, etc. Scan each figure's rendered text with `page.get_text()` and grep for the baseline author-year tags. Fix by regenerating the figure with numeric citation keys or plain role labels (e.g., `Baseline A`, `Proposed`).

**Label overflow / wrap.** Watch for wrapped labels inside figures: `(2, \n -2, 5)` indicates a legend box too narrow for its content. Widen the legend, shorten the label, or move the legend outside the axes.

**Template placeholders.** `VOL. XX, NO. XX`, `XXXXX 2020`, `AUTHOR NAME` in the running header/footer are expected until the journal fills them in at typesetting — do not file as a defect, but do verify the `\markboth{}` line carries the correct author surname list.

## Output format

Produce a single reviewer report with these sections. Keep it terse and actionable — a real reviewer writes in bullet points, not paragraphs.

```
## Summary of Contributions
(2-3 bullets, in the reviewer's own words — NOT copied from the abstract)

## Recommendation
Accept / Minor Revision / Major Revision / Reject (with one-sentence justification)

## Major Concerns
(numbered list — each item is a concrete, citable defect: stale number, proof gap, unfair comparison, missing experiment. Each item ends with the specific action the authors must take.)

## Minor Concerns
(numbered list — writing, notation, missing citations, figure hygiene)

## Questions for the Authors
(3-5 pointed questions the authors must answer in the rebuttal)
```

## When to use

- Before any manuscript submission or resubmission.
- After a large results refresh (e.g., retuning, trajectory profile change) — stale conclusion numbers are the #1 drift.
- When the user asks for a "reviewer pass", "critical review", or "red-team the paper".

## Notes

- Do NOT edit the manuscript during a review pass. The output is feedback only; the user decides which concerns to act on.
- Always run `/verify-tex` as part of the pass — unresolved refs are free points for a hostile reviewer.
- If a major concern is factual (e.g., "the 32-parameter claim is wrong"), verify it with grep before writing it up; a reviewer report that's itself wrong is worse than no report.
