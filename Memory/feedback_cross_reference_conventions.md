---
name: feedback_cross_reference_conventions
description: LaTeX cross-reference conventions — no forward \eqref in subsection intro/roadmap paragraphs; IEEE-TAES composed \ref{sec}-\ref{subsec} with parent-over-sibling rule; supplement→main-paper uses text references, not \eqref
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

Consolidated LaTeX cross-reference conventions for the manuscript + supplement. Merges three feedback memories: backward-only eqrefs in intros, IEEE-TAES section-ref composition, and supplement→main-paper text references.

## 1. Subsection intro / roadmap paragraphs use BACKWARD `\eqref{}`s only (locked 2026-05-07)

Intro / roadmap paragraphs must reference quantities via **backward `\eqref{}`s only** — equations the reader has already passed. NEVER forward-reference an equation introduced later in the same subsection or a downstream subsection (the reader hits the symbol before reaching the eq, defeating the clarification purpose).

| Where | Allowed eqref targets |
|---|---|
| §III.A intro | §II equations (`quadrotor dynamics`, `s dot`, `h dot`, `alpha dot`) |
| §III.B intro | §II equations + §III.A equations |
| §IV intro | All §II + §III equations |

**Why:** Intro paragraphs are roadmaps — their purpose is a framing the reader can immediately resolve. A forward `\eqref{}` to an unseen equation forces a flip-ahead, the opposite of "intro". Backward `\eqref{}`s anchor the symbol to a prior definition the reader already has.

**Example (locked 2026-05-07):** §III.B intro mentions "the body-frame torque $\,^\mathcal{B}\boldsymbol{\tau}_u$" — user directed NOT to forward-ref `\eqref{so3 torque: equation}` (in §III.B.2) but to backward-ref `\eqref{quadrotor dynamics: equation}` (§II.A). Final: "The SO(3) tracker produces the body-frame torque $\,^\mathcal{B}\boldsymbol{\tau}_u$ of \eqref{quadrotor dynamics: equation}." §III.A intro likewise: "the commanded acceleration $\,^\mathcal{I}\boldsymbol{a}_\text{d}$ of \eqref{h dot: equation}" (eq h dot is in §II.B.2, prior to §III.A).

**How to apply:** when drafting an intro `\eqref{}`, ask "is this equation upstream of the reader's current position?" If yes, OK; if no, drop the eqref. Inline math symbols introduced downstream: leave un-annotated in the intro (they get defined when the reader reaches the relevant subsection). Symbols defined upstream in the SAME subsection are OK to backward-ref (a §III.B.2 sentence may ref a §III.B.1 eq).

## 2. Section-reference composition — IEEE TAES (locked 2026-05-03)

With the IEEE TAES document class, `\ref{}` to a subsection or sub-subsection returns only the local counter (e.g., "C", "2") instead of the fully-qualified "III-C" / "III-A2". Use **hardcoded composition** of `\ref{}` calls:

| Target | Composition pattern | Renders to |
|---|---|---|
| Section (top-level) | `Section~\ref{section_label}` | "III" |
| Subsection | `Section~\ref{section_label}-\ref{subsection_label}` | "III-A" |
| Sub-subsection | `Section~\ref{section_label}-\ref{subsection_label}\ref{subsubsection_label}` | "III-A2" |

Join between `section` and `subsection` uses a hyphen (`-`); join between `subsection` and `subsubsection` uses no separator. `IEEEtaes.cls` uses `\Alph{subsection}` and `\arabic{subsubsection}`; the `\thesubsection` redefinition trick was rejected by the user (2026-05-03) — explicit composition is the locked approach.

**How to apply:**
1. **Top-level sections need no composition.** `\ref{background: section}` already renders "II" (roman `\thesection`). Do NOT compose top-level refs (no `Section~II-` prefix needed).
2. **Subsection refs must compose.** Bare `Section~\ref{stability: section}` renders "Section C" (wrong). Use `Section~\ref{control strategy: section}-\ref{stability: section}` → "Section III-C".
3. **Sub-subsection refs must compose all three levels.** Bare `Section~\ref{fov cone: section}` renders "Section 3" (wrong). Use `Section~\ref{control strategy: section}-\ref{outer loop: section}\ref{fov cone: section}` → "Section III-A3".
4. **Prefer parent over sibling range.** When a contribution/sentence cites multiple sibling subsections (or sub-subsections), cite the **parent** section instead of composing a range. Para 7 (a) cites §III.A.2 + §III.A.3 → use "Section III-A", not "III-A2--3". Para 7 (d) cites §IV.B + §IV.C + §IV.D → use "Section IV", not "Sections IV-B--D". The parent subsumes the cited siblings when the range covers most of the parent's children (with at most one excluded sibling that is *foundational* to the others).
5. **Match singular/plural.** When collapsing a sibling range to the parent, change `Sections~\ref{...}` → `Section~\ref{...}` (plural → singular). Forgetting this leaves stray "Sections IV".
6. **Supplement uses the same pattern** for supplement-internal labels (`sup:detailed`, `sec:sup-results`, etc.): `Section~\ref{sec:sup-results}-\ref{sec:sup-sweep}` → "Section S3-D". Do NOT compose supplement refs that already cross to the main paper — use the textual form (§3 below).
7. **Top-level supplement refs do not need composition.** Bare `Section~\ref{sup:baseline}` renders "Section S1" if `\thesection` is `S\arabic{section}` in the supplement preamble. Verify and only add hyphen composition for sub-levels.

**Detection patterns for prose-audit** (flag any of):
- `Section~\\ref\{(stability|fov cone|inner loop|normalized pid|ppc optic flow|outer loop|target image parameters|quadrotor dynamics|problem statement|multi init: subsec|test conditions: subsec|speed envelope: subsec|comparison: subsec): section\}` — bare subsection/sub-subsection ref without parent composition.
- `Sections~\\ref\{[^}]+\}--\\ref\{[^}]+\}` where both labels are siblings under a common parent — flag and recommend collapsing to parent.
- Trailing `Sections~\\ref\{<single ref>\}` — flag plural mismatch with singular ref.

## 3. Supplement → main-paper cross-refs use TEXT, not `\eqref{}` (locked 2026-04-24)

The supplement compiles separately, so any `\eqref{}` / `\ref{}` to a main-paper label (in `manuscript.tex` / `control_formulation.tex` / `results.tex`) resolves as `??`. Use text references instead.

**Convention (matches existing Theorem 1 proof in §S2-C):**
- Equation refs: *"the unconstrained dynamics equation of the main paper (Section~III-A2)"* or *"the control law of the main paper"*.
- Theorem/Corollary refs: *"Theorem~1 of the main paper"*, *"Corollary~1"* (Theorem/Corollary numbering is the same in both docs).
- Assumption/Property: *"Assumption~1 of the main paper"*, *"Property~1 of the main paper"* — explicit "of the main paper" disambiguates from supplement-internal numbering.
- Section refs: *"Section~III-A3 of the main paper"*, *"Section~IV-B"* — IEEE-TAES format `III-A3` (Roman-letter-digit, no dot before digit). NOT `Section~III.A.3` or `Section~III-A.3`.

**Internal supplement labels** (`\label{sup:...}`) work normally with `\eqref{sup:...}` / `\ref{sup:...}` — only main-paper-label cross-refs need the text treatment.

**Why:** User flagged 2026-04-24 in the Theorem 2 proof — `\eqref{yaw control law: equation}`, `\eqref{psi d integrator: equation}`, and `\eqref{yaw UUB bound: equation}` were all main-paper labels and would have rendered `??` in the compiled supplement.

**How to apply:** when writing new supplement prose referencing a main-paper symbol/equation, immediately rephrase as a text reference rather than reaching for `\eqref{}`. When auditing the supplement, grep for `\eqref{` and `\ref{` and verify each label exists in `supplemental.tex` itself; flag main-paper labels for rewriting. The Lemma at `\label{sup:theorem1_proof}` is supplement-internal — `\ref{sup:theorem1_proof}` is fine when invoked from elsewhere in the supplement.

## Related conventions
- `feedback_audit_anchoring_after_relocation.md` — when prose is relocated, audit eqrefs for direction (forward vs backward) — moved-up text gains forward-ref risk.
- `reference_notation_audit.md` — symbol-coverage audit also touches cross-refs.
