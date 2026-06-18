---
name: no-mathematical-symbols-in-introduction-section
description: "Locked 2026-05-13. §I (INTRODUCTION) of the main paper must avoid all mathematical symbols ($\\boldsymbol{h}$, $\\,^\\mathcal{V}\\boldsymbol{v}_{\\text{t/b}}/\\,^\\mathcal{V}z_\\text{t}$, $\\mathbb{R}^3$, etc.). Notation and definitions belong in §II onwards. The intro must read as prose accessible to a non-specialist."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

## Rule

In §I (INTRODUCTION) of the main paper, **do not use mathematical symbols** in the prose. This includes:

- Vector / scalar symbols: $\boldsymbol{h}$, $\boldsymbol{p}_2$, $\sigma$, etc.
- Frame-prefixed quantities: $\,^\mathcal{V}\hat{\boldsymbol{r}}$, $\,^\mathcal{I}\boldsymbol{a}_\text{d}$, etc.
- Set notation: $\mathbb{R}^3$, $\in$, $\subseteq$, etc.
- Operator notation: $L_s^\dagger$, $\dot{\boldsymbol{p}}$, etc.
- Threshold values when embedded in math: $\le 0.08~\text{m}$ — borderline; numbers alone are OK.

Acceptable in §I: numerical values with units written out (e.g., *"25/25 landing rate"*, *"5.7 cm worst-case error"*, *"±40% speed envelope"*).

## Why

The introduction is the *entry point* for readers (including non-controls reviewers) and should establish the *problem* and *motivation* before notation. Mathematical symbols mid-prose interrupt readability and prematurely commit the reader to specific definitions that belong in §II.

The user clarified 2026-05-13 ("Also avoid using mathematical symbols in Introduction Section") when an §I draft used $\boldsymbol{h}$, $\boldsymbol{v}_{\text{t/b}}/z_\text{t}$, and $\mathbb{R}^3$ to introduce the *virtual image velocity*.

## How to apply

- §I prose: spell out concepts in words. E.g., *"depth-normalized image-feature time-derivative"* OR *"optic flow"* (as a common-noun term), but not the symbol.
- Defer symbol-bearing definitions to §II-B (Image Parameters) or wherever the symbol is first formally needed.
- Cite literature using the literature's terminology in §I (e.g., *"optic flow"* per Hérissé/Izzo/Singhal). The §II-B definitions can introduce a paper-specific renaming.

## Related conventions

- `feedback_short_sentences_no_colons_no_emdash.md` — §I prose style.
- `feedback_validate_critiques_against_cited_works.md` — verify claims about cited works.
- `feedback_fiducial_marker_scope.md` — what to say (and not) about scope.
