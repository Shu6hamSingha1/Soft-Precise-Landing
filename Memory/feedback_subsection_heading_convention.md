---
name: Section heading convention originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
subsubsection, not inline italics
description: Use `\subsubsection{}` (not `\textit{Header.}` inline) for content division within a `\subsection{}`. `\textit{...}` is reserved for theorem-environment block labels (Theorem N / Assumption N / Property N / Corollary N / Proof of Theorem~N / Remark). Locked 2026-04-30.
type: feedback
---

**Rule.** Inside an IEEE TAES paper, divide a `\subsection{}` into content blocks using `\subsubsection{}`. Do **not** use inline `\textit{Heading.}` or `\emph{(i) Heading.}` style as a section divider --- it's not the convention for a reputable journal and creates non-uniform headings across the paper.

**Where `\textit{...}` IS appropriate (theorem-environment blocks):**
- `\textit{Theorem 1 (Title).}` --- theorem block header
- `\textit{Assumption 1} (description):` --- assumption block header
- `\textit{Property 1} (description):` --- structural property block header
- `\textit{Corollary 1 (Title).}` --- corollary block header
- `\textit{Remark.}` or `\textit{Remark 1 (Title).}` --- remark block header
- `\textit{Proof of Theorem~N.}` (per `feedback_proof_placement_convention.md`) --- proof block header
- `\textit{Lemma N (Title).}` --- lemma block header

**Where `\textit{...}` is NOT appropriate (use `\subsubsection{}` instead):**
- `\textit{Quadrotor dynamics.}` → `\subsubsection{Quadrotor Dynamics}`
- `\textit{Target image parameters.}` → `\subsubsection{Target Image Parameters}` *(or move to a dedicated `\subsection{}` if it has multiple sub-blocks)*
- `\textit{Sources of model uncertainty and exogenous perturbation.}` → just use plain prose lead-in (no heading) if it's short, or `\subsubsection{}` if it warrants division
- `\textit{Control problem.}` → just use plain prose lead-in ("The control problem is to ...") or `\subsubsection{Control Problem}`
- `\emph{(i) Foo.}`, `\emph{(ii) Bar.}` style enumerated section dividers → `\subsubsection{Foo}`, `\subsubsection{Bar}`

**Pattern from §III of the manuscript (canonical):**
```
\subsection{Outer Loop: Dual Funnel and Adaptive SMC}
\subsubsection{Normalized Virtual Image Feature Control Law} ...
\subsubsection{Optic Flow Control Law} ...
\subsubsection{Target Image Funnel and Acceleration Conditioning} ...
```
Each `\subsubsection{}` contains substantive derivations (multiple paragraphs + equations). If a candidate "subsubsection" would be just one paragraph, prefer flowing prose under the parent `\subsection{}` instead.

**§II structure as locked 2026-04-30:**
```
\section{PRELIMINARIES \& PROBLEM FORMULATION}
\subsection{Quadrotor Dynamics}
  -- Newton-Euler equation + 1 explanation paragraph
\subsection{Target Image Parameters}
  -- bridge paragraph
  \subsubsection{Normalised Target Position}
  \subsubsection{Target Virtual Orientation}
  \subsubsection{Optic Flow}
\subsection{Problem Statement}
  -- enumerated uncertainty/perturbation list (no inner subsubsection)
  -- bridge sentence
  \textit{Assumption 1} (uncertainty bounds): ...
  \textit{Property 1} (cone-clamped lateral acceleration): ...
  -- Control problem (plain prose lead-in)
```

**Where this came up (2026-04-30):**
A draft of §II.A used `\textit{Quadrotor dynamics.}` and `\textit{Target image parameters.}` to divide content. The user asked: "Does this technical writing go for a technical paper for a reputed journal? Also, this type of technical writing is not uniform across the entire paper. In other sections, I have used \subsubsection{} to further divide the subsection." The user then chose Option B (split into three subsections: Quadrotor Dynamics / Target Image Parameters / Problem Statement, with subsubsections used inside the second).

**Related conventions:**
- `feedback_proof_placement_convention.md` --- `\textit{Proof of Theorem~N.}` is the canonical proof-block header.
