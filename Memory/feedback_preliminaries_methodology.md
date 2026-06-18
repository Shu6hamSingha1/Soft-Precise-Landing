---
name: Preliminaries describes methodology, not the simulation choice
description: §II Preliminaries must describe the methodology generally (any number $N\ge3$ of feature points, anywhere on the target). The "four corner points" choice is a simulation/technique decision, introduced only in §IV (results) and §S1 implementation details. Locked 2026-04-30.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** In §II PRELIMINARIES \& PROBLEM FORMULATION (and any other "methodology" section), describe the controller's image-feature pipeline using **$N$** feature points $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$, $i\in\{1,\dots,N\}$, with $N\ge 3$. Do NOT fix $N=4$ or call them "corner points" in Preliminaries.

**Why:** The MDF-ASMC methodology generalises to any visual feature on the target (not necessarily corner points) and any number of points (not necessarily four). The "four corner points of an AR tag / planar marker" choice is a *simulation/technique* decision specific to the experimental setup --- introducing it in Preliminaries would falsely narrow the methodology's scope and would mislead reviewers about what the controller actually requires.

**Where to fix $N=4$ and "corner points":**
- §IV (Results) --- when describing the simulation setup ("In the simulations, we use $N=4$ corner points of a planar AR-tag target...").
- §S1 of the supplement --- implementation details for the pinhole projection on the four corners.
- Figure 1 caption --- may say "$N=4$ in the figure for illustration" so the visual layout is concrete without locking the methodology.

**Where to keep general:**
- §II.A introduction and bridge.
- §II.A item (i) "Normalised target position" derivation.
- §II.A item (ii) "Target virtual orientation" formula (centroid mean and second-order moments use $\sum_{k=1}^N$).
- §II.A item (iii) "Optic flow" --- the optic-flow formulation is point-agnostic.
- §III control-law derivations.
- Theorem statements and proofs.

**Banned in §II prose:**
- "The four target image feature points..." → "The $N$ target image feature points..."
- "$i\in\{1,\dots,4\}$" → "$i\in\{1,\dots,N\}$"
- "the second-order centred moments of the four virtual feature points" → "...of the $N$ virtual feature points"
- "corner-like points" / "corner points" / "the four corners" → "target image feature points" / "feature points"
- Any sentence in §II that depends numerically on $N=4$.

**Where this came up (2026-04-30):**
A draft of §II.A had "the four target image feature points $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$, $i\in\{1,\dots,4\}$" in two places. The user pushed back: "we have used four corner points ... but it can be generalized to any point and any number of points. However, in Preliminaries, we focus on methodology not our technique or approach." All occurrences of "four" in §II.A were replaced with "$N$" (with $N\ge 3$ stated once for definiteness); the figure caption notes "$N=4$ in the figure for illustration" so the rendering is concrete without binding the methodology.

**Related conventions:**
- Three target image parameters: `feedback_target_image_parameters.md`.
- No "landmark" term: `feedback_no_landmark_term.md`.
- Corner-point naming (related): `feedback_corner_points_naming.md`.
