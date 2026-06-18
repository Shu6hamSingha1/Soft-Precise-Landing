---
name: Goal-first when the mechanism follows in the next sentence
description: When a sentence introduces a block / mechanism / step and the next sentence already elaborates the mechanism in detail, lead with the GOAL (the high-level purpose) instead of the mechanism. Goal-first is more reader-friendly. Locked 2026-05-05.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** When a sentence introduces a block, mechanism, or step whose details are explained in the *next* sentence (or paragraph), lead with the **goal** — what the block achieves — rather than the mechanism — how it does it. The mechanism is already coming; saying it twice is redundant. Goal-first lets the reader form an intuition before being immersed in details.

**Why.** Goal-first is more reader-friendly because the next sentence already explains the mechanism in detail. Mechanism-first creates a "two-pass" read where the reader skims the first sentence, hits the same mechanism in the second, and has to re-anchor. Goal-first sets the destination, then the next sentence shows the route.

## Example (where this rule was applied, 2026-05-05)

§III.A.3 (Acceleration Conditioning) opening sentence about why the image feature points are reused at the cone-clamp block:

**Mechanism-first (rejected):**
> "...the image feature points $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ enter this block directly to compute the funnel margin that sizes the cone half-angle."

**Goal-first (locked):**
> "To ensure the target remains visible during the soft-precise landing, the image feature points $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ are also used directly in this block."

The next sentence in §III.A.3 already says "The target image funnel and its funnel-margin cone clamp realize the state-dependent attitude cone of Property~1 by projecting the commanded lateral acceleration onto a cone whose half-angle tracks the instantaneous funnel margin..." — which explains the funnel-margin → cone-half-angle mechanism in full. Mechanism-first leads with that same content prematurely; goal-first ("ensure the target remains visible") gives the reader the *purpose* before the detail.

## How to apply

- Before drafting an opening sentence for a block / subsection / step, check if the very next sentence (or two) already explains the mechanism. If yes, lead the opener with the goal.
- If the mechanism is *not* explained later, then mechanism-first is fine — the reader needs the detail to follow what comes next.
- The goal should be expressed in *physical / operational* language ("ensure the target remains visible", "achieve soft-precise touchdown", "regulate optic flow") not mathematical mechanism language ("compute funnel margin", "drive sliding surface to UUB", "project onto attitude cone").

## Related conventions

- `feedback_visibility_phrasing.md` — "target remains visible" in prose (matches goal-first goal language).
- `feedback_short_sentences_no_colons_no_emdash.md` — short sentences pair well with goal-first (one sentence = one purpose, next sentence = mechanism).
- `feedback_no_overclaim_proven_properties.md` — goal-first claims must be honestly framed (don't over-promise the goal).
