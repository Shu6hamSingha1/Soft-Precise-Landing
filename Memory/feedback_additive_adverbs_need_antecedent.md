---
name: Bare additive adverbs ("also", "too", "additionally") need explicit antecedents
description: Locked 2026-05-06. The user flags "also" / "too" / "additionally" / "as well" as vagueness sources when the antecedent (what the additive is "in addition to") is left implicit. Either name the antecedent explicitly or drop the adverb. Bare additive adverbs force the reader to infer a comparison, and the inference is not always obvious.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** Every "also" / "too" / "additionally" / "as well" must have a clearly named antecedent in the immediately preceding context. If the antecedent is in another section, name it. If naming feels forced or contrived, drop the adverb entirely.

**Why.** Bare additive adverbs imply a comparison ("X is also used in Y" → "in addition to *what*?"). When the comparison referent is implicit, the reader has to scan back to find it; the word *adds vagueness rather than clarity*. The page budget in IEEE TAES does not tolerate words that ask the reader to do extra work.

**How to apply.**

1. **First pass.** When proofreading §II / §III prose, mark every "also" / "too" / "additionally" / "as well".
2. **For each occurrence.** Ask "in addition to what?" If the answer is not stated in the same paragraph or the prior 1–2 sentences, the antecedent is implicit.
3. **Two fixes.**
   - **Drop the adverb.** Often the sentence stands on its own without the additive sense. Example below.
   - **Name the antecedent.** "in addition to feeding X, …, the points feed Y" — verbose but explicit.
4. **Default to dropping.** If naming the antecedent makes the sentence noticeably longer, drop the adverb. The reader will see from §III.A.1 / §III.A.2 / §III.A.3 sub-block structure that multiple inputs feed multiple blocks; restating it adds noise.

## Origin (2026-05-06 incident)

**Sentence:** "To ensure the target remains visible during soft-precise landing, the image feature points $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ are also used directly in the cone clamp (Fig.~1)."

**My initial reading.** Took "also" as benign additive ("the points feed the cone clamp *in addition to* the optic-flow loop's $\boldsymbol{h}$ feed"). Proposed four replacement options that left "also" intact and tried to fix "directly" instead.

**User's correction.** "What I meant is that 'also' is adding vagueness." None of my four options addressed the actual vagueness source.

**Fix applied.** Drop "also". Final: "the image feature points $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ are used directly in the cone clamp".

**Lesson for proofreading.** The antecedent for "also" was the implicit "the optic-flow loop uses $\boldsymbol{h}$" from §III.A.2. That antecedent is plain to me as the writer (because I am tracking the data flow) but it is not plain to a reader hitting this sentence cold at the §III.A.3 lead. "Also" forced an implicit cross-block comparison without naming the prior block.

## Watchlist phrases

When reviewing §I / §II / §III for vague terms, scan for:

- "also"
- "too" (in additive sense; not the intensifier "too small")
- "additionally"
- "as well"
- "moreover" (when used as bare connector with implicit antecedent)
- "further" (when used as additive, not as comparative)
- "in addition" (without naming what it's added to)

These join the existing watchlist of bare/colloquial words flagged in earlier sessions ("practical", "the airframe ceiling", "physical saturation", "executed thrust/torque commands", "measured image features", "the body torque").

## Related conventions

- `feedback_no_vague_this.md` — every "this" must be followed by an explicit noun. Same family of rule (bare deictic / additive needs explicit antecedent).
- `feedback_audit_anchoring_after_relocation.md` — when prose is moved, antecedents can break. Bare "also" / bare "this" are the highest-risk markers to audit.
- `feedback_goal_first_when_mechanism_follows.md` — opening with the goal (visibility) and then the mechanism (cone clamp) reduces but does not eliminate the "in addition to what" problem.
