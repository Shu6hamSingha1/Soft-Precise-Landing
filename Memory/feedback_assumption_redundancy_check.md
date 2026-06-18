---
name: Check Assumption coverage before adding redundant clauses to Problem/Theorem statements
description: When formulating a Problem/Theorem/Corollary that begins with "Under Assumption~$N$, …", do not repeat clauses that Assumption $N$ itself encodes (e.g., "without a priori knowledge of the bounds"). The "Under Assumption~$N$" prefix carries forward what's inside it. Read the Assumption first; if your draft clause is already covered, drop it. Locked 2026-05-05.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** Before adding a qualifying clause to a statement that begins with "Under Assumption~$N$, …", read what Assumption $N$ actually says. If the clause is already encoded inside the Assumption (e.g., "the bounds are unknown / existence-only / need not be known to the controller"), drop the clause. The "Under Assumption~$N$" prefix already carries forward the Assumption's content; restating it inflates the statement without adding information.

**Why.** Repeating Assumption content in a dependent statement (i) inflates word count, (ii) creates two places where the same claim must stay synchronized if the Assumption is later revised, and (iii) suggests to the reader that the dependent statement asserts something *additional* when it does not. The reader then has to mentally compare the two phrasings to be sure they're the same.

## Example (where this rule was applied, 2026-05-05)

§II.C Problem statement was drafted as:
> "Under Assumption~1, formulate suitable control laws … without any *a priori* knowledge of the bounds of model uncertainty and exogenous perturbation."

Assumption 1 itself reads:
> "The aggregate bounds … used in the proofs … are existence-only. The finite values follow from componentwise boundedness above and *need not be known to the controller*."

The "without any *a priori* knowledge" clause restated what Assumption 1 already encodes. The user flagged it; the clause was removed:
> "Under Assumption~1, formulate suitable control laws … and keep the target visible throughout the descent. The resulting closed-loop response achieves the functional requirement of soft-precise landing."

## How to apply

1. **When drafting** "Under Assumption~$N$, …" statements, pull up Assumption $N$'s text first and re-read it.
2. **For each qualifying clause** in your draft (especially "without …", "subject to …", "given that …"), ask: is this already inside Assumption $N$?
3. **If yes**, drop the clause. The "Under Assumption~$N$" prefix is enough.
4. **If no**, keep — but then the clause should be a genuinely *new* qualifier the Assumption doesn't cover.
5. **Apply the same check** to Theorems, Corollaries, Properties, and any other statement of the form "Under [referred condition], …".

## Watchlist of "redundant" candidate clauses

When the dependent statement is "Under Assumption~$N$, …" and Assumption $N$ deals with disturbance bounds:

- "without *a priori* knowledge of the bounds" → likely redundant
- "for unknown but bounded disturbances" → likely redundant
- "subject to the disturbance class of Assumption~$N$" → tautological
- "where $\beta, \boldsymbol{d}_h$ are bounded" → already in the Assumption

## Related conventions

- `feedback_no_overclaim_proven_properties.md` — same spirit on the *other* side: don't claim *more* than what's actually proven.
- `feedback_short_sentences_no_colons_no_emdash.md` — short sentences pair with non-redundant clauses.
- `feedback_verify_before_claim.md` — process discipline: verify the underlying source before making a derived claim.
- `feedback_audit_anchoring_after_relocation.md` — when relocating, audit for orphaned phrases; complementary to this rule (here we audit for *redundant* phrases).
