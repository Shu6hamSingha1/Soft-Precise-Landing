---
name: Funnels are bounds, constraints are properties — "enforce the visibility constraint", not "enforce the target image funnel"
description: Reader-facing prose describing what the cone clamp / projection ENFORCES must name the physical constraint (visibility, actuator saturation), never the PPC bound (target image funnel); funnels are bounds that *hold* or are *invariant*, not things that are *enforced*
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

A mechanical rename from "visibility funnel" → "target image funnel" can introduce awkward / technically wrong phrasing if the original sentence said "enforces the visibility funnel". Do NOT blind-rename in such cases. The fix is to restate the **physical constraint** that the projection enforces, not rename the funnel.

**Correct pairings (reader-facing prose):**
- The cone clamp / attitude-cone projection **enforces** the *visibility constraint* and the *actuator-saturation constraint*.
- The target image funnel is **invariant** / **holds** / **is not violated** / **keeps $\boldsymbol{p}_1(t)$ as a valid bound**.
- Theorem 1 **proves invariance** of the optic-flow funnel $\boldsymbol{p}_2(t)$ (a mathematical property of the bound itself).
- Corollary 1 **delivers** closed-loop target visibility (the physical effect).

**Forbidden mechanical rewrites:**
- "enforces the visibility funnel" → NOT "enforces the target image funnel" → YES "enforces the visibility constraint" / "enforces visibility".
- "enforces the funnel" → NOT "enforces the target image funnel" / "enforces the optic-flow funnel" → YES name the property being enforced.
- "satisfies the target image funnel" → YES "keeps every feature point inside $\boldsymbol{p}_1(t)$" or "satisfies the visibility constraint".

**Why:**
- A PPC funnel is a time-varying bound; constraints are relations between the closed-loop state and physical limits (FoV edges, actuator ceilings). You *enforce* a constraint, you don't *enforce* a bound — bounds are *satisfied*, *invariant*, or *held*.
- The manuscript has already separated the funnel (bound) from the cone clamp (enforcement mechanism) at the subsection-title level ("Target Image Funnel and Funnel-Margin Cone Clamp"). Prose-level phrasing must match that separation.
- Consistent with `feedback_visibility_phrasing.md`: reader-facing prose uses physical language (visibility), math-bound language (the funnel) belongs in equation set-definitions and proof proper.

**How to apply:**
- Before renaming any "visibility funnel" → "target image funnel", read the full sentence. If the verb is *enforce*, *guarantee*, *ensure*, or *satisfy* and the subject is the projection/clamp/controller, rename to "visibility constraint" (or leave as the physical property).
- If the verb is *define*, *shape*, *size*, *collapse*, *shrink*, *bound*, *serve as PPC envelope*, the subject is a mathematical bound and "target image funnel" is the correct replacement.
- Manuscript contribution (c) example (locked 2026-04-22): "A single attitude-cone projection simultaneously enforces the **visibility and actuator-saturation constraints** in closed form" — NOT "enforces the target image funnel".
