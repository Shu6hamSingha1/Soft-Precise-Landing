---
name: lit-review-categorize-by-problem-demand
description: "Locked 2026-05-26. Categorize related work for the VDF-ASMC manuscript by what the landing problem demands (information/objective regime), not by the adjacent technique. Asymmetric naming is allowed when it reflects truth; don't force parallelism."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

When organizing related work for the soft-precise-landing manuscript, structure the categories along **what the landing problem demands of the controller**, not along the technique used to solve it.

**Why:** Two corrections in the 2026-05-26 categorization exchange:
1. Splitting top-level into "Optic Flow" and "Visual Servoing" conflates problem with technique — OF *is* a form of VS, so they cannot be siblings.
2. Naming Part 1 "Target-Free / Unbounded / Unlocalized Target" forced surface parallelism with Parts 2/3 ("Static Target", "Moving Target") and hid the real distinction. The actual axis is the *landing requirement*: soft-only vs soft-precise-on-static vs soft-precise-on-moving.

A problem-driven paper's §I should escalate along the *requirement* axis, not the technique axis. Technique categories (IBVS, image moments, optic flow, virtual camera) become *sub-buckets within a technique-layer section*, not parallel to landing-problem categories.

**How to apply:**
1. **State the axis first.** What does each category demand of the controller? Here: target information — none / position / position+velocity.
2. **Name each category by what it IS along that axis**, not by surface parallelism. Asymmetric names that reflect truth beat forced parallelism that distorts it.
3. **Anchor on the paper's own vocabulary.** "Soft-precise" is the user's term — use it as the organizing word.
4. **Apply the same scrutiny to technique sub-categories.** Don't list "IBVS" and "Optic Flow" as siblings when one subsumes the other.
5. **The §I storyline mirrors the escalation:** prior soft-only work → prior static-target soft-precise work → our moving-target soft-precise contribution.

**Locked taxonomy for §I (2026-05-26):**

| Category | Landing requirement | Example prior work |
|---|---|---|
| **Soft Landing** | softness only (target = infinite surface) | herisse2012 (vertical), izzo2011, singhal2025, baird2013, ho2018, deCroon2016, van Breugel 2014 |
| **Static-Target Soft-Precise Landing** | softness + position | chitrakaran2005, brockers2011, benini2018, zhang2015, shakernia1999, salehi2021 (static fiducial) |
| **Moving-Target Soft-Precise Landing** | softness + position + velocity | lin2022, zhang2026, cho2022, paris2020, herisse2012 (moving), bouazza2025, zhao2021, bouaiss2022 |

VS methodology (IBVS / image moments / virtual camera / optic flow) is a **technique-layer** section, not a sibling of the three problem-layer categories.

## Related conventions
- [[feedback_citation_verification_discipline]] — verify per-cite claims against actual paper.
- [[feedback_citation_classification_audit]] — verify framework classification (PBVS / IBVS / OF) against each cite.
- [[project_naming_decisions]] — paper title and controller name epoch.
