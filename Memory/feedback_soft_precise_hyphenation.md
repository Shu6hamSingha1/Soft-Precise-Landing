---
name: Hyphenate "soft-precise" paper-wide (compound adjective)
description: Locked 2026-05-01. The phrase is always written "soft-precise" (with hyphen) when it modifies a noun (landing, touchdown, envelope, region, count, criterion, etc.); appears in title, abstract, body, captions, and supplement
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

When "soft" and "precise" jointly qualify a noun, use the hyphenated compound adjective **"soft-precise"** (one word, with hyphen). This applies paper-wide:

- soft-precise landing
- soft-precise touchdown
- soft-precise envelope
- soft-precise allowable landing region
- soft-precise count
- soft-precise corridor
- soft-precise threshold

**Why:** It is a single compound adjective denoting touchdowns that simultaneously satisfy both criteria (precise position + bounded relative velocity). Without the hyphen, "soft precise landing" parses ambiguously as either "soft, precise landing" (two coordinate adjectives) or a noun-noun cascade. The hyphenated form locks the meaning as a single property.

**How to apply:**

- Title (`manuscript.tex` L36) carries "Guaranteed Soft-Precise Landing".
- Running head (`\markboth` L64) carries "SOFT-PRECISE LANDING".
- Abstract Sentence 2 (`manuscript.tex` L67): "does not guarantee the soft-precise touchdown".
- Contribution (d) (`manuscript.tex` ~L114): "achieving the functional requirements of soft-precise touchdown...".
- Para 7 (`manuscript.tex` ~L154): "achieves the functional requirements of both the soft-precise touchdown and target visibility...".
- §IV.A intro (`results.tex` L4): "termed *soft-precise* when both criteria hold simultaneously".
- All figure captions for landing-corridor / multi-init / multi-speed plots: "soft-precise allowable landing region", "soft-precise touchdowns are triangles".
- Supplement title (`supplemental.tex` L14) and §S3-G captions: "soft-precise" hyphenated.

**Exceptions / not hyphenated:**
- When listing the two criteria *separately* and emphasising the conjunction: "simultaneously *precise* (...) and *soft* (...)" — italicised, separate words, no hyphen, because they're labelling two distinct properties.
- The bare adjectives "soft" and "precise" used standalone in their italic-emphasis form (e.g., "the touchdown is *precise* when ...") stay separate.

**Where this came up (2026-05-01):**

User instruction following the title rephrase: "Also use hyphen between soft and precise across the paper, including title". A global `sed` pass handled most occurrences, but mixed-case forms ("Soft precise") had to be caught manually because the regex was case-sensitive at first. Eight places were fixed paper-wide.

## Related conventions

- `project_naming_decisions.md` — title (2026-05-01 update) carries hyphenated "Soft-Precise".
- `feedback_soft_precise_definition_location.md` — *precise* and *soft* defined separately in §IV.A; the compound "soft-precise" is the conjunction labeller.
- `feedback_functional_requirement_framing.md` — "achieve the functional requirement of [property]" attribution pattern uses "soft-precise touchdown" as the named property.
