---
name: Supplement may be elaborative; main paper is not
description: Locked 2026-05-08. Supplement has no page limit so prioritize CLARITY over brevity. Per-section sentences with \ref{}s, full mechanism derivations, mechanism contrast tables, and explicit Theorem/Remark/Property references are all preferred over compact prose. Still avoid redundancy — don't repeat the same point twice across the supplement.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

**Main paper**: page-budget-tight. Compact prose, integrated lists, brief references.

**Supplement**: no page limit. Prioritize clarity:
- Prefer per-section explanations with `\ref{}`s over enumerated `(i)/(ii)/(iii)` lists embedded in single sentences.
- Spell out mechanism contrasts (e.g., the §S3-H mechanism contrast table mapping baseline failure → MDF-ASMC mechanism).
- Cite Theorem/Remark/Property/Corollary numbers explicitly when explaining figures or numerical observations.
- Add closing "why" paragraphs after analysis sections (e.g., §S3-A closing on dual-funnel architecture mechanism).

**But still avoid redundancy**:
- Don't restate the same Theorem/Remark/Property explanation in two different supplement sections.
- Don't duplicate numerical claims that are already in the main paper Tables.
- Don't repeat "soft-precise = $r_{xy,\text{f}}\le 0.08$ m, $v_\text{f}\le 0.20$ m/s" multiple times — define once at top of supplement, reference thereafter.

## Why

User's stated preference (2026-05-08): "There is no page limit for Supplement. So you can keep it as elaborative as possible. Main focus is that the explanation should be clear. But still avoid any redundancy."

This is opposite to the main paper's compaction discipline (page-budget enforced). Apply different writing style per document.

## How to apply

1. **When asked to review the supplement**: prefer clarity-enhancing edits even if they add words. Per-section breakdowns, explicit Theorem refs, mechanism contrast tables are all welcome.

2. **When asked to review the main paper**: keep applying the compaction discipline (short sentences, no redundancy, tight cross-refs).

3. **Cross-doc consistency**: notation, figure numbers, table numbers, claims must match across main paper and supplement (already enforced).

4. **Redundancy check**: at the end of each supplement edit, scan whether the new content duplicates an existing supplement section. If yes, either consolidate or cross-reference (`see Section S3-X`).

## Related conventions

- `feedback_short_sentences_no_colons_no_emdash.md` — applies paper-wide; in supplement still avoid em-dashes and narrative colons, but tolerate longer sentences for clarity.
- `project_2026-05-06_section_ii_iii_lock.md` — main paper layout snapshot.
- `feedback_validate_critiques_against_cited_works.md` — verify-before-claim discipline applies in both documents.
