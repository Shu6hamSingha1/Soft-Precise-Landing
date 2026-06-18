---
name: Short sentences; no colons; no em-dashes paper-wide
description: Locked 2026-05-01. Avoid (a) long compound/complex sentences, (b) colons used to introduce explanatory clauses, and (c) em-dashes (`originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
`) in main paper, supplement, captions, and abstract; en-dashes for compound nouns (UAV--target) are still fine
type: feedback
---
## Rules

1. **Short sentences.** Break compound sentences with two or more major clauses into separate sentences. A single sentence should generally carry one main idea. Lists of three or more parallel items separated by commas are acceptable inside a single sentence; what to avoid is multi-clause sentences chained by commas, semicolons, or em-dashes.

2. **No colons in narrative prose.** Do not use a colon (`:`) to introduce an explanatory clause, a definition, or a list. Either start a new sentence, or restructure so the explanation flows inline.
   - WRONG: "We propose MDF-ASMC: an optic-flow funnel generates..."
   - RIGHT: "We propose MDF-ASMC. An optic-flow funnel generates..."
   - Acceptable colons: theorem-environment headers (`\textit{Theorem 1.}`, `\textit{Proof.}`), table captions when introducing a list of column meanings, and the colons inside math (e.g., set-builder).

3. **No em-dashes (`---`).** Replace em-dashes with one of:
   - A period (start a new sentence) — best for two complete thoughts.
   - Parentheses — best for inline parentheticals like "(a *precise* touchdown)".
   - Commas — best for short non-restrictive parentheticals.
   - **En-dashes (`--`)** for compound nouns and ranges (`UAV--target`, `2--5`) are FINE; only the triple-dash em-dash is banned.

4. **No semicolons used as soft sentence-joiners.** A semicolon connecting two independent clauses should usually become a period. Semicolons inside a multi-item enumeration (where commas already separate sub-items) remain acceptable.

## Why

The user's verbatim feedback (2026-05-01):
> "I don't like long sentences, use of colons and long hyphens. Make suitable changes."

This follows an earlier related rule (2026-05-01):
> "Don't use colon like this and stop formulating long sentences. It seems like AI-generated."

Long compound sentences, colon-led explanatory clauses, and em-dash interruptions all carry an "AI-generated essay" register that the user wants out of the IEEE TAES manuscript. Short declarative sentences read more like a technical paper and survive copy-edit better.

## How to apply

- **When writing new tex prose for any active paper file** (`manuscript.tex`, `control_formulation.tex`, `results.tex`, `supplemental.tex`, `frames_planes.tex`), default to short sentences with periods and parentheses; never reach for `:` or `---`.
- **When editing existing prose**, treat any `:` (outside a list-introducing colon at the very end of a clause, like "as follows:") and any `---` as a flag for restructuring.
- **Preserve content**. Punctuation cleanup must not change technical claims, references, or locked terminology. If a sentence carries multiple locked phrases, restructure but keep every locked phrase intact.
- **Captions** follow the same rules. Long figure/table captions chained by `;` or `---` are common; convert them to separate sentences.

## Acceptable exceptions

- The colon after the subsection heading style `\textit{Theorem 1 (...).}` is part of the theorem environment and stays.
- Bullet/enumerate introducers ("The main contributions of this paper are summarised as follows:") use a colon at the end of the lead-in clause; this is standard and acceptable.
- En-dashes (`--`) in compounds: `UAV--target`, `2--5`, `xy_e`-style range bounds. Keep these.
- Inline math with set-builder colons or function-arrow notation. Keep these.
- **Lead clause + comma-separated enumeration of parallel components.** A sentence of the form "X does P by doing A, doing B, and doing C" stays as one sentence even when long, provided A/B/C are syntactically parallel and the lead clause is doing the work of grouping them. Example kept as one sentence by the user (re-ordered 2026-05-02 to follow listing order):
  - §I summary (Para 7, locked 2026-05-02): *"Contribution~(a) closes gaps~(i)--(ii) by fusing..., **(b)** removes..., **(c)** closes gap~(iii) through..., and **(d)** characterises..."* — four parallel contributions enumerated under one summary statement; ordered (a)→(b)→(c)→(d) to match the contributions-list ordering above it. Earlier draft (a, c, b, d) was reordered to (a, b, c, d) for visual parallelism with the enumerate.
  - The trigger to split is two unrelated clauses chained by `:`, `;`, or `---`, NOT a lead clause introducing a parallel enumeration.

## Trigger to split (apply the rule) vs keep (use the exception)

- **Split** (apply the rule): two distinct ideas joined by `:`, `;`, or `---`. Example: "MDF-ASMC achieves X: A does Y, B does Z, C does W." → split the colon-prefixed elaboration into separate sentences.
- **Keep** (use the exception): one lead clause that grammatically groups several parallel sub-clauses under a single subject. Example: "Contribution~(a) closes gap...,~(c) closes gap..., (b) removes..., and (d) characterises..." → stays one sentence.

## Where this came up

User feedback after I wrote a colon-led explanatory clause in §I Para 2. Now extended paper-wide.

## Related conventions

- `feedback_no_legacy_comparisons.md` — write tex from scratch, no chained comparisons.
- `feedback_no_vague_this.md` — explicit nouns after every "this".
- `feedback_caption_equation_break.md` — long captions: close/reopen `$...$` for natural breaks; do not use `\allowbreak` or em-dashes.
