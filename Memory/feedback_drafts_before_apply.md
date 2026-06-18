---
name: Show drafts before applying for non-trivial prose rewrites
description: When the user asks for a non-trivial prose rewrite (e.g., "explain behavior of each axis", "rewrite this paragraph"), present the draft text first and let the user confirm before editing the file. Quick mechanical fixes (typo, single-word substitutions, semicolon→period) can still be applied directly.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

For substantive prose rewrites — especially multi-sentence body paragraphs, per-axis explanations, theorem-proof additions — **draft the replacement text in the chat first** and wait for confirmation before applying with `Edit`.

## Why

The user said 2026-05-10 ("first give me draft") after I started applying a 4-sentence per-axis rewrite of the sup-sliding body without showing the draft first. The rewrite was substantive enough that the user wanted to review the wording (and verify the actual numerical claims) before committing to the file.

## How to apply

**Show drafts first when:**
- Replacing one paragraph with a different paragraph (different content, not just light edits).
- Adding multi-sentence justification or explanation to a proof / caption / remark.
- Per-axis or per-item breakdowns (`(i)/(ii)/(iii)` style).
- Restructuring an argument or rephrasing a theorem statement.
- Anything where the user might want to check numerical values, mechanism claims, or wording before commit.

**Apply directly when:**
- Single-word or single-phrase substitutions (e.g., AI-jargon swaps, terminology fixes).
- Semicolon → period, or other mechanical punctuation fixes.
- Reverting to a previous form the user explicitly named.
- The user has already explicitly approved the exact text in chat (e.g., "Apply Option B" after I showed Option B verbatim).

## Related conventions

- `feedback_short_sentences_no_colons_no_emdash.md` — paper-wide style.
- `feedback_validate_critiques_against_cited_works.md` — verify before claiming.
