---
name: Figure title case convention — Title Case for plot titles, suptitles, and subtitles
description: Locked 2026-05-07. All matplotlib figure titles (plot titles, suptitles, and shared inter-subplot subtitles) use Title Case — capitalize the first letter of each major word, but keep articles/prepositions ("to", "for", "and", "of", "across", "the", "a", "an", "in", "on", "at", "by", "with") lowercase unless they are the first word.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

**Capitalize**: nouns, verbs, adjectives, adverbs, the FIRST word always, and the LAST word always.

**Lowercase** (when not first/last): "to", "for", "and", "of", "across", "the", "a", "an", "in", "on", "at", "by", "with", "or", "but", "vs".

## Examples (correct ✓)

| Element | Text |
|---|---|
| Suptitle | `Dual-Funnel Invariance` |
| Suptitle | `Landing Performance of Five Controllers across Cases 1--5` ("of" and "across" lowercase) |
| Subtitle | `Target Image Funnel` |
| Subtitle | `Optic Flow Funnel` |
| Subtitle | `Landing Performance for Case 5` ("for" lowercase) |
| Subtitle | `Landing Time` |
| Subtitle | `Horizontal Precision` |
| Subtitle | `Touchdown Softness` |

## Examples (wrong ✗)

| Wrong | Why |
|---|---|
| `Dual-funnel invariance` | "Funnel" and "Invariance" should be capitalized |
| `optic flow funnel` | First-word capital missing; major words missing capitalization |
| `Landing Performance Of Five Controllers` | "Of" should be lowercase |

## Hyphenated compounds

In hyphenated words like "Dual-Funnel" or "Soft-Precise", capitalize **both** parts. So "Dual-Funnel Invariance" not "Dual-funnel invariance".

## How to apply

When writing or reviewing matplotlib code:
- `ax.set_title("...")` — Title Case.
- `fig.suptitle("...")` — Title Case.
- `fig.text(..., "...")` for shared subtitles — Title Case.
- Axis labels and tick labels — sentence case is fine (these are not titles).
- Legend entries — usually pure mathematical symbols; sentence case for any descriptive text.

## Confirmed scripts following this convention (2026-05-07)

- `scripts/make_comparison_plots.py` — already in Title Case.
- `scripts/make_plasmc_plots.py` — updated 2026-05-07.

Other plot scripts (`make_multi_init_plots.py`, `make_multi_speed_plots.py`, `make_comparison_multi_speed_plots.py`) — review for consistency if titles exist.

## Related conventions

- `feedback_figure_label_sync.md` — figure scripts must stay in sync with paper notation.
- `feedback_short_sentences_no_colons_no_emdash.md` — paper-prose convention; titles are NOT prose, so this rule doesn't apply.
