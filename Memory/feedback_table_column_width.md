---
name: Tables must fit single-column width
description: User prefers single-column tables and shorter labels to avoid tables overflowing column width in IEEE TAES format
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Keep tables within single-column width. Use short labels (e.g., "Proposed" instead of "DF-ASMC (Proposed)") and prefer `table` over `table*` where possible.

**Why:** User manually changed Table I from `table*` to `table` and shortened "DF-ASMC (Proposed)" to "Proposed" in the comparison table to prevent overflow in the IEEE TAES single-column layout.

**How to apply:** When creating or editing tables in the manuscript, check that row content fits single-column width (~3.5 in). Prefer abbreviated labels and `table` environment unless the table genuinely needs full page width.
