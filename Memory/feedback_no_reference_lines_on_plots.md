---
name: No reference lines on comparison plots
description: User prefers comparison bar charts without budget/precise/soft reference lines
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Do not add reference/threshold lines (budget time, precise distance, soft velocity) to comparison summary bar charts.

**Why:** User explicitly asked to remove the 40s budget, 0.05m precise, and 0.20m/s soft reference lines from comparison_summary.pdf.

**How to apply:** When generating or updating comparison bar chart plots, omit axhline/text annotations for threshold values.
