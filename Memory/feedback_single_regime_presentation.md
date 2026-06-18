---
name: Single-regime "disturbance model" results — no idealised-vs-realistic contrast
description: Paper presents results under ONE disturbance model (Table II of the main paper); do not re-introduce an idealised / noiseless / disturbance-free regime or contrast language ("realistic", "full-disturbance", "full robustness model"). Decision locked 2026-04-24 with idealised §S3-A deleted.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The rule (locked 2026-04-24):**

Paper results are presented under a single disturbance model (specified in Table II of the main paper). There is no "idealised" or "disturbance-free" comparison regime, and therefore no contrast language distinguishing one regime from another.

**Banned phrases in active tex** (grep these before committing):
- "idealised", "idealized", "idealised conditions", "idealised regime"
- "realistic", "under realistic disturbances", "realistic-disturbance"
- "noiseless", "disturbance-free"
- "full-disturbance results", "full disturbance set", "full disturbance model"
- "under the full robustness model"

**Allowed framings:**
- "under the disturbance model"
- "under the simulated disturbance model (Table II)"
- Nothing at all (single-regime framing means no qualifier is needed)

**Why:** User flagged 2026-04-24 that having both regimes in the paper created
- surface-level uniformity issues ("don't project these results as realistic");
- ~92 lines of redundant supplement content (§S3-A was 10 figures + per-IC paragraphs mirroring §S3-B);
- a false dichotomy — the actual paper claim is the controller works under the disturbance model, not that it works better without disturbances.

**Structural decisions made 2026-04-24:**
- Supplement §S3-A "Multi-Initial-Condition Results under Idealised Conditions" deleted entirely.
- Surviving subsection renamed: "Multi-Initial-Condition Results under Realistic Disturbances" → "Multi-Initial-Condition Results".
- All §S3-X cross-references bumped up by one letter (S3-B→A, S3-C→B, ..., S3-H→G).
- Table IV caption: "under realistic disturbances (5 ICs per trajectory)" → "under the simulated disturbance model (Table~II); 5 ICs per case" with column header "Trajectory"→"Case" and row labels dropping the repeated "Case" prefix.

**Data on disk (post-cleanup 2026-04-26):**
- `MATLAB/Multi_init_cond/Datasets/*_multi_init_noiseless.mat` — 5 noiseless `.mat` files still exist as data artifacts; not cited by any tex.
- `Soft_Precise_Landing/Figures/generated/multi_init/*_noiseless.pdf` — moved to `Obsolete/Figures/generated/multi_init/` 2026-04-26.
- Plot scripts (`make_multi_init_plots.py`, `make_multi_speed_plots.py`) had their noiseless code path stripped 2026-04-26; backups at `Obsolete/scripts/<name>_v2.py`. The "(realistic)" / "(noiseless)" cond-label suffix on titles is gone — titles are now bare `<Case>: <TrajName> Target Trajectory`.

**How to apply:**
- Do NOT propose adding an idealised / noiseless comparison regime to the paper. If user asks for one, remind them this was explicitly removed.
- Do NOT reintroduce "realistic" or similar qualifier language on results descriptions, figure captions, or table headers.
- When a reviewer asks "what happens without disturbances?", the answer is "this paper does not characterise that regime; all results are under the disturbance model of Table II".
