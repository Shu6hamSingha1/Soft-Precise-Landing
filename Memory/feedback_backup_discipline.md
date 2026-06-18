---
name: feedback_backup_discipline
description: Backup-before-edit discipline — code/config → Obsolete/<subdir>/<name>_vN.ext (every edit); active tex → Drafts/<name>_vN.tex (structural edits only); Literature_Review_Baselines.xlsx edited in place (exception)
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

Consolidated backup-before-edit discipline. Merges three feedback memories. Code/config files get an `Obsolete/` `_vN` snapshot before EVERY edit; active tex files get a `Drafts/` `_vN.tex` snapshot before STRUCTURAL edits only; `Literature_Review_Baselines.xlsx` is the one in-place exception. Sections below are verbatim.

## General (code/config files → `Obsolete/<subdir>/<name>_vN.ext`)

**Scope broadened 2026-05-31:** back up EVERY file to `Obsolete/<mirrored_subdir>/<filename>_vN.ext` before editing it — not only architectural refactors. This now includes one-line label changes, comment edits, bib additions, doc tweaks. The user asked for this after I skipped `_vN` snapshots for `run_comparison.m`, `bibliography.bib`, and `CLAUDE.md` (treating them as "trivial") during the Chen->Lin2023 baseline swap. (The original 43-day-old rule was architectural-edits-only; superseded.)

N is the next free version number for that filename. Example: `Obsolete/Multi_init_cond/MATLAB/run_simulation_v1.m`.

**Why:** User wants a consistent local rollback snapshot for every edit, regardless of size. Git HEAD is a fallback for tracked files, but the user prefers explicit `_vN` copies.

**How to apply:**
- Back up BEFORE the first edit to a file. Any edit counts — refactors, block replacements, gain tweaks, one-line label/comment changes, bib entries, docs.
- When a change touches multiple files, each file gets its own backup before ITS first edit — don't assume backing up one covers the rest.
- Folder convention: `Obsolete/` lives at repo root (not nested), mirrors the source subdirectory structure below it. Keep the original extension (`.m`, `.tex`, `.bib`, `.md`, `.py`).
- Version numbers increment per file; check `ls Obsolete/<subdir>/` first to pick the next free N (if `_v1` exists, new backup is `_v2`).
- New files created this session need no backup (no prior version exists).
- If you forget and the user flags it mid-edit: reconstruct the true pre-edit version with `git show HEAD:<file> > Obsolete/.../<name>_vN.ext` (works for tracked files), or reverse-apply your own edits on a copy.
- Note: `Obsolete/` is gitignored — these are local safety copies, not committed.

## Tex (active tex files → `Drafts/<name>_vN.tex`, STRUCTURAL edits only)

**The rule (locked 2026-04-25):** Before any structural deletion (e.g., deleting a whole subsection) or large-scale rewrite of an active tex file (`manuscript.tex` / `control_formulation.tex` / `results.tex` / `supplemental.tex` / `block_diagram.tex` / `frames_planes.tex`), copy the current file to `Drafts/<name>_vN.tex` as a safety snapshot. Increment N based on existing Drafts content.

**Current snapshots in `Drafts/`:**
- `control_formulation_v1.tex` — pre-Approach-2 (2026-04 era, pre-funnel-margin cone clamp rewrite)
- `control_formulation_v2.tex`, `manuscript_v2.tex`, `results_v2.tex`, `supplemental_v2.tex`, `block_diagram_v2.tex`, `frames_planes_v2.tex` — pre-idealised-removal (2026-04-25, created before deleting §S3-A and stripping all "realistic" contrast language)

**When to back up (triggers):**
- Deleting an entire subsection or theorem.
- Reordering 3+ subsections.
- Global find-replace touching 20+ sites across multiple files.
- Rename of a symbol convention used in ≥10 equations.
- Wholesale rewrite of an abstract, conclusion, or intro.

**When NOT to back up (skip):**
- Single-sentence tightening.
- Adding one equation.
- Minor notation polish (e.g., `IC$N$` → `IC$_N$` rename is borderline — that one did not get a `_v3` backup because it was safe and reversible via git).
- Changes already committed; git history is itself the backup.

**How to apply:**
1. Before the structural edit, copy each active tex file to `Soft_Precise_Landing/Drafts/<name>_vN.tex` (e.g., `cp "Soft_Precise_Landing/manuscript.tex" "Soft_Precise_Landing/Drafts/manuscript_vN.tex"`, same for control_formulation / results / supplemental / block_diagram / frames_planes).
2. Commit the snapshot in the same commit as the structural edit, so the git history pairs the "before" and "after" states.
3. Naming: strictly `_vN.tex` (no underscores in the middle, no dates). N increments from the highest existing suffix.

**Sentence-level-prose waiver (2026-06-10):** user waived the backup for a sentence-level abstract rewrite in `manuscript.tex` ("No need for backup since there has not been any significant change") — I had made `Drafts/manuscript_v8.tex` and was told to drop it. For tex prose edits, snapshot only before LARGE deletions/rewrites (section-scale), not sentence/paragraph-level edits. The every-edit rule (General section) still stands for code/config files (.m, .py, .sh, .bib).

## Exception (verbatim) — `Literature_Review_Baselines.xlsx` edited IN PLACE

Edit `Literature_Review_Baselines.xlsx` (the Slide 5 `Landscape` + Slide 6 `Baselines` reference tables) **in place — do not snapshot it to `Obsolete/` before editing**. Settled 2026-06-01 after repeated backup churn.

**Why:** the file is small, the user re-pastes its tables into the PPT and re-derives state easily, and the user explicitly asked to stop backing it up and to delete all its `Obsolete/` copies.

**How to apply:** for this one file, skip the snapshot step from the General/Tex sections above; delete any existing `Obsolete/` snapshot of it. The general backup-before-edit rule still holds for tex/MATLAB/other files.
