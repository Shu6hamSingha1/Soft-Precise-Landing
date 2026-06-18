---
name: Authoritative paper title and controller name (2026-05-01)
description: Current title is "Monocular Dual-Funnel Adaptive Sliding-Mode Control for Guaranteed Soft-Precise Landing with Target Visibility"; controller is MDF-ASMC; older memories using DF-ASMC or PLASMC refer to the same framework at earlier naming epochs
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
> **CURRENT (per `project_single_file_manuscript_rewrite.md`):** controller name = **VDF-ASMC** (Visual Dual-Funnel Adaptive Sliding-Mode Control); title = **"Scale-Free Adaptive Sliding-Mode Control for Guaranteed Soft-Precise Landing under Target Visibility Constraints"**. The older **PLASMC / DF-ASMC / MDF-ASMC** names below are the SAME framework at earlier naming epochs. The 2026-05-01 record below is retained as naming-history.

**Paper title (as of 2026-05-01):**
*"Monocular Dual-Funnel Adaptive Sliding-Mode Control for Guaranteed Soft-Precise Landing with Target Visibility"*

- Verbatim in `manuscript.tex` L36 (`\title{...}`).
- Running head (`manuscript.tex` L64): `\markboth{SINGHAL ET AL.}{MONOCULAR DUAL-FUNNEL ASMC FOR SOFT-PRECISE LANDING}` — abbreviated form, hyphenated.
- Supplement title (`supplemental.tex` L14): same wording inside the "Supplementary Material for ..." cite.

**Controller name (as of 2026-04-20):** **MDF-ASMC** = *Monocular Dual-Funnel Adaptive Sliding-Mode Control*

**Naming history:**
- PLASMC → early draft name (prescribed-performance/leakage adaptive SMC emphasis)
- DF-ASMC → intermediate name (2026-04-15 compaction; dual-funnel architecture emphasis)
- MDF-ASMC → current name (2026-04-20; adds the monocular-only sensing claim)
- Title update (2026-04-23): "Dual-Funnel Adaptive Sliding-Mode Control for Vision-Only Soft Precise Landing ..." → "Monocular Dual-Funnel Adaptive Sliding-Mode Control for Soft Precise Landing ..." — "Vision-Only" removed because the title's leading "Monocular" already carries the single-camera claim, and the acronym MDF-ASMC encodes it directly.
- Title update (2026-05-01): "Soft Precise Landing with Guaranteed Target Visibility" → "Guaranteed Soft-Precise Landing with Target Visibility" — "Guaranteed" moved forward to qualify "Soft-Precise Landing" (the headline contribution), and "Soft-Precise" hyphenated as a compound adjective. Target visibility remains a guaranteed property (Corollary~1) but is now an enabling capability rather than the primary headline.

**Why the *M* was added:** The user chose to encode the vision-only sensing story directly into the controller acronym rather than leaving it to the title alone. Every downstream citation of the controller now carries the monocular signal, reinforcing that the entire loop (translational + rotational) runs on a single camera — no metric depth, no magnetic compass.

**Why the title was rebalanced 2026-05-01:** The headline contribution is the *guaranteed soft-precise touchdown* (the property the four cited baselines fail to deliver in the comparative benchmark). Putting "Guaranteed" before "Soft-Precise Landing" matches the paper's own framing — every cited baseline can claim *some* form of target visibility, but only MDF-ASMC achieves the functional requirement of soft-precise landing. The rebalance also aligns abstract Sentence 2 ("does not guarantee the soft-precise touchdown"), Contribution (d) ("achieving the functional requirements of soft-precise touchdown..."), and Para 7 ("achieves the functional requirements of both the soft-precise touchdown and target visibility...") with the title's headline framing.

**Why:** These are the reference names for the manuscript, supplement, figure captions, block diagram, code comments, and any future citation. The abstract, contribution list, and conclusion all use "MDF-ASMC" and "soft-precise" (hyphenated) going forward; "DF-ASMC" and "PLASMC" are deprecated.

**How to apply:**
- When writing manuscript/supplement/figure text, use "MDF-ASMC" and the locked title verbatim.
- When reading older memory files (`reference_plasmc_architecture.md`, `project_plasmc_tuning.md`, `reference_plasmc_vs_px4_inner_loop.md`, `project_dual_funnel_parallel_architecture.md`, etc.), the controller they describe is the same framework — just at an earlier naming epoch. Don't mass-rewrite those files; use this memory as the current-name pointer.
- For the `\markboth{}` running head, the abbreviated form "MONOCULAR DUAL-FUNNEL ASMC FOR SOFT-PRECISE LANDING" is acceptable — IEEE TAES allows shorter running heads than the full title; we drop "Guaranteed" and "with Target Visibility" to fit one line.

**Locked terminology (2026-04-20, commit a35955f):**
- **Theorem 1** is titled *"Adaptive Optic-Flow Funnel Invariance"* (NOT "Inner-loop GUUB"; the "inner-loop" framing is explicitly rejected — funnels are parallel, not inner/outer).
- **Corollary 1** is titled *"Closed-loop target visibility"* (NOT "Closed-loop visibility and layered boundedness").
- **Premise phrasing:** "no metric depth or heading measurement" (NOT "pose" or "range" — those were explicitly rejected as redundant).
- **Image-plane phrasing:** "target image orientation in the virtual image plane" (NOT "image-plane orientation of the target").
- **Cone action verb:** the cone *geometrically clips* the commanded acceleration (NOT just "clips" — "geometrically clipped in closed form" is the canonical phrase wherever the cone acts on acceleration). The cone itself is consistently called the **attitude cone**, never "visibility cone" (the visibility *funnel* sizes the attitude *cone*).
- **Outer-loop PID name:** "image-feature error PID" (NOT "normalized-error PID on the image-feature error" — that phrasing was flagged as redundant, since "normalized-error" implies scale-independence).
