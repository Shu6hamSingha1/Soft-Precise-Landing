---
name: Trim implementation-detail filenames from manuscript prose
description: Do not name implementation scripts, harness files, or internal functions in main-paper prose; page budget is tight
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Do not mention implementation-level filenames (e.g., `init_robustness.m`, `run_simulation.m`, MATLAB script paths) in the main manuscript prose. Replace with a one-phrase description of *what* the thing does, not where it lives.

**Why:** The IEEE TAES page budget is tight and the reader does not need to know the harness filename. Such details bloat the prose without helping the reader understand the method. The reader needs the *method*, not the directory structure.

**How to apply:**
- In manuscript.tex / results.tex / control_formulation.tex / supplemental.tex prose, do NOT name MATLAB files, script names, or function paths.
- When a concept needs attribution to a shared implementation (e.g., "the same disturbance model across all runs"), state it as a claim — "the same disturbance model is applied in every run" — without the `\texttt{init\_robustness.m}` parenthetical.
- Gain tables, figure captions, section titles may still cite the relevant paper-internal label (Table~S2, Section~S3-C), but not filenames.
- Applies retroactively: when editing any block, trim any already-present filename references in the vicinity rather than leaving them.

**Specific patterns to grep for and rewrite (caught 2026-04-24):**
- `\texttt{...}` — typewriter font often signals a code identifier; check each occurrence.
- MATLAB function calls like `\texttt{rng(1000+k)}`, `\texttt{traj_Gen}`, `\texttt{awgn(...)}` — replace with the conceptual claim ("deterministic per-IC seeding", "the fastest target trajectories", etc.).
- Words "harness", "implementation code", "simulation harness", "our harness" — implementation-jargon leaks. Rewrite as "test setup" / "vision-only setting" / drop the parenthetical entirely.
- File extensions `.m` / `.mat` in prose — same rule.
- Hardware identifier "Intel i7" in the timing claim is acceptable as reproducibility context, not a leak.
- The single environment statement "Simulations are carried out in MATLAB" (results.tex §IV intro) is acceptable scientific prose, not a leak.

**Repeat-pattern audit:** before declaring a manuscript-edit task complete, run:
```
grep -nE '\\texttt\{|\\verb|\.m\b|\.mat\b|harness|implementation code|MATLAB function' Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
```
Any non-trivial hit is a defect to fix before commit.
