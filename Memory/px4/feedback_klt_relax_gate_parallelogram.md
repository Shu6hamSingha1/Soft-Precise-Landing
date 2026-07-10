---
name: feedback_klt_relax_gate_parallelogram
description: "KLT-fallback relaxed 3/4-corner gate + parallelogram completion (MARKER_KLT_RELAX_GATE, BAKED default-ON 2026-07-09) - fixes momentary 1-2 frame decode flicker; needs only 2D pixels + ArUco corner order (no depth/scale/intrinsics); weak-perspective caveat."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 32b3f473-5219-48ce-b070-323ef9cc309a
---

**Mechanism validated via new "KLT Diag" per-attempt logging** (`_klt_diag_log` in img_data.py, log key "KLT Diag": t/n_tracked/gate4_passed/gate_accept_passed/reconstructed_idx/in_bounds/accepted; analysis tool `tools/diagnose_klt_gate4.py`). One instrumented landing split cleanly into TWO regimes: (a) **momentary flicker** during clean tracking — 100% of gate rejections were exactly 3/4 corners tracked (one corner loses LK lock 1-2 frames); (b) **cascading collapse** after a terminal event — monotonic 2→1→0, a relaxed gate helps ZERO of these. Aggregate stats (31.9% rescue) were misleading until segmented by regime — segment before judging.

**Fix (BAKED default-ON, commits 12f1f32 + bb0a675):** accept n_tracked≥3; reconstruct the missing corner by PARALLELOGRAM COMPLETION — ArUco corner order [TL,TR,BR,BL] ⇒ opposite corners share the diagonal midpoint (c0+c2=c1+c3) ⇒ `missing = other1 + other2 − diagonal_partner`, raw pixels. **Needs ONLY the 3 tracked 2D pixel positions + the corner-order convention — no depth, no marker size, no intrinsics, no scale** (user asked explicitly; scale-free-safe). Existing in-bounds check stays as safety net. `MARKER_KLT_RELAX_GATE=0` reverts.

**Live verification:** reconstruction fired and the completed corner sat smoothly in the surrounding trend — and was MORE plausible than the raw detection 2 frames earlier (an outlier jump that likely caused the dropout). In the IC1_rep1 post-touchdown divergence it fired 3× in a row correctly WITHOUT injecting new corruption (the yaw corruption predated the first firing) — it delays but cannot stop an escalating attitude divergence (not its job).

**Caveats:** (1) parallelogram identity is exact only under weak-perspective — degrades with keystoning (large marker fill + big tilt = exactly the terminal regime; separate validation needed before trusting it there — see [[project_partial_marker_parallelogram_recovery]] for the extension idea to genuinely partial/occluded markers). (2) LK can fail SILENTLY (status=1 but corner frozen in place across frames — observed pixel-identical corners over ~70ms during an oblique-view divergence) so "3/4 tracked" can overcount; upstream of this gate. (3) n=1-validated per regime; behaved cleanly in the full baked IC1-5 sweep (not implicated in any failure).
