---
name: feedback_pinv_tol_loom_scaling
description: "MATLAB pinv(L_s, tol=0.01) loom improvement does NOT transfer literally to PX4: PX4's interaction matrix _fill_A is on V-frame NORMALIZED coords (σmin~0.047 > 0.01) so an absolute tol=0.01 never truncates = INERT (byte-identical to current lstsq rcond=1e-3). The regularizing EFFECT it would have (truncate the rank-deficient loom σ-direction, ≈ lstsq rcond=1e-2) is MIXED n=2 on off-center fly-aways (helps one rep loom corr 0.11→0.27 rmse 1.62→0.99, hurts another 0.52→0.38). Proper test = CENTERED descent (loom rank-deficient at x,y→0), not fly-aways."
metadata:
  node_type: feedback
  type: feedback
---

**The MATLAB `pinv(L_s, tol)` loom finding is scale- and regime-specific; don't port the
number.** Checked 2026-06-19 against PX4 offline (loom V_v[2] vs `gt_optical_flow` loom on
2 IC2 fly-away recordings).

- **Literal `tol=0.01` (absolute, MATLAB semantics) = INERT in PX4.** PX4's `_fill_A` (the
  interaction matrix `L_s`) is evaluated on **V-frame normalized** coordinates → smallest
  singular value ~0.047, largest ~4.7 (cond ~100). An absolute cutoff of 0.01 is below σmin,
  so it truncates nothing → identical to the current `np.linalg.lstsq(A, Y, rcond=1e-3)`.
  numpy `rcond` is RELATIVE (× σmax); MATLAB `pinv(A,tol)` is ABSOLUTE. MATLAB's `L_s` must be
  normalized so its σ-spectrum sits near 0.01 for `tol=0.01` to bite — PX4's does not.
- **The regularizing effect (truncate the σmin loom direction) ≈ `lstsq rcond=1e-2` in PX4.**
  n=2 MIXED: rep1 loom corr 0.11→0.27, ratio 1.34→1.13, rmse 1.62→0.99 (better); rep2
  corr 0.52→0.38, rmse 0.64→0.77 (worse). Not a clean win by the sweep methodology.
- **VERIFIED on a CENTERED descent (2026-06-19)** — `validation_data/loom_descent` (lateral
  median 0.70 m, alt 4.2→0 m, `record_output_validation.py VALIDATION_PROFILE=landing` +
  `IMG_RECORD_RAW=1`; raw frames `test_data/Test_Videos/Fri Jun 19 10-12-55 2026_raw`). Here the
  loom column IS rank-deficient and the EFFECT reproduces **monotonically**: loom-vs-GT RMSE
  0.88 (rcond 1e-3) → 0.67 (1e-2) → **0.47 (3e-2)**, and CLOSE-RANGE (<1.5 m, where loom drives
  touchdown) **0.78 → 0.42 (nearly halved)**. BUT it's a **spike/magnitude TRADEOFF, not free
  accuracy**: correlation barely moves (0.16→0.19) and loom magnitude ATTENUATES (ratio
  0.90→0.66, ~1.5× under). Mechanism = truncating σmin kills the phantom-loom SPIKES (the
  [[feedback_terminal_descent_loom_overreport]] balloon-on-phantom-loom failure) while also
  shrinking genuine loom. The literal MATLAB `tol=0.01` STILL inert here (σmin=0.077). 
- **PX4 lever = `FLOW_LSTSQ_RCOND` (default 1e-3 = unchanged)**, added to `img_data.py` corner
  lstsq. Set `FLOW_LSTSQ_RCOND=3e-2` to test the regularized loom. **NOT baked — SITL-validate**
  (spike-vs-magnitude tradeoff; n=1 recording). Pairs with [[feedback_pyramidal_lk_inert]]
  (offline harness `tune_lk_dynamic_range.py`).
