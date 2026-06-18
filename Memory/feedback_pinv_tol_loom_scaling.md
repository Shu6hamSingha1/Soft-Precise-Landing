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
- **Wrong regime:** both recordings are off-center fly-aways where the loom column of `L_s`
  is well-conditioned (x,y large) → regularization barely helps / can hurt. The MATLAB benefit
  is expected in a **CENTERED descent** (x,y→0 ⇒ loom genuinely rank-deficient). To verify:
  record a centered descent with `IMG_RECORD_RAW=1` and rerun the loom-vs-GT solve sweep
  (the one-off script lived in-session; rebuild from `tune_lk_dynamic_range.py` helpers +
  swappable solve fn). Pairs with [[feedback_terminal_descent_loom_overreport]] (close-range
  loom over-report) and [[feedback_pyramidal_lk_inert]] (the offline harness).
