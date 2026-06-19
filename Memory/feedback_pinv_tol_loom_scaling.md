---
name: feedback_pinv_tol_loom_scaling
description: "⭐ The loom V_h(3)=vz/z is the σ_min (weak depth-observability) mode of the joint pinv(L_s) flow solve → the SVD tolerance is a truncated-SVD BIAS/VARIANCE dial and NO value wins (truncate=lose loom→sign-flip; keep=amplify noise→limit cycle). MATLAB σ_min(L_s)≈2 (tol=4 kills it, tol=0.01 keeps it); PX4 _fill_A σ_min≈0.077 — spectra scaled ~26-130× apart so the absolute tol doesn't transfer. ESCAPE (PX4-verified): estimate the loom DECOUPLED from the marker's apparent-AREA RATE (loom=-½ d(lnA)/dt on ONE consistent marker) — corr 0.16→0.85, rmse 0.88→0.06 vs joint lstsq. NOT the rcond knob."
metadata:
  node_type: feedback
  type: feedback
---

## ⭐ THE RESOLUTION (2026-06-19): decoupled area-rate divergence, NOT a tolerance

`pinv(L_s)·dP/dt` recovers the scale-free flow `V_h=[vx/z; vy/z; vz/z]` (L_s carries no 1/Z;
no metric vz is ever extracted — monocular-safe). The descent loop correctly consumes the
**divergence** `V_h(3)=vz/z`. The problem is purely the ESTIMATOR: the joint 8×6 least-squares
couples the weak loom row to the lateral/rotational columns, and the loom is the **σ_min mode**,
so the SVD tolerance becomes a truncated-SVD bias/variance dial:
- MATLAB Static-IC5-seed4 spectrum: σ_max≈611, σ_min≈2.09 (1.6–4.7), cond≈292; σ_min∈(0.01,4] for
  93% of steps. `tol=4` zeroes σ_min≈2 → loses loom → V_h(3) under-reports → **terminal sign-flip
  → blow-up (peak|vz| 2.10, land=0)**. `tol=0.01` keeps it (×1/σ_min≈0.5) → recovers loom →
  **arrests descent (|vz|→1.19, land=1)** BUT amplifies the noisiest direction → limit cycle →
  suite 44/100 (mean loom err even slightly WORSE 0.248 vs 0.227). No single tolerance wins.
- PX4 `_fill_A` is on V-frame NORMALIZED coords: σ_min≈0.077, σ_max≈4.7 — **scaled ~26–130× off
  MATLAB**, so the absolute `tol=0.01` is below σ_min → INERT (byte-identical to lstsq rcond=1e-3).
  The PX4 analog of the dial is the RELATIVE `FLOW_LSTSQ_RCOND` (added, default 1e-3); raising it
  to 3e-2 halves close-range loom RMSE by killing spikes but attenuates magnitude (0.90→0.66) and
  barely moves corr — same bias/variance tradeoff, confirming no rcond wins.

**ESCAPE = decoupled estimator (PX4-verified offline, centered descent `validation_data/loom_descent`,
raw `test_data/Test_Videos/Fri Jun 19 10-12-55 2026_raw`):** loom from the marker apparent-AREA RATE,
`loom = -½ · d(lnA)/dt` (planar target ⇒ A∝1/z²). On ONE CONSISTENT marker (pin ID=0; pooling the
13-marker board jumps area across sizes — that bug masked it at first) → **corr 0.85, rmse 0.06,
rmse<1.5m 0.07** vs joint-lstsq 0.16/0.88/0.78. >10× better, escapes the dial (no truncation bias,
not the weak coupled mode). Pixel≈virtual on a level descent; use VIRTUAL (tilt-removed) in general.
Matches MATLAB's clean-loom test (termVz 2.19→0.41). Relates to the existing Ring Divergence
([[feedback_terminal_descent_loom_overreport]]).

**PX4 IMPLEMENTATION LANDED (2026-06-19, default-off, py_compile clean):** `img_data.py` env
`FLOW_LOOM_DECOUPLE=1` overrides ONLY `V_v[2]` (lateral h_x/h_y + ω stay from the lstsq) with
`-½·d(ln M)/dt`, M=μ20+μ02 = trace of the de-rotated (V-frame) primary-corner scatter, via a
CAUSAL short-window linear fit (`FLOW_LOOM_WIN`, default 9) over a `(stamp, ln M)` deque; cleared
on marker-loss so the slope never spans a gap. `FLOW_LOOM_GAIN` (default 1.0) = the FoV-overflow
cal gain. Causal-estimator offline check vs GT loom (centered descent): WIN=5 corr 0.69, **WIN=9
corr 0.93 rmse 0.06**, WIN=13 corr 0.97 — vs joint-lstsq 0.16/0.88. ⏳ NOT SITL-validated yet.

**⚠️ SWITCHING BUG found + FIXED during gain calibration (2026-06-19):** the runtime primary =
`min(decoded IDs)` FLICKERS as the board's markers (which span ~7× in physical size, layout
`Images/aruco_board_layout.npy` `[x,y,sz]`) enter/leave decode → M jumps on every switch →
d(lnM)/dt garbage (the corr-0.93 was on a SINGLE pinned marker; raw min-ID = corr 0.41). FIX:
**size-normalize `M/sz²=(f/Z)²`** (marker-independent, switch-continuous) using `_board_layout[primary_id][2]`
(stored `self._primary_id`; fallback sz=1 single-marker). Restores min-ID corr 0.41→0.65, all-markers
0.70. all-markers-median is +0.05 better but needs all-corner plumbing (unavailable in KLT fallback) →
shipped the **min-ID size-norm** (minimal). **GAIN CALIBRATION** (`tools/calibrate_loom_gain.py`, 2 CLEAN
centered descents `validation_data/loom_descent{,_2}`; a 3rd `_1` was a fly-away, excluded):
`FLOW_LOOM_GAIN=1.15` global best-fit (BAKED default) — but MARGINAL (rmse 0.172→0.170) because the
error is ALTITUDE-DEPENDENT not constant-scale: per-alt gain 0.75 @2-3m (noise on |loom|~0.09) → **1.61
@<0.5m** (causal-lag UNDER-read of the ramping terminal loom — OPPOSITE the predicted FoV-overflow
over-read). A scalar can't fix the trend; the terminal under-read needs WIN tuning (lag↔noise, the same
bias/variance tension) — settle by SITL. ⏳ IC1 A/B `FLOW_LOOM_DECOUPLE=1` vs off still pending.

---
## Earlier framing (superseded by the above; kept for the trail)

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
