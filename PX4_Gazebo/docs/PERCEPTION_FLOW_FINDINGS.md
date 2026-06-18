# Perception layer — optic flow at landing: what's real & the binding limit

> _Rewritten 2026-06-10; renamed from `PERCEPTION_FLOW_UNDERREPORT_BRIEF.md` (that old name was a misnomer —
> flow does **not** under-report). It was an advocacy brief for the hypothesis "optic flow under-reports the
> real velocity 5–25×, the fusion EKF over-suppresses", which was **GT-FALSIFIED (2026-06-08)**; this file now
> records the **correct** understanding. Concise copies of this finding: `PARAMETER_ANALYSIS.md` §4 and memory
> `feedback_flow_underreport_brief_falsified` / `feedback_optic_flow_underreports_root`._

**One line:** the binding perception limit at landing is **LK dynamic range (~2 m/s)**, not a flow
under-report and not the fusion EKF. The at-altitude flow is honest; the divergence that ends in a crash
begins on the **control/latency** side (see `PARAMETER_ANALYSIS.md` §2, the κ-runaway explosion chain).

---

## ⭐ 2026-06-11 — TERMINAL touchdown blocked by a close-range LOOM over-report (clean-baseline landing + video + data)

Traced on an `aruco_board` baseline landing (final xy 1.86 m; got to **min xy 0.0 @ alt 1.5 m** then drifted). The descent law is **correct down to ~1 m**: the z-SMC tracks the loom to `h_rd=−0.42` perfectly (at 0.99 m, measured `h_z=−0.41`, implied rate = ideal −0.414; proportional descent v∝Z). **Below ~0.5 m all corner optic-flow signals over-report together → terminal failure:**
- **LOOM (binding):** corner `h_z` goes noisy/spikes (std **1.45**, a +5.06 outlier at n_corn=4) and **over-reports ~4×** — read −1.5…−2.15 at 0.21 m (implies ~0.35 m/s) while the **actual descent (GT alt-gradient) was ~0.1 m/s (near-ideal)**. The z-SMC reads "4× too fast" → `a_u_z`→−9 → the drone **BALLOONS up 0.21→0.36 m** instead of settling. On a phantom loom.
- **LATERAL:** 1/Z-amplified (`s_e_n`≈1.1, `h_xy`≈2.3) → `a_u_xy`→42 → over-tilt + drift.
- **DECODE:** balloon+drift pushes the board to the FoV edge → `n_corn` 32→0 → open-loop off-center touchdown.

So the "no precise touchdown" is the **same close-range corner-perception breakdown, now on the vertical axis** — not control lag, not a fast descent, not gains.

**The RING divergence is the cleaner terminal loom** (within-Img_Data): std **0.70** (½ the corner's), no spikes, **n_ring 127–151 (4–6× n_corn)**, and still 19–21 stations at touchdown when n_corn=0. **Per-axis is the fix:** the loom is observable from a planar target, so the **ring is a valid VERTICAL signal** → lean on it at the terminal for the descent; keep the **corner for LATERAL** (the ring has ~0 lateral — the *opposite* of the failed `FLOW_NCORN_SWITCH`, which routed the lateral to the ring). Full detail: memory `feedback_terminal_descent_loom_overreport`.

---

## The correct picture (GT-verified — correct time-alignment + smoothed GT velocity)

1. **Flow is HONEST at altitude.** measured corner `h_xy` / GT `v/Z`, binned by altitude:
   `4–6 m ≈ 1.0 · 3–4 m ≈ 1.0 · 2–3 m ≈ 0.85 · 1.5–2 m ≈ 0.7`. **Not 5–25× under.**
2. **The corner+ring fusion EKF is NOT a suppressor.** Offline reproduction of `_ekf_fuse_step` (matches logged
   `Opt Flow Fused`) shows it consumes the **raw per-frame corner** (not a lagging corner-KF) and `h_tr` tracks
   it; it correctly down-weights an *over-reading* ring. The original "fused 0.07 below both inputs" compared
   against a corner-KF that was lagging (carrying the takeoff transient) and an over-reading ring-KF.
3. **The real `h→0` collapse is downstream.** Below ~1.5 m it is **ArUco detection loss** (`N Flow Corners → 0`),
   once the marker has already drifted out of the FoV — i.e. *after* a divergence that starts at altitude where
   the flow is honest.

## The binding perception limit (current)

**LK dynamic range ≈ 2 m/s.** `cv2.calcOpticalFlowPyrLK` saturates to ~0 when apparent motion exceeds its
search window, so fast (>~1.5–2 m/s) lateral/loom motion reads as zero. This is the stochastic TARGET_LOST
failure (1–2 per 5 reps **even at a perfect IC**) and the entire gap between xy_min (~2 m) and xy_median
(~4–6 m). It is **not gain-tunable** — it is a property of the raw flow estimator.

## The perception lever — pyramidal LK FALSIFIED (2026-06-19); real lever = AVAILABILITY

> ⚠️ **2026-06-19 — pyramidal LK is INERT for levels ≥ 3** (memory `feedback_pyramidal_lk_inert`). The
> claim below ("levels 2→3 lifts the ceiling toward 4 m/s") was tested DIRECTLY with a new GT-dynamic-range
> harness `tools/tune_lk_dynamic_range.py` on two IC2 fly-away recordings (`IMG_RECORD_RAW=1`):
> `maxLevel` 3/4/5 and `winSize` 21→51 change the corner flow by **~0 in every GT-speed bin**. When LK
> tracks, the raw lateral flow is **already proportional to GT** (ratio ~1.0–1.3 up to 3 m/s) — it does NOT
> collapse to 0. The binding deficit is **AVAILABILITY**: only ~46–63% of frames yield a valid flow, and the
> track-rate collapses at **close range** (90% @ 5 m → 22–41% @ ~1.7 m) — ArUco-decode / 4-corner-gate /
> lstsq-rejection drops half the frames (close-range corner breakdown), not high-velocity LK saturation.
> The combined-barrier "ṡ ≈ 0.5×" is the controller **holding stale flow** across those dropouts, not a
> raw-LK under-report. **Real lever = decode/track robustness** (KLT corner-track persistence, relax the
> strict 4-corner primary gate, marker/board design) — NOT pyramid depth. Don't sweep `FLOW_LK_LVL`/`FLOW_LK_WIN`.

~~**Pyramidal LK levels 2 → 3** (+ larger search window) — lifts the dynamic-range ceiling toward ~4 m/s.~~
**(falsified above — kept for the record.)** The at-altitude flow is honest, so the perception cure is
**availability**, not a fancier flow estimator; the remaining control-side cure stays as previously held.

## Don't-retry (falsified branches)

- **Retuning the corner+ring fusion EKF Q/R** — the original "primary fix". The fusion is honest; it won't help.
- **Treating "flow under-reports 5–25×" as real** — it was a GT **mis-alignment** artifact: raw finite-difference
  of the jittery bridge pose inflates `|v|` ~2× and manufactures a fake under-report. Use
  `GT_abs = Ground_Truth['Start Time'] + GT['Time']`, uniform-dt resample + savgol before differencing.
- **Fusing IMU/VIO for metric velocity** — violates the hard scale-free / depth-free constraint.

## Method note (why the original brief was wrong)

GT time-alignment + velocity smoothing are load-bearing for any flow-vs-GT check; verify the alignment by
cross-correlating measured loom `h_z` vs GT `v_z/Z` (corr 0.98 at lag ≈ 0). Separately, the KLT off-screen
drift that once produced bogus `s[0]=3.15 rad` / `w_z=4.54 rad/s` is fixed — `img_data` stops the KLT fallback
when any corner exits the image bounds (memory `feedback_theta_norm_klt_drift`).
