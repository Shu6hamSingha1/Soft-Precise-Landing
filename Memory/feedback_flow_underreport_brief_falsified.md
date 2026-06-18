---
name: feedback_flow_underreport_brief_falsified
description: "GT-VERIFIED CORRECTION (2026-06-08) to PERCEPTION_FLOW_FINDINGS.md (formerly PERCEPTION_FLOW_UNDERREPORT_BRIEF) / [[feedback_optic_flow_underreports_root]]: the claim that the fusion EKF over-suppresses lateral flow 5-25x AT ALTITUDE is FALSIFIED. With correct GT time-alignment (GT_abs = Ground_Truth['Start Time'] + GT Time, matches Img Time; cross-corr 0.98), the measured corner/fused flow tracks GT v/Z to ratio ~0.7-1.9 wherever the marker is visible (alt > ~1.5-2 m). The fusion faithfully passes the raw corner (reproduced offline). The brief's '5-25x under at altitude' was an artifact of (a) raw finite-difference of jittery bridge GT pose inflating GT velocity ~2x, and (b) comparing fused against the LAGGING corner-KF / an OVER-reading ring-KF. The real perception gap is the LK dynamic-range collapse for FAST motion (>~1.5-2 m/s) hitting BOTH corner and ring, plus ArUco DETECTION loss (nfc=0) once the drone has drifted the marker out of FoV — both DOWNSTREAM of a divergence that begins at altitude where the flow is honest."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 4078f031-2152-4b28-a97c-3faa6d6e1fb3
---

**The 2026-06-08 perception brief's primary diagnosis is GT-falsified.** Re-traced the cited rep (`test_data/P2infZ_up/20260608-135447/PLASMC_P2INF_Z=1.5/rep1`) plus DescentFix_n6 / Nz05_n6 baselines with proper GT alignment.

**Method note (the fix that overturned it):** GT and Img_Data use the same perf_counter; align via `GT_abs = Ground_Truth['Start Time'] + GT['Time']` → maps onto Img `Time`. Verified by cross-correlating measured loom h_z vs GT v_z/Z: corr **0.98** at lag ~0. GT velocity MUST be smoothed (uniform-dt resample + savgol, see [[feedback_gt_noise_uniform_dt]]); raw finite-difference of the bridge pose inflates |v| ~2x and manufactures a fake under-report.

**What GT actually shows — AGGREGATED over 73 reps (46 n6 + 27 n3 sweeps), median corner ratio = measured h_xy / GT v_lat/Z, binned by altitude, robust across savgol windows. Last col = frac of frames with N Flow Corners==0:**
- alt 4–6 m: ratio **1.2** [IQR 0.99–1.46], nfc=0 frac 0.02 — **honest**
- alt 3–4 m: **0.92** [0.85–1.12], nfc=0 0.05
- alt 2–3 m: **0.79** [0.30–0.88], nfc=0 0.20
- alt 1.5–2 m: **0.68** [0.00–0.78], nfc=0 0.34 (mild under-report as |v| passes ~1 m/s, detection starting to drop)
- alt 1–1.5 m: ratio → **0.0** (nfc=0 in 58%)
- alt < 1 m: 0.0 (nfc=0 in ~80%)

The collapse is **ArUco detection loss** (nfc→0, monotonic 0.02→0.80 with descent/divergence — not LK decorrelation, not the lstsq bad-gate). The ring ALSO collapses there (stations 120→21, ratio →0.04).

**The fusion EKF is NOT the suppressor.** Reproduced `_ekf_fuse_step` offline (matches logged `Opt Flow Fused`): it consumes the **raw per-frame** corner (`cal_hw@raw`), not the corner-KF, and `h_tr` tracks it. At the brief's cited moment (t≈1.45): raw corner=0.175 (≈GT 0.19), fused=0.166, ring=0.68. The fused correctly tracks the honest corner and down-weights an **over-reading** ring — the opposite of suppression. The brief's "corner-KF 0.96 / fused 0.07 below both" compared against a lagging KF (carrying the takeoff transient) + an over-reading ring.

**Consequences for action:**
- Retuning the fusion EKF Q/R (the brief's "primary fix") will NOT help — the fusion already passes the honest corner.
- The honest perception levers are: (1) **LK dynamic range** for fast motion (pyramidal levels / larger search window) — pushes ratio 0.7→~1 at |v|~1–2 m/s, modest; (2) keeping the marker in FoV / detection robustness at low alt. BUT both kick in only AFTER the lateral divergence is already underway.
- **The divergence develops at altitude (~1 m/s lateral at 4 m) where the flow IS honest (reports ~0.2).** So the binding cause is NOT a perception under-report at altitude. Either the honest small-flow signal isn't braked hard enough, or it's loop latency — i.e. back on the control/timing side, NOT perception. This re-opens [[feedback_optic_flow_underreports_root]] as the wrong layer for the binding cause.

Tools: `/tmp/flow_vs_gt.py` (altitude-binned ratio), `/tmp/ekf_probe.py` (offline EKF reproduction). See [[feedback_dont_conclude_lag_floor]] (methodology — verify before concluding) and [[feedback_use_gt_yaw_not_ea]] (same lesson: the controller's INPUT signal misled the diagnosis).
