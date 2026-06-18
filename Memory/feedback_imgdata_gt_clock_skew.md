---
name: imgdata-gt-clock-skew
description: "METHODOLOGY TRAP (found 2026-06-04): Img_Data.npy and Ground_Truth.npy in output_calibration recordings are on DIFFERENT clocks — correlating them BY INDEX is invalid and produces spurious r≈0. Always use the GT log's co-sampled image features instead. This artifact falsified the 'alpha doesn't track yaw' conclusion (real r=1.00)."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**In `output_calibration` (and likely landing) recordings, `Img_Data.npy` and `Ground_Truth.npy`
are logged on SEPARATE, NON-ALIGNED clocks.** Measured on output_postreboot run1:
- `Img_Data['Time']`  = perf_counter: 29.80–98.71 s, ~83 Hz, 5937 samples
- `Ground_Truth['Time']` = mission-relative: 0.00–52.90 s, ~125 Hz, 7400 samples
- Non-overlapping ranges, different lengths, Δorigin ≈ 29.8 s.

**So correlating `Img_Data[...][k]` against `Ground_Truth[...][k]` BY INDEX compares two different
moments in time → spurious r≈0 even when the signals match perfectly.** `n=min(len_a,len_b)` +
truncate does NOT fix it (the clocks still differ).

**THE FIX — use the GT log's CO-SAMPLED image features** (the GT thread records both on its own clock):
`Ground_Truth['Img Feature Params']` (= live `s`=[xc,yc,1,alpha]) and `Ground_Truth['Opt Flow Ang Vel']`.
These ARE time-aligned with `Ground_Truth['UAV Pose']`. (Or interpolate Img_Data onto a common clock,
but the co-sampled GT arrays are simplest.) `compute_gt_signals`-style tools that pull from Img_Data are
suspect — audit them.

**What this artifact FALSIFIED (re-derived correctly 2026-06-04, both signals on GT clock):**
- **alpha (yaw heading): r = 1.00, slope = +1.00** across 5 runs (alpha ptp 75–80° == GT-yaw ptp 74–79°).
  Index-aligned had shown slope −0.09, r −0.04…−0.29 → the basis for the now-WRONG "alpha can't be
  calibrated / compass-drift confound" in [[yaw-calibration-pending]]. Alpha is a PERFECT heading sensor.
- **wz (yaw rate): r = 0.75** (index-aligned showed ~0.07). Well-observed.
- **wx/wy (roll/pitch rate): r ≈ 0.0 even when aligned** — this one is REAL (planar-marker rotation/
  translation ambiguity), NOT an artifact. So "use IMU for roll/pitch, target wx/wy unobservable" stands (OVERTURNED 2026-06-07 — wx/wy ARE observable w/ the spread board; zeroing is a level-target choice; [[wxy-unobservable-imu-fusion-deferred]]).

**Consequence:** any historical conclusion built on index-correlating Img_Data vs Ground_Truth needs
re-checking on the GT clock. Yaw (alpha + wz) is observable and clean; only roll/pitch is unobservable (OVERTURNED 2026-06-07 — wx/wy ARE observable w/ the spread board; zeroing is a level-target choice; [[wxy-unobservable-imu-fusion-deferred]]).
See [[yaw-calibration-pending]] (corrected), [[per-axis-tuning]], [[compass-yaw-drift]].

**AUDIT (2026-06-04): production tooling is CLEAN — the bug was confined to ad-hoc scripts + one memory
conclusion.** Swept tools/, apps/, src/ for cross-clock index-correlation:
- **Cal derivation is SAFE.** `aggregate_calibration_phased.py`, `aggregate_calibration.py`,
  `analyze_calibration.py` all take the image-side signal from `gt['Opt Flow Ang Vel']` and
  `gt['Img Feature Params']` — the GT thread's CO-SAMPLED features (same clock as `gt['UAV Pose']`),
  NOT from Img_Data. So the derived `sensor_cal_hw` / `sensor_cal_s` are NOT contaminated by clock skew.
  (Lesson: the recordings deliberately co-sample image features into the GT log precisely so cal
  derivation is single-clock. Use those, never Img_Data, for any image-vs-GT comparison.)
- `analyze_loop_latency.py` builds a common 5 ms grid + cross-correlates Control vs Telemetry (finds lag) — aligned.
- `analyze_timeseries.py` / `analyze_marker_switch.py` / `diagnose_intervention_reps.py` correlate ACROSS REPS
  (one scalar metric vs one xy_err per rep) — not within-rep cross-clock time-series; safe.
- Final regex sweep: zero files corrcoef/polyfit Img_Data-clocked data against GT poses without alignment.
So NO sensor_cal re-derivation is needed on account of this bug; only my session scratch scripts and the
old [[yaw-calibration-pending]] alpha conclusion were affected (both corrected).
