---
name: feedback_recurring_analysis_mistakes
description: "PRE-FLIGHT CHECKLIST of the analysis mistakes Claude makes REPEATEDLY on this project, each with dated instances and the one check that catches it. Recurring classes: log-to-log time alignment; pairing assumed from directory names; reference-frame/offset (rover rel-z 0.5 MEANS landed); metric sampled at the wrong instant; confounded or non-overlapping comparisons; one-sided metrics (catch rate without false-positive rate, message without base rate); stale derived docs trusted over source; unverified baselines under concurrent sessions. ~85 of ~200 memory files record a correction. Run these BEFORE concluding, not after."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-02T00:00:00.000Z
---

**Written 2026-09-02 at user request**, after a single session re-committed several of these.
**~85 of ~200 memory files contain a CORRECTION / RETRACTION / SUPERSEDED / FALSIFIED note.**
The same handful of shapes recur. This is the checklist, not a confession list.

> ⚠ **The meta-lesson, and the reason this file exists:** recording a trap does NOT prevent
> re-committing it. On 2026-09-02 I derived the rover platform constant empirically, wrote it
> into memory *as a trap to avoid*, and then fell into it about an hour later while reading a
> chart. Knowing the fact is not the control; **running the check before concluding is.** The
> common shape of every failure below is: reach a conclusion, then read weak evidence as
> confirmation.

## 1. Log-to-log TIME ALIGNMENT (the most repeated one)

Different logs are on different clocks and different windows. Correlating by INDEX, or
assuming coverage, is invalid.

- 2026-06-04 [[feedback_imgdata_gt_clock_skew]]: `Img_Data` (perf_counter, ~83 Hz) vs
  `Ground_Truth` (mission-relative, ~125 Hz), Δorigin ~29.8 s, non-overlapping ranges.
  Index-correlating falsified an "alpha doesn't track yaw" conclusion (real r=1.00).
- 2026-08-27: that file PARTIALLY SUPERSEDED — a validated direct fix exists
  (`Img_Data['Time'] - gt['Start Time']`).
- **2026-09-02, hit again in a new form:** `validate_detector_gt.py` aligns frames to the tail
  of `Img_Data` and interpolates GT with `np.interp`, which **CLAMPS** outside the GT window
  instead of rejecting. On one hand-paired run **117/308 frames (38 %) fell past the end of GT**
  and were scored against frozen touchdown values — reporting detOK 49 % where the truth was 18 %.

**CHECK:** print both series' `t0..t1`, sample rate and length, and the OVERLAP, before any
comparison. Filter to `t_g[0] <= t <= t_g[-1]`. Prefer the curated `--set` path over
hand-pairing.

## 2. PAIRING assumed from directory/file names

- 2026-09-01: `validate_detector_gt.py`'s own docstring warns the data-dir ↔ `_raw`-dir pairing
  is MANUAL (data dir lags ~13 s).
- [[reference_final_landing_recordings]] records that IC1-4 chase↔dataset links rest on
  save-time adjacency + visual confirmation only.
- **2026-09-02:** I claimed `montage_rover_static_ic1*.mp4` showed a non-landing because I
  paired it to `Cross_Marker_Montage_Rover` on directory-name similarity. WRONG — mtimes showed
  the montage was written 61 s after `Rover_Static_IC1_Montage/rep1.out`, a genuine 0.487 m
  landing. The filename had named the right directory all along.

**CHECK:** confirm a pairing by **file mtimes** AND by matching a plotted/recorded trace against
the candidate run's data. Never by name similarity.

## 3. REFERENCE FRAME / OFFSET confusion

The single richest source of wrong conclusions in this project.

- **Rover:** the target pose is the rover BASE (z~0.02), not the landing surface. The platform
  top is 0.50 m. **`uav.z - target.z ~ 0.49` MEANS LANDED**, not airborne. Verified empirically:
  settled rover runs rest at rel-z **0.487, mode 0.5, n=12**.
  - 2026-09-02 instance A: a first-pass archive audit using a flat 0.20 m floor condemned **34
    genuine platform landings**.
  - 2026-09-02 instance B: an hour after writing (A) down, I read a montage's
    `|rel. position| -> 0.5` as "hovering above the plate". It meant landed.
- [[feedback_image_center_bug]]: the centre bug was in the NOTEBOOK, not `img_data.py`.
- The `_resolution = (480, 640)` / `center = (240, 320)` pairing in `img_data.py` is correct for
  the POST-`ROTATE_90_CW` frame — don't "fix" one without the other (CLAUDE.md).
- The 90° SMC↔CBF `Rz_p90b` convention: a synthetic-validated "fix" (13/13) regressed every IC
  in SITL and was reverted (`4d7bc210`).

**CHECK:** for any height/position claim, state the datum explicitly and verify it against runs
with a KNOWN outcome before interpreting.

## 4. Metric sampled at the WRONG INSTANT / no absolute gate

- [[feedback_false_sp_frozen_gt]] (2026-06-10): frozen→origin-reset GT gives `xy_err`~1e-21 and
  trips `precise`.
- [[feedback_relative_flatness_needs_absolute_gate]]: a "stopped changing relative to its own
  history" test proves local stationarity, NOT a terminal state — learned from **two consecutive**
  touchdown-detector bugs (2026-08-26, 2026-08-28).
- 2026-09-02 [[project_20260902_archive_rescore_false_precise]]: **159 of 1192 (13.4 %)** archived
  `precise` verdicts were computed mid-air; `xy_err` was read at whatever instant the control loop
  exited. Fixed `d6610ea7`.

**CHECK:** does the evaluation instant correspond to a VERIFIED terminal state, gated on an
absolute quantity — not just a relative/flatness one?

## 5. Confounded or NON-OVERLAPPING comparison presented as a mechanism

- [[feedback_historical_cal_confound]] (user, 2026-06-03): ~2000 reps' "lag is the floor"
  conclusions were confounded by a 2-13× broken output calibration.
- [[feedback_correlation_needs_pooling]]: Pearson on a near-constant true signal is
  noise-dominated; pool across flights before trusting the SIGN.
- [[feedback_sensitivity_sweep_methodology]]: n=1 is noise.
- **2026-09-02:** I asserted a "~30° obliquity cliff" in detection from two datasets whose angle
  ranges **barely overlap**. Where they DO overlap (30-45°) they disagree completely (rover_IC4
  100 % vs rover_IC2 31 %) — angle was confounded with altitude. Retracted.

**CHECK:** do the compared groups overlap on the proposed explanatory variable? Is there a
matched pair (same scene, one variable changed)? The flat-vs-clutter pair was valid *because*
it was controlled; the IC2-vs-IC4 comparison was not.

## 6. ONE-SIDED metric — a rate quoted without its complement

- **2026-09-02:** recommended a `|dz/dt|` terminal-rest gate citing its CATCH rate (152/159)
  without measuring its FALSE-POSITIVE rate. It rejects **58 % of LEGITIMATE landings**. Wrong,
  retracted, recorded in [[project_20260902_archive_rescore_false_precise]].
- **2026-09-02:** read `Disarming denied: not landed` as PX4 contradicting a touchdown latch.
  Base rate over 2396 paired runs: **97.4 % of genuine landings**. It is routine noise.
- 2026-09-01: `validate_cbf.py` 13/13 synthetic passes were "misleading" — the fix regressed
  every IC in SITL.

**CHECK:** a detection rate needs its false-positive rate; a log message needs its BASE RATE in
the negative class; a synthetic pass needs a real-data confirmation.

## 7. Stale DERIVED doc trusted over source

- CLAUDE.md itself warns the sensor-cal block "will be stale; read the live values in
  `img_data.py`".
- 2026-09-02 [[feedback_verify_injected_docs_before_trusting]] (another session): the
  auto-injected STATUS block and CLAUDE.md both asserted wrong gains/camera facts.
- [[project_20260825_overlay_detection_artifact_logging]] is stamped CONTRADICTED — verify
  directly before trusting its line.

**CHECK:** quote load-bearing numbers from source files, never from a doc, a memory, or an
injected block. Memories reflect what was true when written.

## 8. BASELINE not verified (multi-session repo)

- 2026-09-02 [[feedback_ab_baseline_verify_concurrent_commits]]: another session committed the
  exact fix under test 3 minutes before my A/B ran, so the "baseline" arm already contained it.
  A real fix measured as a 0 % delta and I wrongly retracted the root cause.
- Sibling: [[feedback_check_concurrent_sitl_before_launch]] — 3+ `claude` sessions commit to this
  repo concurrently (verified: `session_01QgSdJd`, `01Jtuav4`, `01D2gjuv`, plus mine, all on
  2026-09-02).

**CHECK:** `git log --oneline -5` immediately before an A/B and again before writing up a NULL
result; pin the control arm by explicit revert env; echo the live constant per arm. **A null
result is the trigger to re-verify the baseline, not to conclude the mechanism was wrong.**

## Also worth screening for

- 2026-09-02: the recorded `_raw`/frames PNGs carry a **drawn debug overlay**
  (`CROSS_RING_OVERLAY_DBG` default ON) — live detection is unaffected but offline replay is
  contaminated. [[feedback_detector_offline_replay_gotchas]]
- Headline detOK%/SP% are inflated by post-touchdown ground frames — read per-ALTITUDE bands.
- 2026-08-25: a correctly-worded HARD RULE buried mid-file (`WORLD=cross_marker
  MARKER_TYPE=cross`) was violated across an entire session once a command pattern got
  copy-pasted. Rules need to fire at the point of action, not sit in a file.
