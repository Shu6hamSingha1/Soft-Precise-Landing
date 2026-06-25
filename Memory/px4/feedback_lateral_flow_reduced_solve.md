---
name: feedback_lateral_flow_reduced_solve
description: "⭐ Lateral flow h_xy is the sigma_min mode of the full 8x6 lstsq (degenerate w/ tilt w_xy, principal angle 0.4deg) -> noise-amplified (corr-vs-GT 0.1-0.66). FIX = REDUCED solve: drop the w_xy columns (V-frame leveled -> level-target w_xy~0) -> h_xy becomes largest-sigma (cond 14->2, 206x less noise; corr 0.2-0.3->0.5-0.65). BAKED FLOW_LAT_REDUCED=1. ⚠ REQUIRES paired recal (cal h_x/h_y rows are a w_xy RECOMBINATION -> break under the reduced solve, ~3x under-read)."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**WHY h_xy perception is poor + the REDUCED-SOLVE fix (2026-06-25, user-led).** Lateral flow `h_xy`
correlates only 0.1-0.66 with GT (loom h_z is 0.92). ROOT: in the full 8x6 corner-flow lstsq, the
columns for lateral translation (`h_xy`) and tilt rotation (`w_xy`) are NEAR-PARALLEL — the **principal
angle between their column subspaces is 0.4deg** (a downward camera + small marker: a sideways image
shift is ambiguous between "drone slid" and "drone tilted"). So `h_xy` is the **sigma_min mode** (SVD:
cond 14, lateral noise-gain ~146), and per-corner LK jitter is amplified into it. NOT translation/rotation
LEAK (GT h_x corr to perception w_y ~0), NOT availability (91% in descent) — it's SNR: small lateral
signal (mostly-vertical descent) buried under the sigma_min noise floor (ratio P/GT 160x at low flow).

**FIX = REDUCED 4-DOF solve.** The V-frame is gravity-LEVELED, so a LEVEL target has rotational flow
`w_xy ~ 0` (drone tilt leveled out, no target tilt). DROP the w_xy columns (3,4) -> solve `[h_x,h_y,h_z,w_z]`
-> `h_xy` becomes the LARGEST-sigma mode (SVD: cond 14->2, lateral noise-gain 146->0.71, **206x cleaner**).
Loom/yaw (h_z,w_z) are 90deg-orthogonal to the lateral blocks (separable); the moment loom still overrides
h_z. **Offline-reprocess (difference logged V-frame corners, full vs reduced vs GT): corr h_x 0.20-0.28
-> 0.52-0.65, h_y 0.11-0.47 -> 0.45-0.59.** Reaches ~0.5-0.65 not the loom's 0.9 — the residual gap is
SNR (lateral signal intrinsically small), NOT conditioning.

**IMPLEMENTED + BAKED** (img_data.py): `FLOW_LAT_REDUCED=1` (default-ON; drop w_xy cols via
np.delete(A,[3,4]); w_xy set 0) + `FLOW_TARGET_LEVEL=1` (level-target gate: =0 falls back to the FULL
solve for a TILTING target — a ship deck's pitch/roll is REAL w_xy not in the level assumption, and the
IMU can't supply it because it only sees the DRONE's tilt, not the target's). Target yaw/translation are
fine either way (w_z orthogonal; h_xy = relative velocity).

**⚠ HARD PREREQUISITE — PAIRED RECAL (the cal is a w_xy RECOMBINATION).** sensor_cal_hw rows 0,1 are NOT
diagonal — they are `h_x_cal = 0.86*h_x_raw + 0.87*w_y_raw`, `h_y_cal = 0.79*h_y_raw - 0.78*w_x_raw`:
the cal UNDOES the full-solve degeneracy split at the CAL stage (recombines the split-off raw w_xy back
into h_xy). The reduced solve does that recombination at the SOLVE stage (cleaner, pre-noise) AND zeros
w_xy -> the cal's w_xy cross-terms go INERT -> calibrated h_xy UNDER-READS ~3x. The two are MUTUALLY
EXCLUSIVE: reduced solve needs the h_x/h_y cal rows RE-DERIVED to ~DIAGONAL (no w_xy). **Recal = run
output-calibration with FLOW_LAT_REDUCED=1 + aggregate_calibration_phased.py -> new diagonal rows.**
GT-FB is UNAFFECTED (perception not consumed) so the baked default is safe for the control work; but
**production/perception-on is a REGRESSION until the recal** (corr is better, scale ~3x off).

**How to apply.** The reduced-solve `h_xy` cleanup is the PREREQUISITE for the kappa-lateral fix:
`N_xy=0.02` was kept tiny to avoid NOISE-PUMPING `kappa_xy` (it's frozen, tau=33s>>7s descent —
[[feedback_kappa_xy_frozen]] / the 4-axis kappa check); a cleaner sigma_xy lets N_xy be raised so
kappa_xy adapts. Sequence when SITL resumes: (1) recal h_xy, (2) SITL-validate FLOW_LAT_REDUCED, (3)
raise N_xy. Continues [[feedback_sp_task2_terminal_limit_cycle]], [[project_gt_feedback_control_tuning]].
