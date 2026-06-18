---
name: cal-data-provenance-cleanup
description: "Which recordings the APPLIED sensor_cal came from, and the 2026-06-05 cal-data cleanup. The applied 6x6 sensor_cal_hw + sensor_cal_s were derived from calibration_data/output/ (4 multisine runs) via tools/derive_board_cal.py — NOT from the phased/yawagg recal experiments. Superseded cal data archived to Obsolete/calibration_data/. [UPDATE 2026-06-07: applied cal REFRESHED to the all-13-run corner M + re-keyed transfer ring; this is now historical provenance.]"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 75fad30b-d46f-4f5a-8783-a865d1b91031
---

> **⚠️ UPDATE (2026-06-06) — two corrections + a cal-validity flag:**
> 1. **The cal source MOVED.** The 4 Jun-2 multisine runs that derived the applied
>    cal are now in **`calibration_data/output_pre_fpsfix/`** (renamed at the fps
>    fix). The current `calibration_data/output/` holds the **Jun-6 stamped runs**,
>    NOT the cal source. So `derive_board_cal.py` with `CAL_DIR=output` would now
>    read the WRONG runs — point it at `output_pre_fpsfix/` to reproduce the
>    applied cal.
> 2. **Cal-validity flag.** The applied cal was derived from those Jun-2 recordings,
>    whose V-frame flow **RETAINED roll/pitch** (the leveling effectively didn't
>    remove it — see [[feedback_vframe_rhs_yaw_only]]). The current runtime levels
>    correctly (r/p removed). So the cal may be **mismatched to the current
>    runtime flow** (r/p leaks into h_x/h_y via the rank deficiency). **Re-validate
>    / re-derive from the correctly-leveled `output/` (Jun-6) recordings** before
>    trusting it further. NOT yet done.
> 3. **2026-06-06 cleanup** archived the Jun-5 output/ re-runs, `output_mismatchedLK`,
>    `output_postreboot_20260603`, and `yawagg_cal_20260604-203806` to
>    `Obsolete/calibration_data/` (reversible). `output_postreboot` was previously
>    kept "for a planned `IMG_FEATURE_FILTER=kf` recal" — restore from Obsolete/ if
>    that recal is still live.

**APPLIED calibration provenance (img_data.py):**
- `sensor_cal_hw` is a **full 6x6 matrix** (h<->w cross-terms), NOT the diagonal the
  aggregators (`aggregate_calibration[_phased]`, `analyze_calibration`, `tune_savgol`)
  produce. It comes from **`tools/derive_board_cal.py`** (`CAL_DIR=calibration_data/output`,
  full `np.linalg.lstsq(R,G)` so GT = M @ raw), derived from the **4 multisine runs in
  `calibration_data/output/`** (Tue Jun 2). Commits: `bde925f` ("adopt confirmed 4-run
  multisine M as landing cal of record") + `0986245` (s-cal), both 2026-06-02.
- So `calibration_data/output/` is the **cal-of-record source — keep it**, and keep it where
  `derive_board_cal.py` expects (`CAL_DIR`). The phased (`output_postreboot…`), `yawagg_cal_*`,
  and `rollpitch_cal_*` recordings are LATER recal EXPERIMENTS that did NOT become the applied cal.
- Tools default-scan `calibration_data/output` (derive_board_cal, aggregate_calibration[_phased],
  analyze_calibration, tune_savgol) and `calibration_data/input`.

**2026-06-05 cleanup (user: move unnecessary cal data, don't delete):** moved 110M to
`Obsolete/calibration_data/` (both calibration_data/ and Obsolete/ are gitignored → reversible,
zero git impact):
- `input_yawgain/`, `input_KMfix/` (one-off input sweeps)
- `rollpitch_cal_20260604-205113/` (wx/wy excitation — deferred; wx/wy now zeroed via
  `CTRL_ZERO_WXY=1`, IMU-wxy rejected, see [[wxy-unobservable-imu-fusion]])
- 5 older `yawagg_cal_*` (kept newest `20260604-203806`)
KEPT in `calibration_data/`: `output/` (cal source), `output_postreboot_20260603-222903/`
(phased, for the planned `IMG_FEATURE_FILTER=kf` recal), `input/`, `yawagg_cal_20260604-203806/`.

See [[feedback_vframe_rhs_yaw_only]] (cal now derives V-frame GT on all channels),
[[reference-aggregate-calibration]].
