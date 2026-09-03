---
name: project-two-output-cals-aruco-vs-cross
description: There are TWO independent output sensor cals (ArUco img_data.py vs cross-marker cross_marker_perception.py) — cross was recalibrated for 320x240, ArUco was not, and the user declined that recal
metadata:
  type: project
---

Easy to conflate, and it happened 2026-09-02/03. "We recalibrated the output cal" is true for
ONE of two independent pipelines:

|  | cross-marker (**recalibrated**) | ArUco (**not**) |
|---|---|---|
| module | `src/cross_marker_perception.py:797` | `src/img_data.py:186` |
| data dir | `calibration_data/output_cross/`, `landing_cal_cross/` | `calibration_data/output/` |
| derive tool | `derive_cross_marker_cal.py` / `derive_cross_marker_landing_cal.py` | `derive_board_cal.py` + `derive_ring_cal.py` |
| state | **"2026-08-28 RECAL for 320x240/fx=135"**, from GT-FB landing recordings | rows all tagged **"obs-rebuild recal 2026-07-17"**, derived at 640×480/fx=270 |

Verification that settled it (redo these three if ever in doubt, don't re-read comments alone):
newest run in `calibration_data/output/` is **Aug 9** (18 days before the 08-27 camera change);
`git log -- src/img_data.py` has **no commits** since `e31fbfd5` (the camera drop itself); every
`_sensor_cal_hw` row comment says 2026-07-17.

**⛔ USER DECLINED the ArUco recal (2026-09-03). Do not re-raise it as an open task.** ArUco is
comparison-only under [[feedback_aruco_perception_scope]], so nothing in the live cross-marker /
rover thread touches the stale cal. It matters only if ArUco comparison numbers are ever used for
the paper — those would be silently wrong. Stamped next to the existing warning in `CLAUDE.md`
(commit `3a6f3a0a`) so it reads as accepted, not as an oversight.

**If an ArUco recal is ever actually wanted:** don't just re-record 5 phased runs at defaults. The
cross-marker cal hit this same resolution and got **negative R² on every axis** from phased
excitation, which is why it moved to landing recordings. Reading the code, the recorded reason
("at 320×240 PX4's position loop tracks only ~15% of the sinusoid") conflates two effects:
`CALIB_AMP_XY=0.35` is in METRES, so resolution does not change commanded motion — the ~15%
tracking shortfall is a 0.5 Hz bandwidth problem present at ANY resolution, and what actually
changed is that fx 270→135 halved the pixel displacement, pushing an already-weak signal under the
noise floor. The untested-but-cheap lever is therefore `CALIB_FREQ_HZ` 0.5→~0.2 (env-overridable;
~6× more real motion, and LOWER peak tilt since tilt ~ A·ω², so the marker stays framed better),
with `CALIB_PHASE_S` 8→~20 to keep ~4 cycles/phase. `CALIB_AMP_XY` is hardcoded, not
env-overridable, and raising it would not help anyway — hfov is unchanged, so the same tilt/FoV
limit that forced 0.7→0.35 still applies. Record with `FLOW_LAT_REDUCED=1` (the live default) or
the `h_x/h_y` rows break ~3×, and re-run `derive_ring_cal.py` after any `_sensor_cal_hw` change
(the transfer ring cal is keyed to it). Full procedure: the `io-calibration` skill.
