---
name: reference_alpha0_vs_alpharef
description: "alpha_0 vs alpha_ref — two different layers in the yaw-feature chain. alpha_0 (CROSS_ALPHA_0 / _moment_alpha_0, in the perception module) is a fixed CALIBRATION offset subtracted from the raw 2nd-moment principal angle so that `alpha` means 'vehicle yaw relative to the marker, 0 = aligned'. alpha_ref (_s_d[3], in the controller) is the CONTROL SETPOINT — the alpha you want the drone to hold; default 0, settable via DES_ALPHA. Chain: raw_angle -[- alpha_0 cal]-> alpha -[- alpha_ref setpoint]-> e_a -> yaw law."
metadata:
  node_type: memory
  type: reference
---

## The chain
```
raw 2nd-moment principal angle
   --[ - alpha_0  (perception calibration) ]-->  alpha  (= s[3])
   --[ - alpha_ref (control setpoint)      ]-->  e_a   --> yaw controller drives e_a -> 0
```

## `alpha_0` — perception calibration offset
- `cross_marker_perception.py`: `self._alpha_0` = `CROSS_ALPHA_0`, default `radians(0.58)`
  (re-derived 2026-08-31, `derive_cross_alpha0.py`, phased flights).
  `alpha = wrap(_unweighted_principal_angle(...) - self._alpha_0)` (line ~2788).
- `img_data.py` (ArUco): `self._moment_alpha_0` = `MOMENT_ALPHA0`, default `-2.533`.
- WHAT it absorbs: the fixed geometric mismatch between "raw principal angle in pixels" and
  "vehicle yaw relative to the marker, zero at aligned hover" — camera-mount yaw convention +
  marker/stub SDF orientation + the `cv2.ROTATE_90_CW` working-frame rotation.
- Property of the SENSOR + SCENE. Derived once from calibration data; never changes at runtime.
- If mis-calibrated by δ: `alpha=0` no longer means aligned → driving `e_a->0` (with alpha_ref=0)
  leaves the drone physically yawed by δ. (This was RULED OUT as the cause of the ~15° stationary
  e_a residual — the residual is in `e_a` itself and is IC-direction-dependent, neither of which
  a constant offset produces. See [[project-yaw-rate-law-sign-bug-and-validation]].)

## `alpha_ref` — the control setpoint
- `controller.py:3148`: `e_a_raw = _alpha - self._s_d[3]`, then wrapped. `_s_d[3]` = the desired
  feature vector's alpha component = `alpha_ref`.
- The yaw you WANT the drone to hold relative to the marker. Default **0** (square-aligned
  landing); settable via `DES_ALPHA` (tested at 0/10/30/45°).
- A CONTROL choice, applied downstream of perception. Independent of `alpha_0`.

## Not the same as
- `_sensor_cal_s[3]` — kept IDENTITY by design (the yaw chain is built on raw moment-alpha:
  offset `alpha_0` upstream + gain `BODY_YAW_ALPHA_K` downstream; setting `s[3]!=1` double-counts).
- `BODY_YAW_ALPHA_K` (default -1.0 cross / -0.949 ArUco) — the alpha->yaw_c GAIN/sign map used
  when `BODY_YAW_SOURCE=alpha` rebuilds the measured attitude for `R_d` (see
  [[project-yaw-rate-law-sign-bug-and-validation]] FINAL STATE).
