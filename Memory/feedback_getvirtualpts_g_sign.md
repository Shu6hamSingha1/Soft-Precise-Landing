---
name: getvirtualpts-g-sign-bug
description: "img_data.py:_getVirtualPts had R @ [0,0,1] instead of R.T @ [0,0,1]; V-frame projection amplified tilt-induced offset instead of cancelling it"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## The bug

`img_data.py:_getVirtualPts` (pre-2026-06-01) computed:

```python
R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
g = R @ np.array([0, 0, 1])     # ← comment claimed "world-down in camera frame"
z_axis = g / np.linalg.norm(g)
x_axis = np.cross([0, 1, 0], z_axis)  ...
```

But `ahrs.Quaternion.to_DCM()` returns the **body→NED** rotation (when applied to a PX4 quaternion). So `R @ (0,0,1)_body = body_z_in_NED`. The comment claimed it was world-down-in-body, which is `R.T @ (0,0,1)_NED`. These differ.

**The bug is invisible for level drone** (body z = world-down = (0,0,1) regardless), so all pre-existing tests on hovering recordings passed. The bug **only manifests when the drone tilts** — and then the V-frame projection **amplifies** the tilt-induced apparent marker offset instead of canceling it.

## Numerical signature

For drone hovering directly above marker (V-frame `s` should be `(0, 0)`):

| Drone pose | Buggy V-frame s | Fixed V-frame s |
|---|---|---|
| Level (any yaw) | (0, 0) | (0, 0) |
| Pitch +10° | **(−0.364, 0)** | (0, 0) |
| Roll +10° | (0, +0.364) | (0, 0) |
| Yaw 45° + Pitch 10° | (−0.310, −0.124) | (0, 0) |

The spurious offset is roughly `tan(2 · tilt_angle)`.

## Why yaw-phase exposed it

During phased yaw calibration the drone is commanded to yaw ±15° while holding XY position. PID translates this into 5–10° pitch/roll corrections to fight position drift. Those tilts feed the buggy `g`, producing oscillating spurious `s` that GT (computed correctly in cell 6 of `plotter_output_calibration.ipynb` as `V_x_NED · W_x_tu / V_z`) doesn't see. Symptom: yaw-phase corr(GT_s, cal_s) drops to **+0.21 / +0.02** while x/y/z phase corr stay at **+0.92**, and cal_s RMS is ~2.5× GT_s RMS only during yaw.

## The fix

Change `g = R @ np.array([0, 0, 1])` → `g = R.T @ np.array([0, 0, 1])`. Applied 2026-06-01.

## Downstream cal invalidation

`_getVirtualPts` feeds both:
- `_getImgFeatures` (the centroid `s` → `sensor_cal_s`)
- `_fill_A` (corner positions inside the L matrix → `sensor_cal_hw` for `h`/`w`)

So the L matrix evaluation has been carrying the same tilt-amplified `(x, y)` entries the whole time. `sensor_cal_hw` was implicitly absorbing the bias — most strongly on `w_x`/`w_y` axes (roll/pitch columns of L depend most sensitively on `(x, y)`), less on `w_z` (corner-spread-dominated). Both `sensor_cal_hw` and `sensor_cal_s` need re-derivation from new post-fix recordings.

## Diagnostic protocol

When verifying any "level frame" projection that uses quaternions:
1. Test at level + several non-zero tilts; the V-frame output of a stationary marker directly below the drone should be `(0, 0)` ± noise for ALL of them.
2. If the V-frame s grows with tilt magnitude, the rotation direction in the world-down computation is wrong.
3. Distinguish AHRS-library quaternion convention from PX4 convention before assuming `R @ v_body = v_NED` vs the transpose.

## Related

- [[image-center-bug]] — the cell-38 transposition fix was real; the `img_data.py` "fix" of `[::-1]` was bogus. Don't confuse this `_getVirtualPts` bug with that one — they are independent and only this one is in img_data.py.
- [[project-camera-calibration-status]] — sensor_cal values are again stale.
