---
name: image-center-bug
description: "The center bug was in plotter_output_calibration.ipynb cell 38, NOT img_data.py — verify the offset against drone-marker geometry before touching runtime code"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## The real bug and the false alarm (2026-06-01)

There were **two** "center" values in the codebase. Only one was wrong:

| Location | Pre-2026-06-01 | Status | Action |
|---|---|---|---|
| `plotter_output_calibration.ipynb` cell 38 | `center = np.array([320.0, 240.0])` | **REAL bug** (transposed) | Fixed to `[240.0, 320.0]` |
| `img_data.py:62` | `self.center = np.array(self._resolution)/2` | **NOT a bug** — was always correct | Left alone (was briefly broken-then-reverted) |

### Why `img_data.py` was always correct

`gz_subscriber.py:154` stores `_res = (msg.height, msg.width)` from the ORIGINAL (pre-cv2-rotation) image. For the 640×480 sensor:

```
msg.height = 480, msg.width = 640
_resolution = (480, 640)
```

After `cv2.ROTATE_90_CW`, the image becomes **480 wide × 640 tall**. cv2.aruco returns corners as `(x=col, y=row)`, so the principal point is `(cx=W/2, cy=H/2) = (240, 320)`.

```python
self.center = np.array(self._resolution) / 2
            = np.array([480, 640]) / 2
            = [240, 320]
            = (cx, cy)              ✓ CORRECT
```

It LOOKS like a height-first ordering, but because `cv2.ROTATE_90_CW` swaps the two dimensions, `_resolution[0]` and `_resolution[1]` happen to equal post-rotation `W` and `H` respectively. The "obvious" `[::-1]` "fix" actually breaks it.

### How the cell-38 bug was traced (and why it was conflated with img_data.py)

Symptoms on 5m phased recordings (`Sun May 31 21-38-25 2026`):
- plotter cell 38 LHS-RHS yaw rel err = **286%**
- z rel err = 118%
- marker centroid empirically appeared at normalized `(-0.27, +0.30)` in cell 38's analysis

Tracing led to "the center is wrong somewhere." The first hypothesis was that `img_data.py:62` was transposed, but in fact:
- Cell 38 used `(320, 240)` directly, while img_data.py used `(240, 320)` via the `_resolution/2` expression.
- Both made the marker "look offset by `(-0.27, +0.30)`" — but each from a different baseline. They APPEAR symmetric so it's easy to assume they share a bug.

Fixing cell 38 alone dropped the LHS-RHS err from 286% → 39% (the real fix). Subsequently "fixing" `img_data.py` to use `[::-1]` then made the runtime use `(320, 240)` and the next 9 recordings (Mon Jun 1 02:26–02:51) were genuinely centered wrong. Those recordings now live in `calibration_data/output_archive_bogus_centerfix_20260601/` and are NOT used for cal derivation.

### Diagnostic protocol before touching `self.center` again

1. **Sample raw corner pixels** from a recording: `np.array([np.asarray(p[1]) for p in img['Image Feature Pts'][100:600]])`
2. Verify `corner.shape[2] == 2` and `corner[:,:,0].max()` fits the post-cv2 width (480), `corner[:,:,1].max()` fits the post-cv2 height (640).
3. For drone hovering above marker, **corner col 0 mean** should be `≈ 240` (cx) and **col 1 mean** should be `≈ 320` (cy).
4. If `(corner_mean - center) / fx` is `≈ 0` for the correct center, but `~0.30` for the wrong one — that locates the bug. Confirm against `drone_marker_horizontal_offset / altitude` (should be `< 0.02` for a hovering drone).

### Why baseline_3m masked the cell-38 bug

The cell-38 bug had the same `±0.30` constant-bias contribution to RHS at every recording, but:
1. **Vigorous combined excitation → large |LHS|.** Baseline_3m had |LHS| ≈ 0.76 — the 0.15 bias is ~20% of that, visually still tracks. 5m phased yaw has |LHS| = 0.05, same 0.15 bias becomes 286%.
2. **Lower altitude widens corner spread.** At 3m the corners cover ~1.7× more normalized area; v_z column `[-x, -y]` and ω_z column `[-y, +x]` keep per-corner variance even with the constant offset. At 5m the corners cluster tightly around any offset and these columns become collinear (LᵀL[v_z, v_x] = +1.22 with the buggy offset).

Empirical: running the OLD buggy cell-38 config on baseline_3m gives 95% rel err; FIXED gives 81%. Both mediocre, but neither the 286% catastrophe seen in 5m yaw. The phased single-axis-at-a-time excitation at higher altitude **exposed** the long-standing cell-38 bug.

### Diagnostic lesson

When a calibration "works" visually under combined high-amplitude excitation but breaks under low-amplitude phased excitation, suspect a *constant-magnitude bias* (DC offset, frame transposition, sign convention error). The high-amplitude case hides it as a small fraction; phased single-axis isolates it.

When two pieces of code have a similar-looking parameter, **don't assume they share the bug**. Verify each one independently against ground truth (drone-marker geometry) before changing both.
