---
name: plotter-cal-unbaking-impossible
description: "Notebook KF/Savgol plots show recording-time cal as-is; un-baking a 6x6 cal from post-cal logs is ill-posed (caused \"GT not aligned\" 30-60x blowup)"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: d6332b67-4f1a-4377-8694-65dc8d3d45b4
---

The plotter notebook's cell 22 used to "infer recording-time cal and re-apply the cell-4 candidate cal" to the KF/Savgol logs. Removed 2026-06-02.

**Why:** Recordings made with the full 6×6 board cal baked into img_data.py break any un-baking attempt:
- Per-channel ratio `median(kf[k]/raw[k])` → ~0.013 on channels dominated by off-diagonal coupling (w_y ← h_x) → ÷0.013 = 30–60× amplitude blowup. This is what made GT look like "a flat line not aligned with the rest of the plots."
- Full 6×6 lstsq fit is also unreliable: it entangles the KF's channel/frequency-dependent gain with the cal; `M_new @ pinv(M_fit)` has entries ±10–40 even on well-conditioned multisine runs (cond≈109).

**How to apply:** KF/Savgol comparison plots (cells 22–28) show the runtime's actual output in whatever cal was recorded. To evaluate a candidate cal against GT, use cell 8 / cells 12–16, which apply the cal to the RAW (pre-cal) logs — the well-posed path. Never reintroduce post-cal un-baking.

Also note: GT vs image-side in these recordings is genuinely time-aligned — GT leads by the physical ~250 ms image-pipeline lag (corr 0.83–0.99). Apparent "misalignment" is almost always an amplitude/scaling artifact, not a time-base bug. Related: [[aggregate-calibration-methodology]], [[project_landing_target_design]].
