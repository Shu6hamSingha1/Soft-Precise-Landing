---
name: P_DS column convention — virtual vs physical features
description: P_DS columns 1-4 (MATLAB) / 0-3 (Python) are virtual features; cols 9-12 / 8-11 are physical. Plots and FoV checks use PHYSICAL.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
The `data.P_DS` log array has multiple feature blocks. Confusing them silently produces wrong FoV/visibility conclusions.

| Python slice    | MATLAB slice    | Block               | Where it shows up                                   |
|-----------------|-----------------|---------------------|-----------------------------------------------------|
| `P_DS[:, 0:4]`  | `P_DS(:, 1:4)`  | **Virtual** corners | Controller input (de-rotated, nadir camera). What the cone clamp / funnel `rho_fov(t)` constrains. |
| `P_DS[:, 8:12]` | `P_DS(:, 9:12)` | **Physical** corners| What the camera actually sees. What `make_multi_init_plots.py` plots. What FoV checks must use.    |

**Why:** Got it backwards in the 2026-04-19 audit — used `P_DS[:, 0:4]` to argue "no FoV violations" → recommended removing the manuscript's FoV caveat. User caught it: IC5 physical corners on Static reach |v|≈134 px (limit ±120). The funnel does bound the virtual features, but physical pixels aren't directly clipped — they follow from perspective projection through the (tilted) camera pose. So aggressive ICs (low altitude + lateral offset, e.g. IC5 `[2,2,-3]`) cause brief physical-FoV breaches even with Approach 2 active.

**How to apply:**
- For "is the controller bounded?" questions → check virtual (`0:4` Py / `1:4` MATLAB) against `rho_fov_log`.
- For "did the camera lose the target?" / FoV questions → check physical (`8:12` Py / `9:12` MATLAB) against `±res/2` from `Constants.m` (`±160 u`, `±120 v`).
- The image-plane figures in the manuscript show physical features, so anything the user sees on those plots is physical.
- Before recommending removal of the manuscript's FoV caveat, verify physical max |u|, |v| stays inside ±160/±120 across **all** trajectories × ICs (not just one).
