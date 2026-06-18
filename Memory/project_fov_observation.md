---
name: FoV observation and assumption
description: Strict FoV-failure semantics added 2026-04-19; rho_fov_0=[150;110] inset achieves 49/50 prec+soft; Static IC5 realistic remains the lone breach
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
IC5 (`p0 = [2, 2, -3]` m — 3 m altitude, 2 m horizontal offset) provokes an aggressive initial camera tilt. Under perspective projection through a tilted camera, the **physical** corner pixels spread outward and briefly leave the v-axis sensor bound (±120 px). The **virtual** features (de-rotated to a nadir camera, `P_DS[:, 0:4]` 0-indexed) stay well inside ±160/±120.

**Strict FoV-failure policy (added 2026-04-19, commit pending):**
Any |C_nP(1,:)| > res(1)/2 or |C_nP(2,:)| > res(2)/2 inside the ZOH block (after noise injection) terminates the run with `success=false`, `precise=false`, `soft=false`, plus `fov_fail=true` and `fov_fail_t` set. Implemented in:
- `MATLAB/Multi_init_cond/run_simulation.m`
- `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m`
- `MATLAB/Comparison/visualControl_comparison.m`
Result struct + multi_Init_Var/multi_speed_cond template carry `fov_fail`/`fov_fail_t` fields.

**Cone-inset retune history (rho_fov_0):**
| Setting           | prec+soft | fov_fail | Notes |
|-------------------|----------:|---------:|-------|
| `[160; 120]` orig |     43/50 |     7/50 | All 7 fails were IC5 v-axis breaches across the 5 trajectories |
| `[120; 90]` (B)   |     41/50 |        — | Over-tightened; IC2-IC4 chase under-actuation, Linear IC3 blow-up |
| `[150; 110]`      |     49/50 |     1/50 | Only Static IC5 realistic still breached |
| **`[145; 105]`**  | **50/50** | **0/50** | **LOCKED 2026-04-19** — 15 px inset per side; max_xy = 5.71 cm, max_vrel = 0.165 m/s |

**Why [145;105] closes Static IC5 realistic:** the cone-clamp's `d_min` saturates to 0 earliest on Static (by t≈0.3s vs t≈0.4s on moving cases), because Static has no target-velocity feedforward and the 33° geometric-tilt regulation transient peaks unattenuated. A 5-px-tighter `rho_fov_0` freezes the desired tilt slightly sooner, preventing the physical-projection overshoot from crossing the 120-px sensor limit under NOISE+GE+delay.

**Plot column convention (do not confuse):**
- `P_DS[:, 0:4, :]` (Python 0-indexed) — **virtual** features (Approach 2 cone clamp operates on these via `rho_fov(t)`)
- `P_DS[:, 8:12, :]` (Python 0-indexed) — **physical** features (what `make_multi_init_plots.py` plots and what the user sees on the image-plane figures)
- MATLAB 1-indexed equivalents: `P_DS(:, 1:4, :)` virtual, `P_DS(:, 9:12, :)` physical

**Why Approach 2 alone does not eliminate physical breach:** the FoV-adaptive cone clamp bounds virtual features (where the funnel is defined). The `[150; 110]` inset on `rho_fov_0` indirectly tightens the cone-cap angle (`atan(d_min_fov / f)`) and reins in the initial tilt enough to keep the physical corners inside ±120 px on 49/50 — but it cannot eliminate the kinematic projection artifact when the geometry is already adverse and disturbances pile on.

**Manuscript caveat (results.tex, after Fig. 9 reference):** Update to reflect the strict policy + the [150;110] retune + the 1/50 residual rather than removing it. The caveat now also justifies the ≥90° FoV camera assumption with the residual Static-IC5-realistic case.

**Why:** Reviewer would flag features leaving physical FoV. Pre-empt by acknowledging the residual case + how Approach 2 reduces it from 7/50 to 1/50, and how a wider-FoV camera would absorb the rest.

**How to apply:**
- LOCKED gain: `rho_fov_0 = [145; 105]` (15 px per side inset from sensor-half [160;120]); other Approach 2 fields unchanged (`rho_fov_inf=[40;40]`, `l_fov=0.1`, `theta_cap=60°`).
- Sync across the 3 gain files (PLASMC gain-sync hook fires on every edit).
- The `fov_fail` field is now part of all result structs — downstream plotters/analysis MUST treat fov_fail==true runs as failed (success/precise/soft already gated).
- Same physical-vs-virtual breach pattern occurs on ALL trajectories' IC5 (not Static-specific) — the cause is initial tilt, not the trajectory.
- Do NOT remove the FoV caveat from results.tex — verify with the [150;110] tally before claiming "no FoV violations."
