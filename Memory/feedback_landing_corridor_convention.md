---
name: 3D landing-corridor visualization (replaces single dashed target line)
description: All 3D plot scripts now draw a shaded "soft-precise allowable landing region" (a 3D corridor) around each target trajectory instead of a single dashed line. Dimensions xy ±0.08 m perpendicular to target tangent, vertical extent 0..z_f=0.20 m above true target altitude (follows heave). Caption phrasing "shaded box: soft-precise allowable landing region around the target trajectory".
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The rule (locked 2026-04-26):**

In every 3D plot script that draws the target trajectory, replace the single line with a 3D rectangular corridor that visualizes the soft-precise allowable landing region:
- **horizontal**: ±0.08 m perpendicular to the target's xy-tangent (a ribbon following the trajectory)
- **vertical**: from the true target altitude (`-target_z` in display, NED) up to the lifted target altitude (`-target_z + z_f`) where `z_f = 0.20 m` is the harness above-target termination gap; the corridor follows target heave on Linear/Circular cases.
- **Static target**: corridor degenerates to a vertical square prism `0.16 × 0.16 × 0.20` m centered on the target point.

The corridor's TOP face coincides with where soft-precise UAV touchdowns sit (UAV terminates at z_f above target). Triangle markers should land on or inside the top face.

**Helper function** in each script: `draw_landing_corridor(ax, xt, yt, zt, half_xy=0.08, z_height=0.20, ...)`. Faces filled at α≈0.18; 4 corner outlines drawn dashed black; takes NED inputs and converts to display altitude internally.

**Caption phrasing (paired with this corridor):**
- Main paper Fig. 3 (first introduction): "the shaded box around the target trajectory is the soft-precise allowable landing region"
- Other captions (multi-speed, comparison 3D, supplement Cases 1–4): "Shaded box: soft-precise allowable landing region around the target trajectory"
- DO NOT spell out the dimensions (`xy ±0.08`, `z_f=0.20`) in any caption — soft-precise is already defined in §I, and the dimensions are visible in the figure.

**Why:** The earlier "silent +0.20 m target lift" approach left soft-precise UAV markers floating below the lifted target line because the lift was not paired with a tolerance visualization. The corridor explicitly visualizes what "soft-precise" means in 3D — UAV inside corridor = soft-precise, outside = miss. Markers are now on the top face of the corridor when soft-precise.

**How to apply:**
- When adding a new 3D plot script, copy the `draw_landing_corridor()` helper from `make_multi_init_plots.py` verbatim.
- Old `TARGET_LIFT_M = 0.2` constant is gone; the lift is implicit in the corridor's top face.
- Multi-speed plots draw 5 corridors (one per λ) at α=0.10 each so they remain readable when stacked.
- Triangle/cross marker convention is unchanged: see `feedback_landing_marker_convention.md`.

**Affected scripts (2026-04-26):**
- `scripts/make_multi_init_plots.py`
- `scripts/make_multi_speed_plots.py`
- `scripts/make_comparison_plots.py`
- `scripts/make_comparison_multi_speed_plots.py` (added 2026-04-26 with the 80-run baseline-controller speed sweep)

**Affected captions (2026-04-26):**
- `results.tex` Fig. 3 (Circular Case 5), Fig. 4 (multi-speed), Fig. 6 (comparison 3D)
- `supplemental.tex` Cases 1–4 3D figure captions
