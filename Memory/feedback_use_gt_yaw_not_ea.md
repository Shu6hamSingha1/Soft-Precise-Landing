---
name: feedback_use_gt_yaw_not_ea
description: "METHODOLOGY (user, 2026-06-08): evaluate yaw OUTCOME with GROUND-TRUTH yaw, NOT e_a, for the ENTIRE landing (not just terminal). e_a (=alpha-alpha_d, the controller INPUT) is unreliable both ways: it OVER-reports (marker-fills-FoV corrupts alpha -> e_a jumps to ±150-180° while the drone is HELD) AND UNDER-reports (a real GT yaw divergence reads small). Tool: tools/gt_yaw_analysis.py."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

**Rule (user, 2026-06-08): use GT yaw, not `e_a`, for yaw analysis across the WHOLE landing test — not just the terminal phase.**

`e_a` (= alpha − alpha_d, computed in `_yawCtrl` from the image feature `s[3]`) is the controller's **INPUT**, not the **outcome**. It is unreliable as an outcome metric in BOTH directions:
- **OVER-reports**: when the marker fills the FoV near touchdown (or alpha glitches), `s[3]` corrupts → `e_a` jumps to ±150–180° **while the drone is physically HELD** (GT yaw swing ~8°). Many past yaw-tuning "terminal spins" (124/179/180°) were this artifact — the drone did NOT spin (verified via GT yaw: GT END +1° vs e_a +155°). The measured-yaw-hold fix (`yaw_c` frozen during `_yaw_hold`, commit 704e577) stops corrupted alpha *driving* the drone, but `e_a` is still logged from live `s[3]`, so it still *reads* corrupted.
- **UNDER-reports**: a genuine GT yaw divergence can read small in `e_a` (e.g. YawHoldFix rep2: GT END **−86°** real divergence, but `e_a` said only +66°).

**Consequence — past yaw-tuning conclusions are confounded** and must be re-judged on GT yaw:
- "`YAW_GAMMA=1.0` worse (94°→124°)" and "`KR_YAW`↓ worse (−179° spin)" were reading terminal `e_a` corruption with the drone actually HELD → likely FALSE dead-ends → **re-test on GT yaw** when yaw work resumes.
- On GT yaw the yaw control is actually performing *better* than `e_a` implied: it converges large initial yaws (71/69/80°) to ~4–20° GT residual — not the 58–70° `e_a` suggested.

**Tool: `tools/gt_yaw_analysis.py <rep...>`** — derives a board-square GT-yaw reference from clean windows (|e_a|<15° AND s_e_n<0.5 AND non-terminal) across the passed reps, then reports per-rep `GTyaw_err start→min/max→END, SS(last 1s)` and flags REAL divergences (|END|>45°). Pass several reps from the same world for a stable board reference. See [[feedback_clamps_during_tuning]], [[feedback_matlab_yaw_square_start]], [[moment-yaw-canonical]].
