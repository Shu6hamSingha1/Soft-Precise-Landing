---
name: project_20260922_lissajous_cbf_and_divergence_mechanism
description: "⛔ CORRECTED 2026-09-22 by [[project_20260922_ackermann_rover_loops_not_tracking]]: the k=0.1 SP rep landed on a rover PARKED for its final 6.5 s (the Ackermann rover parks/loops at ~2 m/s, never tracks 0.15 m/s) -- NOT Lissajous tracking; speed-retune conclusions describe the command only. ORIGINAL: ⭐ RESOLVED 2026-09-22 same day: k=0.1 (w1=-0.05,w2=0.0475, ~0.14 m/s) achieves genuine SOFT+PRECISE (xy=0.013m, rel_vel=0.120m/s, clears even the manuscript's strict thresholds), live default. 2026-09-22: Lissajous moving-target investigation on rover_cross, GT-FB. A stale Aug-24 montage (rover_aruco, SOFT+PRECISE) looked like a contradiction but rover_aruco/rover_cross share the SAME airframe/steering (4022_gz_rover_aruco -> 4012_gz_rover_ackermann) so vehicle physics isn't the explanation; user correctly identified the CBF as the one thing GT-FB does NOT isolate (marker_center_px is live-perception-derived, confirmed in controller.py/visibility_projection.py -- passthrough only when None). Cross-rep correlation: vis_active engagement tracks failure severity (63/51/27% on 3 bad reps vs 3.4% on the 1 good one at k=0.4). But within-rep trace at k=0.2 shows the DIVERGENCE TRIGGER at t~2.7s is NOT CBF (vis_active=0 through onset) -- it's h_d_z pinned at -0.30 regardless of lateral error while I_a_z gets cannibalized by growing I_a_xy, the SAME thrust-sacrifice mechanism as project_20260922_rover_moving_sp_investigation's original finding. Speed retunes (k=0.4->0.2->0.1) delay but do not prevent this; 0/4 SP at k=0.2 (5f's batch) confirms not a clean speed fix. CONCLUSION: trajectory-level retuning (this whole thread) is very likely the wrong lever -- the fix needs to be in controller.py (gate descent on lateral convergence / protect I_a_z), untried. Read the RESOLVED section at the bottom first -- the mechanism finding stands but Lissajous itself is solved (slow enough = no trigger), don't re-open it."
metadata:
  type: project
---

> ⛔ **CORRECTION (2026-09-22):** see [[project_20260922_ackermann_rover_loops_not_tracking]] — GT target speed in every Lissajous_final rep peaks 1.6-2.8 m/s (parked/looping), so "going slow enough" never happened on the target; the RESOLVED rep landed on a parked rover.

## Chronology (same day, several follow-ups on top of [[project_20260922_rover_moving_sp_investigation]])

**1. 5f found + partially fixed a geometric cusp.** The literal MATLAB Lissajous spec
(`A=0.4,B=0.8,w1=-0.5,w2=0.85`, no phase) has vx/vy hitting zero near-simultaneously --
curvature radius ~0.0006 m at that instant, independent of amplitude (confirmed: curvature
radius is scale-invariant under uniformly rescaling w1/w2 by a common factor -- verified
numerically both by 5f and independently by me). A 104 deg phase offset between axes breaks
the coincidence; amplitude then scaled 4x (`A=1.6,B=3.2`) for a ~0.85 m margin over the
rover's ~0.56 m physical min turn radius. Curvature radius scales roughly LINEARLY with
amplitude alone (verified: kA=1->R=0.113m, kA=4->R=0.450m, exactly 4x) -- so the 4x
amplitude was not optional, it's what the curvature margin requires at this shape.

**2. A stale Aug-24 montage (`montage_lissajous.mp4`) looked like a contradiction and
turned out not to be, but for a subtler reason than first assumed.** Source log
(`test_data/Rover_AB_harness/montage_lissajous_1.out`, dated Aug 7) shows the SAME
never-modified Lissajous formula landing SOFT+PRECISE (xy=0.009m) in one attempt -- but on
`WORLD=rover` (`rover_aruco`), not today's `rover_cross`. First hypothesis (mine): different
vehicle physics. WRONG, checked and retracted: both `rover_aruco`'s airframe 4022
(`4022_gz_rover_aruco`) and rover_cross use the SAME base steering config
(`4012_gz_rover_ackermann`) -- confirmed via ROMFS, no separate airframe file for rover_cross.
**User correctly identified the real candidate: the CBF.** GT-FB isolates the main control
law's image features (s/h) but NOT the CBF's input -- `marker_center_px`/`cbf_corners` is
still live-perception-derived (`controller.py` ~4037; `visibility_projection.py`'s own
docstring: "`marker_center_px`: ... or None -> passthrough"). The CBF pipeline itself
changed enormously between Aug 7 and today (axis-transposition bug present+silent until
09-17, full two-tier visibility rewrite 09-09) -- a well-evidenced remaining difference,
unlike vehicle physics.

**3. Sept 9 evidence (closer in time, same rover_cross, same un-modified Lissajous
formula) directly contradicts the Aug 7 montage and is the better comparison.**
`test_data/RoverCBFSweep/20260909-163929/Lissajous/off/rep{1,2}`: both NOT_LANDED
(xy 2.4-3.0m, min 1.5-1.6m above surface) -- and CBF WAS engaged non-trivially
(`vis_active` 9.9-11.2% of frames, `az_joint_delta` max 11.7-18.3). Confirms rover_cross
genuinely cannot do this un-modified shape (unlike rover_aruco once, apparently), and the
CBF is active during the failure, not inert.

**4. Speed retune saga (my own edits to `rover_trajectory.py`'s Lissajous block,
env-tunable `ROVER_LISS_{A,B,W1,W2,PHI_DEG}`):**
- k=1.0 (w1=-0.5,w2=0.475, the cusp-fixed shape as 5f left it): median speed 1.21 m/s (2-3x
  the ~0.4-0.6 m/s envelope everything else in this session's sweeps landed at). Live: 7/7
  FAIL (`test_data/RecordGTFB_dev/Lissajous_slow`), one `a_u_xy` hit 1e5.
- k=0.4 (w1=-0.2,w2=0.19): median ~0.52 m/s. Live 4 reps
  (`test_data/RecordGTFB_dev/Lissajous_final` reps 1-4): 3/4 badly FAIL (xy 2.4-6.0m), 1/4
  close (xy=0.36m, rel_vel=0.363, still over the harness's 0.15m/0.5m/s thresholds).
  **Cross-rep check: NOT excursion-driven** (the successful rep had the LARGEST target
  excursion, 2.91m, of all four -- rules out "sweep too wide" as the sole driver).
  **Correlates with `vis_active`** (CBF engagement fraction): 63%/51%/27% on the three
  failures vs 3.4% on the success; `a_u_xy` peaked 90-94k on the high-vis_active reps vs
  20k on the low one.
- k=0.2 (w1=-0.1,w2=0.095, user-requested further reduction): median ~0.26 m/s (close to
  CircularYaw's ~0.26-0.28 m/s, the slowest profile tested this session). Live: MY test
  (1 rep, separate dir) FAILED (xy=3.09m, rel_vel=7.13m/s). 5f's batch (4 reps, same dir
  reps 5-8): still 0/4 SP, xy=[1.90,1.71,0.21,3.45]m -- **not a clean improvement over
  k=0.4 on 3/4 runs**, matching the vis_active finding (speed isn't the whole story).
  Combined across both sessions: **0/5 SP at k=0.2.**
- k=0.1 (w1=-0.05,w2=0.0475): further reduced on-disk (in-file comment attributes this to
  a user request) to median ~0.14 m/s. **NOT YET SITL-validated as of this entry.**

**5. The key mechanistic finding -- within-rep trace, my k=0.2 test rep
(`test_data/RecordGTFB_dev/Lissajous_slow` sibling dir, saved separately as
`liss_k02_test`; scratch-only, see below): the CBF is NOT what triggers the divergence,
even though it correlates with severity ACROSS reps.** Timeline:
- t=0.00-2.55s: excellent tracking (xy_err drops to 0.032m, the best of ANY Lissajous rep
  this session at any speed), `vis_active=0` throughout.
- t=2.70s: `h_xy` (measured optic flow) jumps 0.02->0.14 (7x) with NO CBF engagement
  (`vis_active` still 0) and nothing distinctive in the commanded target kinematics at that
  instant (checked: heading changes smoothly, <=12deg/s, no reversal near t=2.7s at this
  parameter set).
- t=2.70s onward: error grows smoothly and continuously (not a spike-recover) --
  `I_a_xy` climbs 0.21->7.05, while **`I_a_z` gets cannibalized 9.64->4.01** (lateral
  demand eating vertical thrust) and **`h_d_z` (commanded descent optic flow) stays PINNED
  at -0.30 throughout, never adapting to the growing lateral error** -- descent is not
  gated on lateral convergence, exactly the [[project_20260922_rover_moving_sp_investigation]]
  mechanism. `vis_active` only turns on at t=4.65s+, well AFTER the divergence is already
  established -- a SYMPTOM of the drone being far off and low, not the trigger.
- `a_u_xy` explodes to 549 by t=5.25s (terminal blow-up), landing FAIL.

**Synthesis: slowing the trajectory delays the divergence (bought ~2.7s of clean tracking
vs instant failure at k=1.0) but does not remove it.** There appears to be an implicit
stability threshold in the lateral-tracking + descent loop: below it, tracks fine; above
it (however reached -- speed, cumulative drift, CBF interference), runs away via the same
thrust-cannibalization mechanism regardless of trajectory shape. This matches every other
profile's "close but not soft" or outright-fail pattern in the main investigation.

## Conclusion / how to apply

**Trajectory-level retuning (amplitude, phase, frequency/speed -- this entire thread) is
very likely the wrong lever going forward.** It can shift WHEN the divergence starts, not
whether it happens. The two real candidate fixes both live in `controller.py`, are both
still untried, and were already flagged (not yet acted on) at the end of
[[project_20260922_rover_moving_sp_investigation]]:
1. Gate/reduce commanded descent rate (`h_d_z` / `h_rd`) on lateral convergence, instead of
   holding it constant regardless of lateral error.
2. Protect `I_a_z` from being cannibalized by growing lateral (`I_a_xy`) demand.

The CBF-engagement correlation across reps is real and worth keeping in mind (a fast/wide
excursion likely triggers the CBF more, which may compound an already-diverging loop), but
is NOT the initiating mechanism -- don't chase a CBF-only fix (e.g. relaxing
`CBF_BUFFER_FRAC`) expecting it alone to close this out.

**Scratch tooling used (not committed):** ad-hoc trace scripts in the session scratchpad
(vis_active/a_u/h_d timeline dumps, curvature-radius numeric search, montage-source
cross-reference). Not promoted to `tools/` -- recreate if needed rather than searching for
them on disk.

## RESOLVED (2026-09-22, same day, final): k=0.1 achieves genuine SP

5f's follow-up: `k=0.1` (`w1=-0.05, w2=0.0475`, median ~0.14 m/s) landed **SOFT+PRECISE**:
`xy=0.0128m, rel_vel=0.120m/s` — clears even the manuscript's strict 0.08m/0.2m/s
thresholds, not just this harness's relaxed 0.15m/0.5m/s gate. Promoted to
`test_data/Final/Lissajous/`. Live default in `rover_trajectory.py` as of this entry.

**This tempers the "trajectory retuning is the wrong lever" conclusion above without
retracting the mechanism finding.** Going slow enough DID convert to a clean landing in
the end — so speed (combined with the phase/amplitude cusp fix) is a sufficient practical
fix for Lissajous specifically, even though the CBF-engagement correlation and the
within-rep thrust-cannibalization trigger (t~2.7s divergence at k=0.2) are both likely
still-real contributing mechanisms that a sufficiently slow trajectory simply never
triggers (never gets close enough to the FoV edge, never accumulates enough lateral error
to cross whatever implicit stability threshold exists). **Distinguish two claims going
forward:**
1. "Lissajous can be made to land" — YES, solved, k=0.1 is the answer, done.
2. "The underlying thrust-cannibalization / descent-not-gated-on-lateral-error mechanism
   is fixed" — NO, still open, still belongs in `controller.py`
   ([[project_20260922_rover_moving_sp_investigation]]). k=0.1's success is a workaround
   for ONE profile via brute-force slowness, not evidence the architectural issue is
   resolved — Circular/Sinusoidal/EightShape/CircularYaw's "precise but not soft" ceiling
   at their own already-fairly-slow ~0.4-0.6 m/s speeds suggests they are NOT simply a
   "go slower" fix away from SP; each would need its own speed floor found empirically
   (costly) or the controller-side fix (general, addresses all profiles at once).
