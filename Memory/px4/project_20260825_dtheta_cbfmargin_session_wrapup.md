---
name: project_20260825_dtheta_cbfmargin_session_wrapup
description: "Session wrap-up (2026-08-24 -> 2026-08-25, this thread): traced IC5 cross-marker/ArUco perception failures end-to-end -- dtheta self-defeating attitude-history loop (confirmed, unfixed), the uncapped-dtheta-correction fly-away (band-aid mitigated, not root-fixed), and a genuine perception FoV-margin gap (marker corners physically exit frame). Attempted fix (CBF_MARGIN_RESERVE) tested INCONCLUSIVE-to-negative in a clean sweep. Also: repeatedly, wrongly defaulted to ArUco instead of cross-marker (now corrected in memory). Next-session priorities listed at the bottom."
metadata:
  node_type: memory
  type: project
  modified: 2026-08-25
---

Wrap-up for this thread's IC5 investigation, written because the user is moving to a fresh
chat. Ties together [[project_20260824_dtheta_az_filter_self_defeating_feedback]],
[[project_20260824_dtheta_href_continuous_compensation]],
[[project_20260824_dtheta_ic5_flyaway_rootcause]], [[project_20260824_dtheta_cap_flyaway_fix]],
[[project_20260824_ic5_perception_fov_margin_gap]], and
[[project_20260825_cbf_margin_reserve_fix]] into one ordered summary + next steps. See
[[project_20260825_session_wrapup_touchdown_hardening]] for the OTHER concurrent session's
wrap-up (touchdown-detect/HW_FROZEN work) -- these are two separate, currently-unmerged
threads on the same codebase, both relevant to "what's pending for cross-marker."

## What's committed (safe, done)

- `727e73a`: `PLASMC_DTHETA_HREF` (continuous h_ref compensation + direct-term crossfade) and
  `PLASMC_DTHETA_AZ_CAP` in `controller.py`; `CBF_MARGIN_RESERVE` in `cbf_visibility.py` +
  `cbf_visibility_aruco.py`. All default-off/unchanged-behavior, all gated behind env vars,
  `validate_cbf.py` still 12/12. Plus the initial memory trail (self-defeating loop, fly-away
  root cause, FoV-margin gap, first CBF_MARGIN_RESERVE A/B).
- `a1a5adf`: re-stated the ArUco-comparison-only rule at the top of `px4/MEMORY.md` after
  repeatedly violating it this session (see below).

## Findings, in causal order

1. **`dtheta` (the az-visibility-filter correction) is self-defeating**: it fires to slow
   descent when the CBF is suppressing lateral authority, but the extra lift shifts realized
   attitude, which feeds back into `cbf2_filter`'s `th_curr` on later cycles, causing the QP to
   grant LESS lateral authority over time -- confirmed directly via the
   `theta_safe/theta_desired` ratio shrinking with gain. **Not fixed** -- the proposed structural
   fix (anchor `th_curr` against desired, not realized, attitude) was never implemented.
2. **IC5's `TARGET_LOST`/`ASCENDING` fly-away is a SEPARATE, independent bug**: `dtheta` has no
   cap on cumulative lift, and fires near-continuously (65-98% of frames) at IC5, so the
   correction compounds into a real multi-meter runaway climb + massive lateral divergence.
   **Confirmed NOT caused by the v3 h_ref work** -- present in the pure pre-v3 baseline too.
   Mitigated (not root-fixed) by `PLASMC_DTHETA_AZ_CAP` + `PLASMC_DTHETA_HREF` together (cap
   alone insufficient) -- this is explicitly a BAND-AID per user correction, masking the
   symptom (climbing) without addressing why `dtheta` runs chronically high.
3. **The residual lateral-only failure (once the fly-away is masked) traced to a REAL,
   different cause than first assumed**: not the old "kappa-leakage wall" (that's already
   fixed/baked since 2026-06-29, confirmed clean under GT-feedback on this exact IC/config).
   The real mechanism: real perception at IC5 (3m altitude, 2m/2m offset) has the marker's
   corners GENUINELY exit the camera frame (`KLT corners left image bounds`), something
   GT-feedback structurally cannot experience. Confirmed on BOTH marker types (not
   cross-marker-flicker-specific) and confirmed to be the SAME mechanism at IC2/IC3, just
   happening too late in those flights (near touchdown) to matter, vs. too early at IC5 (mid
   flight) to be benign.
4. **Attempted fix**: `CBF_MARGIN_RESERVE` -- Phase 1's FoV box bound was centroid-only by
   design (no proactive margin for the marker's own footprint, only reactive after decode
   already fails). Proactively reserving footprint margin in Phase 1 too is mechanistically
   sound and confirmed genuinely engaging (real trajectory divergence, not a no-op) -- but
   **two independent n=3 sweeps give no evidence it helps**: the first was contaminated by a
   concurrent SITL session (corrected after the user asked directly whether that had been
   checked); the CLEAN re-run reversed the direction entirely (reserve=0.0 1/3 TARGET_LOST vs
   reserve=1.0 2/3). **Not baked. Left at default 0.0.**

## Process mistakes this thread made, corrected in memory

- **Repeatedly defaulted to ArUco instead of cross-marker** despite an existing 2026-08-23
  directive that ArUco is comparison-only -- one legitimate one-off check (is this failure
  cross-marker-specific?) turned into the default for the entire rest of the investigation
  (12+ landing tests). User had to call it out explicitly. Fixed: loud restatement at the top
  of `px4/MEMORY.md` + incident logged in [[feedback_aruco_perception_scope]]. **The
  CBF_MARGIN_RESERVE result above is therefore ArUco-only data -- it has NOT been checked on
  cross-marker at all.**
- **Did not check for concurrent SITL sessions before launching**, causing a contaminated
  sweep that had to be redone -- see [[feedback_check_concurrent_sitl_before_launch]] (a
  parallel lesson independently learned by the other concurrent session the same day).

## Next-session priorities, in order

1. **Re-run the `CBF_MARGIN_RESERVE` A/B on cross-marker** (not ArUco) -- the existing result
   is marker-type-mismatched with the project's actual active track and was inconclusive
   anyway; this needs to be redone properly regardless.
2. **Implement the structural `th_curr` fix** for the self-defeating `dtheta` loop (anchor
   against desired, not realized, attitude in `cbf2_filter`) -- the band-aid (cap+crossfade)
   only prevents the fly-away symptom, doesn't address why `dtheta` runs chronically high.
3. **Check whether `HW_FROZEN` (the other session's in-progress touchdown-detect fix,
   uncommitted as of this wrap-up) interacts with anything in this thread** -- both threads
   touch `controller.py`/perception timing; worth a sanity check once both are committed.
4. Before ANY new SITL launch: `ps -eo pid,ppid,tty,user,lstart,cmd | grep -E
   "px4_sitl|gz sim|landing_test.py|MicroXRCEAgent"` AND confirm `WORLD=cross_marker
   MARKER_TYPE=cross` is in the command, every single time -- both were violated this thread
   despite existing rules, and both are now loudly restated at the top of `px4/MEMORY.md`.
