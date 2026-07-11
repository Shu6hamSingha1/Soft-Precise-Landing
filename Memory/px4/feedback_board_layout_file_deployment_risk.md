---
name: feedback_board_layout_file_deployment_risk
description: "Images/aruco_board_layout.npy was auto-loaded from a hardcoded default path whenever present, hardcoding THIS deployment's marker IDs ([0,10]) -- a future deployment with different IDs would silently use this wrong, non-corresponding layout during the ~5-frame self-cal warm-up window. Fixed 2026-07-11: file only loads if ARUCO_BOARD_LAYOUT is explicitly set; default is pure self-cal (ID/size-free) + single-marker fallback during warm-up."
metadata:
  node_type: memory
  type: feedback
  originSessionId: af2bb6fc-90fc-4192-b79a-1590e374c873
---

## Context

User directive: the board-homography centroid/yaw path (`_board_feature`) should be ID-free and
size-free for deployment, since future marker IDs/sizes aren't known in advance. Investigated
whether this required a bigger architectural change (geometric/frame-to-frame data association
replacing ID-based correspondence) or was already satisfied.

**Scope clarified by user:** ArUco decode stays (markers still produce a numeric ID via
`cv2.aruco`) — the constraint is only that IDs/sizes aren't *known ahead of deployment*, not that
decode itself becomes label-free. Under this scope, [[reference_board_selfcal]] (i.e.
`_update_board_selfcal`, default `BOARD_SELFCAL=1`) already satisfies the requirement structurally:
- Never uses physical/metric size anywhere (`_board_corners`'s layout is scale-invariant,
  normalized to an arbitrary reference — `findHomography` doesn't care about real units).
- Discovers whichever marker IDs actually decode live, with zero pre-registration; IDs are used
  purely as a same-session correspondence bookkeeping key ("this detection is the same physical
  tag I saw last frame"), not a hardcoded deployment assumption.
- Bootstraps from the first co-visible (≥2 marker) frame, similarity-fits later frames to the
  running layout, gates on residual (rejects poorly-leveled/occluded frames).

**So no data-association rewrite was needed** for the stated scope — that would only be required
if markers become genuinely unlabeled (identical fiducials with no decode ID at all), which the
user did NOT select.

## The real gap found and fixed

`Images/aruco_board_layout.npy` (a FILE PRIOR, used only until self-cal warms up, or as a
fallback if self-cal is disabled) was auto-loaded from a hardcoded default path
(`os.path.join(..., 'Images', 'aruco_board_layout.npy')`) whenever the file happened to exist —
no opt-in required. Checked its contents: IDs `[0, 10]`, correct for the CURRENT nested-marker
deployment, but literally deployment-specific data committed to the repo.

**The hazard:** self-cal takes `BOARD_SELFCAL_MIN_FRAMES=5` frames to warm up. Before that, `
_board_feature` falls back to this file prior if present. A future deployment with DIFFERENT
marker IDs would silently load THIS file (IDs 0/10, not the new deployment's IDs), producing a
wrong/non-corresponding homography fit for the first ~5 frames of every landing — a real,
deployment-portability landmine, exactly the risk the user was flagging.

**Fix (`src/img_data.py:__init__`, 2026-07-11):** `_layout_path` no longer defaults to the
hardcoded repo path — `ARUCO_BOARD_LAYOUT` env var must be EXPLICITLY set for any file prior to
load at all. Default behavior is now pure self-cal (ID/size-free) + single-marker moment fallback
bridging the ~5-frame warm-up window (which needs no ID/layout knowledge of any kind). No script
in `scripts/`/`apps/`/`tools/` referenced `ARUCO_BOARD_LAYOUT` before this change, so nothing else
needed updating.

## How to apply

- Don't reintroduce a default-path auto-load for `aruco_board_layout.npy` or any similar
  deployment-specific prior file — treat "opt-in via explicit env var" as the pattern for any
  future per-deployment calibration/layout artifact.
- If a future session wants a KNOWN-correct prior for a specific redeployment (e.g. to skip the
  5-frame self-cal warm-up), set `ARUCO_BOARD_LAYOUT=<path>` explicitly rather than relying on
  the default.
- If deployment ever moves to genuinely unlabeled fiducials (no decode ID), THAT is the point
  where `_board_feature`/`_update_board_selfcal`'s ID-keyed correspondence needs a real redesign
  (geometric/RANSAC data association) — not yet needed under the current scope.
