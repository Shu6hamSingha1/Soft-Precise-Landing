---
name: project_20260826_chasecam_resolution_bump
description: "2026-08-26: bumped rover_cross.sdf's chase-cam sensor resolution 640x480 -> 1280x960 to fix visibly blurry montage videos. Validated via a real landing run that this does NOT starve the down-cam's (perception-critical) frame rate -- median/mean fps and worst-case stall were statistically indistinguishable from a 640x480-chase baseline run."
metadata: 
  node_type: memory
  type: project
  modified: 2026-08-26T00:49:41.112Z
  originSessionId: 64366844-45aa-444e-96db-ea10a4e7d714
---

## Finding

The "main" recorded video in every montage (the third-person CHASE view) was native
640x480, same resolution as the down-facing perception camera, then upscaled ~1.5x AND
further center-cropped (`--chase-crop`, added [[project_20260825...]] zoom work) before
display -- compounding into visibly blurry/pixelated footage. The chase-cam sensor is
PURELY for external recording (never consumed by perception/control), so unlike the
downward camera (which CLAUDE.md documents as rejected at 1280x960 due to Gazebo's own
renderer dropping to 21Hz with 92ms bridge outliers), there was no control-loop-latency
reason to keep it low-res.

## Risk considered, and ruled out

Gazebo renders all cameras on the same shared render thread/GPU, so a second high-res
sensor COULD still starve the down-cam's frame budget even though nothing consumes the
chase feed in real time. Validated with a real `montage_landing_cross.sh` run
(`chasehz_test`, `rover_cross.sdf`'s chase sensor bumped to 1280x960 @ same 20Hz
`update_rate`) by comparing the down-cam's own `Img_Data.npy` `Time` log against a same-day
640x480-chase baseline (`gapfill_verify`):

| | median fps | mean fps | worst single-frame gap |
|---|---|---|---|
| 640x480 chase (baseline) | 62.5 | 33.4 | 0.894s (1.12 fps) |
| 1280x960 chase (bumped) | 62.5 | 34.7 | 0.766s (1.31 fps) |

Statistically indistinguishable (the bumped run's worst-case stall was actually slightly
BETTER) -- the occasional multi-hundred-ms stall is a pre-existing SITL/detection-dropout
characteristic, not caused by the chase-cam resolution.

## Change

`~/PX4-Autopilot/Tools/simulation/gz/worlds/rover_cross.sdf` (outside this repo, not
git-tracked here) -- chase sensor `<image><width>/<height>` 640x480 -> 1280x960. Original
backed up as `rover_cross.sdf.bak_before_chasehz_20260826`. `rover.sdf` (ArUco world,
comparison-only per [[feedback_aruco_perception_scope]]) NOT touched.

## SUPERSEDED same day: 1280x960 -> 1920x1440 + zoom reverted back OUT

User's next request ("increase resolution further AND zoom out") revealed the 2026-08-25
zoom-in (pose moved 0.7x closer + FOV 2.0->1.6, see that session's own chase-cam-zoom
memory) was actively counterproductive once combined with `--chase-crop`: at
descent-start the rover sits far from the pose's look-at centroid, and the tight FOV
clipped/overlapped it under the left PiP panel.

Fix: reverted pose/FOV fully back to the pre-zoom values (`8 -8 6 0 0.442 2.216`,
`horizontal_fov=2.0`; the 1.6-FOV zoomed-in intermediate is now in
`.bak_before_chasehz1920_20260826`), and bumped resolution again to **1920x1440**.
Re-validated the same way: `chasehz1920_test` run, median fps still 62.5, worst-case gap
0.64s (better than both prior runs, not worse) -- still no down-cam impact.

**Current recommended pattern**: keep the WORLD-level pose/FOV wide (no clipping risk,
covers all trajectories including wide ones like circular), and do any zoom-in via
`make_landing_montage.py --chase-crop` (post-process, tunable per montage, reversible
without re-flying) -- e.g. `--chase-crop 0.8` gave a nicely tight-but-uncropped frame on
the 1920x1440 source. Don't re-introduce a world-level FOV/pose zoom without checking it
against a wide-altitude-range frame (descent-start, not just touchdown) first.

## How to apply

Any future chase-cam recording via `rover_cross` world gets the 1920x1440, wide-FOV
source automatically -- no action needed. Use `--chase-crop` per-montage for framing, not
the world SDF. If a future change to `update_rate` or a MUCH larger resolution bump is
considered, re-validate the down-cam fps the same way (compare `Img_Data.npy`'s `Time`
log median/mean/worst-case gap against a known-good baseline) rather than assuming
headroom holds indefinitely.
