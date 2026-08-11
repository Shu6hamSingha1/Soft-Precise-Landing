# PX4-Autopilot overrides — backup & restore

`~/PX4-Autopilot` is an external repo, not tracked by this project's git —
any edits made there for Gazebo simulation assets (camera/marker/world SDFs,
airframe configs) are invisible to `git status`/`git log` in this repo and
would be **silently lost** if that PX4-Autopilot checkout is ever reset,
reinstalled, or re-cloned. This directory is a snapshot of every file under
`~/PX4-Autopilot` that has been modified (or added) for this project, taken
2026-08-09, mirroring the real repo's relative paths. It IS tracked by this
repo's git, so it survives a PX4-Autopilot reset.

## Restore (after a fresh/reset PX4-Autopilot checkout)

```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo/px4_autopilot_overrides
cp -r Tools ROMFS ~/PX4-Autopilot/
```

That's it — `cp -r` merges into the existing tree since the directory
structure mirrors PX4-Autopilot's own layout exactly. Verify with:

```bash
diff -r ~/Soft-Precise-Landing/PX4_Gazebo/px4_autopilot_overrides/Tools ~/PX4-Autopilot/Tools
diff -r ~/Soft-Precise-Landing/PX4_Gazebo/px4_autopilot_overrides/ROMFS ~/PX4-Autopilot/ROMFS
```
(only diffing the subpaths that exist under this override dir — both
commands should print nothing if the restore worked.)

## What's in here and why (one line each; see the project memory /
`Memory/px4/project_cross_marker_pipeline_20260801.md` for full history)

**Cameras / vehicle:**
- `Tools/simulation/gz/models/mono_cam/model.sdf` — camera intrinsics
  (640x480, hfov=1.74 -> fx=fy=270px). Likely still stock/unmodified, backed
  up anyway for completeness (nothing in it carries a dated edit comment).
- `Tools/simulation/gz/models/x500_mono_cam_down/model.sdf` — camera mount
  pose on the airframe. Current LIVE state: `<pose>0 0 .15 0 1.5707
  1.5707</pose>` (stock/reverted — the 2026-08-09 X-offset prototype was
  tested and found not to help, reverted cleanly). History baked into this
  one file's own comments: yaw+90 (2026-08-04, moves the landing-leg ghost
  from top/bottom to left/right margins), Z .20->.18->.15 (2026-08-05,
  keeps the landing legs out of the downward FoV -- this Z value is
  deliberately chosen, not arbitrary, confirmed via direct A/B test).
- `Tools/simulation/gz/models/x500_base/model.sdf` — the airframe itself.
  **LIVE PROTOTYPE CHANGE (2026-08-09, still active):** `base_link_
  collision_3`/`_4` (landing-gear skid bars, the actual ground-contact
  collision element) extended from Z=-.2195 to Z=-.7195 (+0.5m). COLLISION-
  ONLY (physics proxy, invisible to the camera render -- the visual mesh
  `NXP-HGD-CF.dae` is untouched), raises the physical touchdown height
  without any camera-visibility tradeoff. Confirmed working via telemetry
  (body touchdown altitude 0.05m -> 0.487m) and visually by the user in the
  Gazebo GUI (clear gap between the rendered legs and the ground at rest).
  Fixes (partially -- not fully, see below) the cross-marker's close-range
  texture-resolution flow-quality collapse.
- `ROMFS/px4fmu_common/init.d-posix/airframes/4014_gz_x500_mono_cam_down` —
  airframe 4014 boot config (x500_mono_cam_down, stationary-target scenario).

**Cross+stub marker (decode-free ArUco alternative):**
- `Tools/simulation/gz/models/cross_marker/model.sdf` — marker plate (3m
  plane). Fixed a PBR-material mirroring bug (2026-08-01, explicit
  metalness=0/roughness=1) and the camera-mount-yaw+90 downstream axis-sign
  fix.
- `Tools/simulation/gz/models/cross_marker/cross_marker.png` — the marker's
  texture (cross+stub shape + speckle background for GFT point density).
  1024x1024px currently -- this is the file a higher-resolution swap (to
  push the texture-resolution crossover altitude down, see below) would
  replace. `_finetex`/`_reftex` (+2) variants are earlier iteration
  candidates, kept for reference/rollback, not the live texture.

**ArUco (nested board + single-marker variants):**
- `Tools/simulation/gz/models/arucotag/model.sdf` — single/nested ArUco
  marker geometry, iterated extensively (nested multi-scale board ->
  single-large-marker world, see the project's own memory for the
  rank-deficiency investigation that drove this).
- `Tools/simulation/gz/worlds/aruco.sdf` — the stationary-ArUco-target world
  (shadow rendering tweak).

**Moving-target (rover) scenario:**
- `Tools/simulation/gz/models/rover_aruco/model.sdf` — marker mounted on
  the rover.
- `ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_rover_aruco` —
  airframe 4022 boot config (rover scenario).
- `Tools/simulation/gz/worlds/rover.sdf` — the moving-target world (montage/
  camera tweak for recording).

**Cross-marker world:**
- `Tools/simulation/gz/worlds/cross_marker.sdf` — the dedicated cross+stub
  world (shadow rendering tweak, same class of change as aruco.sdf's).

## Known gap as of this snapshot (2026-08-09)

The landing-gear extension above is a real, confirmed improvement but not a
full fix on its own -- camera-to-marker distance at touchdown is still
short of the texture-resolution crossover. The next planned step (not yet
done) is swapping `cross_marker.png` for a higher-resolution texture,
which is NOT yet reflected in this backup -- when that lands, re-run this
same snapshot process to capture it.
