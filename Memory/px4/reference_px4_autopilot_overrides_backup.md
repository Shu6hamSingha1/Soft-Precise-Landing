---
name: reference_px4_autopilot_overrides_backup
description: "PX4_Gazebo/px4_autopilot_overrides/ (git-tracked in Soft-Precise-Landing) is a snapshot+restore point for every ~/PX4-Autopilot file modified for this project (camera/marker/world SDFs, airframe configs) -- ~/PX4-Autopilot itself is an external repo not tracked by this project's git, so those edits are invisible to git status/log and would be silently lost on a PX4-Autopilot reset/reinstall/re-clone. Check/update this whenever a PX4-Autopilot asset is edited."
metadata: 
  node_type: memory
  type: reference
  originSessionId: eed8d6c3-67a5-47df-94ed-188c3c073ae8
  modified: 2026-08-09T08:28:56.637Z
---

`PX4_Gazebo/px4_autopilot_overrides/` in the Soft-Precise-Landing repo holds
a full snapshot (as of 2026-08-09) of every file under `~/PX4-Autopilot`
that's been modified or added for this project -- camera/marker/world SDFs,
airframe boot configs. Its own `README.md` has the restore command
(`cp -r Tools ROMFS ~/PX4-Autopilot/`) and a per-file changelog.

**Why this exists:** `~/PX4-Autopilot` is a separate, externally-managed
repo -- none of the Gazebo asset edits made there over the course of this
project (camera mount pose, marker textures, landing-gear collision
geometry, world shadow settings, etc.) are visible to `git status`/`git log`
in the Soft-Precise-Landing repo, so a PX4-Autopilot reset/reinstall/re-clone
would silently erase all of it with no record.

**How to apply:** whenever a NEW edit is made to a `~/PX4-Autopilot` file
(a `.sdf`, `.png` texture, airframe config, etc.), copy the updated file
into the matching path under `px4_autopilot_overrides/` and update its
README changelog entry -- treat it as a live backup, not a one-time snapshot.
If you're about to touch a PX4-Autopilot file and aren't sure whether it's
already covered, check this directory first (`find PX4_Gazebo/
px4_autopilot_overrides -type f`) rather than assuming.

See [[project_cross_marker_pipeline_20260801]] for the substantive history
behind each backed-up file (why each edit was made).
