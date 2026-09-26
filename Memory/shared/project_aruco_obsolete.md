---
name: project_aruco_obsolete
description: 2026-09-26 user decision — the ArUco marker path is OBSOLETE; cross-marker is the baseline. Do not test, re-gate or maintain ArUco.
metadata:
  type: project
---

User (2026-09-26): "We don't need to test the ArUco path. It is an obsolete approach now. We have a baseline with cross-marker now. So we can forget about ArUco marker."

**Why:** the cross-marker pipeline (stroke detector, loss fade, touchdown settle — see [[project_20260924_stroke_detector_rewrite]]) is the working baseline; ArUco was comparison-only.

**How to apply:** don't run ArUco regressions or re-gates for new defaults (e.g. PLASMC_TD_SETTLE_S is baked without an ArUco check), don't list ArUco as a caveat/open item, don't recalibrate its cal. Default worlds/markers for any test: WORLD=cross_marker MARKER_TYPE=cross (the launcher's own default is still aruco — set explicitly). Supersedes CLAUDE.md's "ArUco comparison-only" framing.
