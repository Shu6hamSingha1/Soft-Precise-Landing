---
name: feedback-verify-injected-docs-before-trusting
description: "The auto-injected STATUS block and CLAUDE.md both silently went stale and asserted wrong gains/camera facts — verify load-bearing numbers against source files, never quote them from a doc"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: d5ea67ff-56f9-4c30-84f2-28d7d6cfab7d
  modified: 2026-09-02T06:14:07.112Z
---

On 2026-09-02 the two documents loaded into EVERY session were both materially wrong:

- **`docs/PLASMC_TUNING_GUIDE.md` STATUS block** — last updated 2026-07-09 despite its own header
  saying "KEEP THE STATUS BLOCK CURRENT". It still labelled the 07-09 ArUco session "(LATEST)" and
  predated the cross-marker default rule, the 320×240 camera change, the cross cal re-derive, the
  `CROSS_ALPHA_0` fix and the whole rover phase. Its baked-gain list was **wrong on 7 of ~15
  entries** (`P`, `XI2`, `E`, `Ω`, `N`, `W_U_MAX` all wrong; `KAPPA_MAX_XY` missing).
- **`CLAUDE.md`** — asserted 640×480 / fx=270 in 3 places; the live SDF has been 320×240 / fx=135
  since 2026-08-27. Its sensor-cal section described only one calibration when there are two.

**Why:** these are the highest-trust surfaces in the project — they are injected automatically and
read as authoritative, so a wrong number there propagates into tuning reasoning unchallenged and is
never re-derived. Staleness in an auto-injected doc is strictly worse than staleness in a doc
someone has to deliberately open. This is also a plausible contributor to the repeated ArUco-default
violations: every session opened with an obsolete picture that contradicted the newer rule.

**How to apply:** before using any concrete value from CLAUDE.md, the STATUS block, or a memory
entry — a gain, a resolution, a calibration date, a default — **read it out of the source file
first**. Cheap checks:
`grep -oE 'pa\("[A-Z0-9_]+", *[0-9.e+, ]*\)' src/controller.py` for per-axis gains;
`grep -E '<width>|<height>|horizontal_fov' ~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf`
for the camera. When a doc and the code disagree, the code wins and the doc gets fixed in the same
pass. When writing into an injected doc, mark each entry as verified-with-date or ⚠-unverified
rather than silently carrying values forward — carrying forward is exactly how these drifted.

Related: [[project_landing_test_is_a_duplicate_mirror]] (the same session's data-side cleanup).
Fixed in `c60eabc3`; ArUco-era history preserved at `docs/TUNING_HISTORY_ARUCO_ERA.md`.
