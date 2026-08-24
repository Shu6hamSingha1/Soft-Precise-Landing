---
name: project_20260814_cbf_radius_split_and_hardware_check
description: "Explains commit 27410ed (CBF radius split into live-capped vs fixed-Phase-2), confirms Hardware has zero cross-marker code (extent bug doesn't apply there), and the NaN-fix cross-session reminder file"
metadata: 
  node_type: memory
  type: project
  originSessionId: bc71d54e-36e7-454b-9b2e-33cc103d6bc0
  modified: 2026-08-13T20:06:10.449Z
---

**Commit `27410ed`** ("Cross-marker FOV-CBF: split radius into live-capped (d_min_fov) vs
fixed (Phase-2)") splits the single `cbf_radius` from the 2026-08-13 extent-blindness fix into
two independent knobs, `src/controller.py` (near top of file):
- `CROSS_CBF_PHASE2_RADIUS_PX` (fixed, default 100px) — feeds ONLY `cbf2_filter`'s Phase-2
  decode-fail fallback (`delta2`/`state["delta_prev"]`). A live/growing value there risked
  reactivating 3 known Phase-2 defects (scalar-collapse via `np.min`, direction-blind magnitude
  clamp, margin-flooring self-latch) that `cbf_visibility_aruco.py`'s rewrite exists to fix —
  dormant for cross-marker only because a bare center point made `delta_prev` trivially zero
  before the extent fix.
- `CROSS_CBF_RADIUS_CAP_PX` (cap, default 100px) — caps the LIVE radius before it feeds
  `controller.py`'s own `d_min_fov`/overflow/drift-off classification (lines ~2770-2785). At
  point-blank range (10 pre-takeoff flights landed directly on the marker), `MARKER_EXTENT_PX`
  is consistently 633-639px, far exceeding `rho_fov`'s smaller axis (~210-290px). Uncapped, this
  trips BOTH per-axis breach checks simultaneously even at perfect centering -> classified as
  OVERFLOW (which ArUco treats as benign hand-over-readiness to a smaller nested marker).
  Cross-marker has no hand-over target, so this would silently disable drift-off pullback — the
  only real corrective mechanism cross-marker has — exactly when the marker is largest/closest.

**Validated via a rerun of the n=5 IC1-5 sweep** (see [[project_20260813_cbf_extent_fix_followup]]
for full before/after numbers): the two catastrophic fly-away outliers from the pre-split sweep
are GONE (IC3 max xy_err 28.673m->2.877m, IC4 14.752m->3.430m). Overall precise/soft counts
unchanged (0/25, 1/25) since the separate lateral-convergence-rate root cause (see the other
memory file) is untouched by this fix. Both fixes are real, validated, and address DIFFERENT
failure modes.

**Hardware cross-marker status, checked 2026-08-14 (full git history search, `git log --all
--oneline | grep -i cross` across the whole repo, zero hits touching `Hardware/`):**
`Hardware/scripts/controller.py` has no `MARKER_TYPE` branching, imports a single unconditional
`cbf_visibility.py` (old pre-split signature, no `radius` param), which derives `delta2` from
the real ArUco corner-array spread directly (`0.5*(ct.max(0)-ct.min(0))`) — same as
`PX4_Gazebo/src/cbf_visibility_aruco.py`. **Hardware is ArUco-only; there is no cross-marker
code path there at all**, so the extent-blindness bug (and today's radius-split fix) genuinely
don't apply — nothing to port. If a different session claimed this was "fixed on Hardware,"
that's very likely a mix-up with the PX4_Gazebo-side fix (which IS real and DOES apply there);
worth clarifying with that session directly, since the repo shows no trace of a cross-marker
port to Hardware (local `main` also confirmed in sync with `origin/main`, nothing unpushed
missed).

**NaN self-latch bug — confirmed STILL PRESENT in `Hardware/scripts/controller.py` (~line
1195-1197)**, same unguarded `self._last_loop_dt = _now_loop - self._last_loop_t` pattern (no
dt floor, no finite check) as the PX4_Gazebo pre-fix code. This one DOES affect Hardware (it's
a general wall-clock timing bug, not marker-specific — unlike the CBF bug above). Created a
git-tracked cross-session reminder file, `Hardware/docs/PENDING_NAN_FIX_PORT.md`, so a Windows
Claude session (which won't share this machine's local memory store) can pick it up via a
normal `git pull`. Instructs: port the fix from `git show 2a8ab76 -- PX4_Gazebo/src/controller.py`
(dt floor + finite-check guard on the `dh_d` division), verify, then DELETE the file as the
completion signal — so a later PX4/Gazebo-side session can check for its absence rather than
trusting a status claim. **Not yet committed/pushed as of this session — user was asked to
confirm before pushing** (the file only helps once it's actually on the shared repo).
