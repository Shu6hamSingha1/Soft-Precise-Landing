# Memory Index — SHARED (cross-cutting MATLAB <-> PX4)

> Findings that span both phases (e.g. a MATLAB result that ports to PX4). Either chat
> may append here; keep entries linked from both phases. Rebase before push.

- [Concurrent-session lanes: the finding-owner session writes its own STATUS/memory; cross-cutting sessions keep to records/preservation/verified-error fixes](feedback_concurrent_session_lanes.md) — 2026-07-02 user correction (audit session had pre-empted the rover session's speed-sweep STATUS update); also: re-read shared files right before editing + check for owner's own data copy before preserving scratchpad data. Post-shrink pointer map: root MEMORY.md = COLD 45-line core; hot shared surfaces = px4/MEMORY.md banners + guide STATUS; phase indexes are the append targets
- [Moment-loom descent fix (MATLAB→PX4)](project_moment_loom.md) — 2026-06-19; descent failures root-caused to pinv(L_s) loom = vz/z divergence on σ_min≈2 weak mode (tol truncate→sign-flip; low tol→limit cycle). Fix = scale-free moment area-rate loom −½·d(lnM)/dt (rotation-immune, no gyro) → MATLAB 92→95/100, better-calibrated. PX4: port to img_data.py (PX4 chat already has FLOW_LOOM_DECOUPLE). Gated USE_MOMENT_LOOM default-off.
