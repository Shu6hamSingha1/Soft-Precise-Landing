# Memory Index — SHARED (cross-cutting MATLAB <-> PX4)

> Findings that span both phases (e.g. a MATLAB result that ports to PX4). Either chat
> may append here; keep entries [[linked]] from both phases. Rebase before push.

- [Moment-loom descent fix (MATLAB→PX4)](project_moment_loom.md) — 2026-06-19; descent failures root-caused to pinv(L_s) loom = vz/z divergence on σ_min≈2 weak mode (tol truncate→sign-flip; low tol→limit cycle). Fix = scale-free moment area-rate loom −½·d(lnM)/dt (rotation-immune, no gyro) → MATLAB 92→95/100, better-calibrated. PX4: port to img_data.py (PX4 chat already has FLOW_LOOM_DECOUPLE). Gated USE_MOMENT_LOOM default-off.
