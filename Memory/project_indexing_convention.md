---
name: MATLAB landing-index convention — aggregator now aligned with tex (2026-04-22)
description: Both tex and Python aggregator now use (idx-1)*dt + x_t/dx_t col idx-1 convention. Off-by-one fix applied.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Status 2026-04-22:** `scripts/_analyze_results_for_tex.py` has been fixed to match the tex / MATLAB-log convention. Table V now reads directly from the aggregator without manual alignment.

**Convention (both sides):**
- `t_f = (idx - 1) * dt` — matches MATLAB's `fprintf('Landed at t=…')` printout
- `UAV pos/vel` at landing: `X_DS[:, idx]` in Python (MATLAB harness stores `X_DS(:, idx+1) = x_c`, so Python col `idx` = post-step state)
- `target pos/vel` at landing: `x_t[:, idx-1]` and `dx_t[:, idx-1]` in Python (MATLAB fills `x_t(:, k)` in place, so MATLAB col `idx` = Python col `idx-1`)
- **The two array families use DIFFERENT column-index conventions** — UAV uses Python `idx`, target uses Python `idx-1`. Off-by-one if you miss this.

**Bug that motivated the fix:** prior aggregator used `x_t[:, idx]` / `dx_t[:, idx]` (wrong — reads uninitialized preallocation column). Caused spurious 32 m xy_e on PLASMC-Linear while the actual landed run was 0.005 m. MATLAB harness was always correct; only the Python post-processor drifted.

**How to apply:**
- Any new script that reads termination state from a `{Traj}_comparison.mat` or similar aggregate MUST use `x_t[:, idx-1]` (not `x_t[:, idx]`) for target state, and `X_DS[:, idx]` for UAV state.
- Do NOT reintroduce a dual-convention (tex idx-1 / Python idx) — both must agree. When you refresh Table V, pasting aggregator cells directly into tex is safe now.
- Reference: `scripts/_analyze_results_for_tex.py` lines ~220–225 (comparison_report function) document the off-by-one in an inline comment.
