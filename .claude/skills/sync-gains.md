# /sync-gains

Audit MDF-ASMC gain parity across the three files that must stay in lock-step, and report any drift. Use this **before** committing a tuning round and **after** any edit to MDF-ASMC gains, funnels, or the hard-coded `izeta_2_max` / `att_cone` / precision-criterion constants.

> **STATUS 2026-07-02 — `MATLAB/VDF_ASMC/vdf_params.m` is now the primary gain source for the
> blocks path.** `run_simulation.m` and `visualControl_comparison.m` consume the `+blocks/`
> controller, whose gains come from `vdf_params()` (override via `global VDF_OVERRIDE`), NOT from
> inline constants. The three-file audit below still applies to the legacy inline gains
> (visualControl_IBVS_adaptive keeps its own inline controller), but any VDF-ASMC tuning round
> lands in `vdf_params.m` FIRST — check IT for drift against the manuscript before the legacy trio.
> The 2026-06-26 LOCKED bake (kappa0/N/Pleak/E/Xi_r/p_rinf/theta_per_axis) lives there; backup
> `Obsolete/vdf_params_pre_LOCKED.m`.

## Why this skill exists

MDF-ASMC gains live in three places that must agree:

1. `MATLAB/Multi_init_cond/run_simulation.m` — source of truth for the multi-init sweep
2. `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m` — single-run MDF-ASMC sim
3. `MATLAB/Comparison/InitGains_Comparison.m` — 5-controller comparison harness (`K_PLASMC` struct — MATLAB identifier retained for code compatibility)

Drift between them produces silent lies: the sweep reports 50/50 with one gain set while the manuscript's Table VIII runs a *different* controller. The 2026-04-15 lock-in session shipped with `izeta_2_max` drifted (10 vs 5.0) despite a manual audit — the Edit tool's pattern match missed one call site. This skill automates the check.

Non-gain constants that must also match:
- `izeta_2_max` (anti-windup clamp — 3 files)
- `att_cone` / `a_xy_max` cone angle (2 files — run_simulation and visualControl_comparison)
- Precision criterion `0.08 m` (7 call sites across 3 files — see below)
- `h_rd` in `MATLAB/Common/Constants.m` (single source, no drift possible — but worth reporting the current value)

## Usage

```
/sync-gains                    -- full audit, report all drift
/sync-gains fix                -- audit + auto-apply run_simulation.m values to the other two files (requires user confirmation before each Edit)
```

## Canonical parameter list

Extract these from `run_simulation.m` (source of truth) and compare against the other files:

| Param | run_simulation.m | visualControl_IBVS_adaptive.m | InitGains_Comparison.m |
|---|---|---|---|
| `gamma_1` | `K_ctrl.gamma_1 = [...]` | `K.gamma_1 = [...]` | `K_PLASMC.gamma_1 = [...]` |
| `p_1inf` | `K_ctrl.p_1inf` | `K.p_1inf` | `K_PLASMC.p_1inf` |
| `zp` | `K_ctrl.zp = diag(...)` | `K.zp` | `K_PLASMC.zp` |
| `zi` | `K_ctrl.zi` | `K.zi` | `K_PLASMC.zi` |
| `zd` | `K_ctrl.zd` | `K.zd` | `K_PLASMC.zd` |
| `gamma_2` | `K_ctrl.gamma_2` | `K.gamma_2` | `K_PLASMC.gamma_2` |
| `p_20` | `K_ctrl.p_20` | `K.p_20` | `K_PLASMC.p_20` |
| `p_2inf` | `K_ctrl.p_2inf` | `K.p_2inf` | `K_PLASMC.p_2inf` |
| `Omega` | `K_ctrl.Omega = diag(...)` | `K.Omega` | `K_PLASMC.Omega` |
| `Gamma` | `K_ctrl.Gamma` | `K.Gamma` | `K_PLASMC.Gamma` |
| `P` | `K_ctrl.P` | `K.P` | `K_PLASMC.P` |
| `N` | `K_ctrl.N` | `K.N` | `K_PLASMC.N` |
| `kappa_0` | `K_ctrl.kappa_0` | `K.kappa_0` | `K_PLASMC.kappa_0` |
| `E` | `K_ctrl.E` | `K.E` | `K_PLASMC.E` |
| `izeta_2_max` | local var `izeta_2_max = N` | local var | local var in visualControl_comparison.m |
| `att_cone` | `att_cone = deg2rad(N)` | (not present — single-run sim doesn't clamp) | `visualControl_comparison.m` same line |
| Precision | `xy_err <= N` / `final_xy <= N` | same | same (visualControl_comparison.m) |

**Note on `visualControl_comparison.m`**: `izeta_2_max` and `att_cone` live in this file, NOT in `InitGains_Comparison.m`. The gain struct lives in `InitGains_Comparison.m`; the control-loop constants live in `visualControl_comparison.m`. A full audit must read both.

## Instructions

### Step 1 — Read all four files in parallel

Files to read (single batch):
1. `MATLAB/Multi_init_cond/run_simulation.m`
2. `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m`
3. `MATLAB/Comparison/InitGains_Comparison.m`
4. `MATLAB/Comparison/visualControl_comparison.m` (for `izeta_2_max`, `att_cone`, precision sites)

Also `MATLAB/Common/Constants.m` for the current `h_rd` value (report, don't compare).

### Step 2 — Extract each parameter

Use Grep with the exact LHS pattern (e.g., `K_ctrl\.gamma_1\s*=`) and capture the RHS literal. Don't try to evaluate expressions — compare strings after whitespace normalization.

For `izeta_2_max` use `izeta_2_max\s*=`.
For precision use `xy_err\s*<=\s*0\.` and `final_xy\s*<=\s*0\.` (capture all 7 call sites).
For `att_cone` use `att_cone\s*=\s*deg2rad`.

### Step 3 — Compare and report

Emit a table in the response. Mark each row:
- ✓ if all three files agree
- ✗ if any drift, with the drifted value and file

Example:
```
PARAM          RUN_SIM           VC_ADAPTIVE       INIT_GAINS        STATUS
gamma_1        [0.1, 0.1]        [0.1, 0.1]        [0.1, 0.1]        ✓
p_2inf         [2.5;2.5;1.5]     [2.5;2.5;1.5]     [2.5;2.5;1.5]     ✓
izeta_2_max    5.0               5.0               10.0 (vc_comp)    ✗ DRIFT
precision      0.08 (x3 sites)   0.08 (x2 sites)   0.08 (x2 sites)   ✓
att_cone       35° (run_sim)     n/a               35° (vc_comp)     ✓
h_rd           -0.46 (Common/Constants.m)                            [info]
```

### Step 4 — If `fix` argument was passed

For each `✗` row, propose an Edit to bring the drifted file into line with `run_simulation.m`. Ask the user to confirm before applying (one confirmation per drift, not one blanket).

Do NOT auto-fix drift without the `fix` argument. A silent report of drift is often all that's needed — sometimes the drift is intentional and the user will update `run_simulation.m` instead.

### Step 5 — Done

Under 150 words in the final report. Just the table plus one line per drift indicating severity (controller logic vs. termination threshold vs. cosmetic).

## Related

- The 2026-04-15 lock-in commit message (`b23926b`) documents the last synchronized state — use this as a reference point if an audit produces confusing output.
- `project_multi_init_final_lock.md` in memory holds the canonical values and the *why* for each.
- This skill is read-only by default. Only `fix` mode writes, and only with per-row confirmation.
