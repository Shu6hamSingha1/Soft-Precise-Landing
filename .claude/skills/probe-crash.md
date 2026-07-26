# /probe-crash

Reproduce a Multi_init_cond simulation failure at a specific `(trajectory, run)` and dump telemetry. Covers two distinct failure modes now that Approach 2 (FoV-adaptive cone clamp) is committed:

1. **Crash-fail** — `I_a_cd norm > 1e2` or NaN; `break` fires mid-run.
2. **Hover-fail** — run completes at `idx = N_steps` but UAV cannot touchdown (xy small, alt > 0.20m at t=40s). New failure mode after Approach 2.

## Usage

```
/probe-crash <trajType> <runIdx>
```

Arguments:
- `trajType` (required): one of `Static`, `Linear`, `Sinusoidal`, `Lissajous`, `Circular`
- `runIdx` (required): 1-5, matches the `k` in `multi_Init_Var.m`'s run loop

Examples:
```
/probe-crash Sinusoidal 4
/probe-crash Circular 3
```

## Seed and IC mapping (from multi_Init_Var.m)

```
p0(1,:) = [ 0,  0, -5]
p0(2,:) = [ 2,  2, -5]
p0(3,:) = [ 2, -2, -5]
p0(4,:) = [ 2,  2, -7]
p0(5,:) = [ 2,  2, -3]
seed    = 1000 + runIdx
```

Quaternion `q0=[1;0;0;0]`, zero velocity and angular rate. `cfg_override = struct('NOISE',1,'GE',1,'delay',1)`.

## Instructions

### Step 1 — Check existing dataset first

Before re-running MATLAB, try reading from `MATLAB/Multi_init_cond/Datasets/<trajType>_multi_init.mat`. The dataset already contains the full workspace of every run. Use `scripts/diag_ic4_z_channel.py` as a template for pulling signals with `scipy.io.loadmat`.

Only proceed to Step 2 if the dataset is stale or missing.

### Step 2 — Write the probe script (if needed)

Create `MATLAB/Multi_init_cond/probe_lateCrash.m` (reuse this filename; it's scratch and gets overwritten each call). Approach-2-aware version:

```matlab
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));

trajType = "<trajType>";
p0_table = [0,0,-5; 2,2,-5; 2,-2,-5; 2,2,-7; 2,2,-3];
k = <runIdx>;
x0 = [p0_table(k,:)'; 1;0;0;0; zeros(6,1)];
cfg = struct('NOISE',1,'GE',1,'delay',1);

tmp = run_simulation(x0, trajType, [], 1.0, cfg, 1000+k);
d   = tmp.data;

idx_f = d.idx;
win   = max(1, idx_f-20) : idx_f;
fprintf('\n=== Telemetry: %s R%d, landed=%d, idx=%d t=%.3fs ===\n', ...
    trajType, k, tmp.success, idx_f, d.tRange(idx_f));
fprintf('  t       ||I_a_cd||  ||V_s_e_n||  ||zeta_2||  ||sigma||  ||kappa||  theta_cone  d_min\n');
for ii = win
    fprintf('  %6.3f  %9.2f  %10.4f  %9.4f  %9.4f  %9.4f  %9.2f  %7.1f\n', ...
        d.tRange(ii), norm(d.I_a_cd(:,ii)), norm(d.V_s_e_n(:,ii)), ...
        norm(d.zeta_2(:,ii)), norm(d.sigma(:,ii)), ...
        norm(d.kappa(:,ii)), rad2deg(d.theta_cone_log(ii)), d.d_min_log(ii));
end
fprintf('\nDrone pos : [%.3f %.3f %.3f]\n', d.I_p_c);
fprintf('Target pos: [%.3f %.3f %.3f]\n', d.x_t(1:3,idx_f));
fprintf('Drone vel : [%.3f %.3f %.3f]\n', d.I_v_c);
fprintf('wind_mean : [%.3f %.3f %.3f]  sigma=%.2f\n', d.wind_mean, d.wind_sigma);
fprintf('delta_m=%+.3f  delta_J=%+.3f  r_cog=[%+.4f %+.4f %+.4f]\n', d.delta_m, d.delta_J, d.r_cog);
fprintf('kappa final: [%.4f %.4f %.4f]\n', d.kappa(:,idx_f));
```

Note: Approach 2 removed `zeta_1`, `p_1`, `dp_1`, `S_1`, `G_1` — do NOT reference these. Use `V_s_e_n` (normalized raw error) instead. New Approach-2 logs: `V_s_e_n`, `iV_s_e_n`, `raw_dV_s_e_n`, `rho_fov_log`, `d_min_log`, `theta_cur_log`, `theta_cone_log`.

### Step 3 — Run it

```bash
cd "L:/Claude/Soft Landing/MATLAB/Multi_init_cond" && matlab -batch "probe_lateCrash" 2>&1 | tail -60
```

This overrides CLAUDE.md's "don't run MATLAB" convention — crash/hover probing is a diagnostic loop the user has approved as an exception.

### Step 4 — Interpret

**For crash-fail** (idx < N_steps), look at the 20-step window and identify which signal blows up first. Common mechanisms:

1. **FoV corner saturation** — `d_min_log` drops to 0, `theta_cone` collapses to `theta_current` (body tilt). Lateral authority dies; `I_a_cd(1:2)` spikes as the cone clamp zeros them out. New failure mode after Approach 2.
2. **Funnel saturation** — `||zeta_2||` near `log(39) ≈ 3.66` (the `S_2_margin=0.05` clamp). Outer funnel physically unachievable under current disturbance.
3. **Kappa runaway** — adaptive gain growing without bound before `I_a_cd` blows up. Check `K_ctrl.N` (leakage) vs `K_ctrl.P` (growth rate).
4. **Wind-driven overshoot** — `wind_mean` near max (~0.2 m/s per axis) plus adverse `delta_m`/`delta_J`/`r_cog` combo. Runs 4/5 ICs `[2,±2,-5]` are off-center so lateral wind compounds startup transient.

**For hover-fail** (idx == N_steps but not landed), cone-clamp mechanisms are usually NOT the culprit. Check:

1. **z-channel integrator saturation** — `izeta_2(3)` pinned at ±5.0 for >50% of run with persistent non-zero `zeta_2(3)` and `I_a_cd(3) ≈ −9 m/s²` (hover gravity). This is the signature of the IC4 regression documented in `project_ic4_z_hover_fail.md`. Load-bearing suspect: `Omega(3,3)=0.006` structurally too small.
2. **Lateral integrator saturation** — analogous but on lateral channels; `xy_err` won't close despite cone being healthy.
3. **h_rd / descent-rate mismatch** — commanded image rate produces decaying altitude rate near ground.

### Step 5 — Report

Under 250 words, structured as:
- **Failure mode:** crash-fail vs hover-fail; traj, run, idx, t
- **Dominant signal:** which signal saturated/stalled first
- **Mechanism:** which of the above fits
- **Proposed lever:** the single most likely fix (gain, funnel, FoV param, disturbance softening, or sensor model)

Do NOT propose edits in the report. The user will decide the lever.

## Related

- `run_simulation.m` line ~423 is the crash `break` condition: `norm(I_a_cd) > 1e2 || NaN`.
- `run_simulation.m` lines 20-44 hold the robustness model (mass/inertia/CoG/wind/pixel-noise magnitudes).
- `probe_lateCrash.m` is scratch — don't commit it.
- `scripts/diag_ic4_z_channel.py` — Python-only analogue for hover-fail diagnosis using cached datasets (no MATLAB re-run).
