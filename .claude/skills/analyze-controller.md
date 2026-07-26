# /analyze-controller

Analyze the performance of a controller from the MATLAB comparison simulation.

## Usage

```
/analyze-controller <ctrl_id>     -- analyze controller 1-5
/analyze-controller <ctrl_id> <topic>  -- focus on a specific topic
```

Arguments:
- `ctrl_id` (required): Controller number 1-5
  - 1 = MDF-ASMC (Proposed)
  - 2 = Lin 2022 (PBVS+PPC)
  - 3 = Zhang 2026 (PBVS+AEDO)
  - 4 = Lin 2023 (robust IBVS + performance funnel)
  - 5 = Cho 2022 (FF-IBVS)
- `topic` (optional): Focus area -- one of `thrust`, `trajectory`, `attitude`, `convergence`, `observer`, `all` (default: `all`)

## Instructions

When this command is invoked, perform the following analysis by loading `result_ctrl_<ctrl_id>.mat` using Python (scipy.io.loadmat). Do NOT run MATLAB.

### Step 1: Load and validate data

```python
import scipy.io as sio
import numpy as np
d = sio.loadmat('MATLAB/Comparison/Datasets/result_ctrl_<N>.mat')
```

Extract key variables:
- `X_DS` (13 x N+1): UAV state [pos(3); quat(4); vel(3); omega(3)]
- `U_DS` (4 x N): Control inputs [tau_x; tau_y; tau_z; T]
- `x_t` (7 x N): Target state [pos(3); quat(4)]
- `dx_t` (6 x N): Target derivative [vel(3); omega(3)]
- `idx`: Last valid simulation step (simulation may break early)
- `tRange`: Time vector
- `dt`: Time step (0.01s)
- `V_X_DS` (24 x N): Visual features [image(12); analytical(12)]
- `D_DS` (17 x N): Desired signals [V_h_d(3); I_a_cd(3); E_crd(2); dE_cd(3); B_w_cd(3); B_dw_cd(3)]

**Always slice to `idx` valid steps**: use `X[:, :idx]`, `U[:, :idx]`, etc.

### Step 2: Basic health check (always do this first)

Report these metrics:
1. **Simulation duration**: `idx * dt` seconds out of `tend` (40s). If < 40s, the sim either landed or broke early.
2. **Final position error**: `norm(X_DS(1:3, idx) - x_t(1:3, idx))` in meters
3. **Landing achieved?**: Unified criterion -- altitude < 0.20 m AND horizontal error < 0.30 m.
4. **NaN/Inf check**: Any NaN or Inf in X_DS, U_DS?
5. **Thrust range**: min/max of `U_DS(4, :idx)`. Physical limits are [0, 60] N. Values outside this range indicate a bug (see known issues below).
6. **Torque range**: min/max of `U_DS(1:3, :idx)`. Limits: tau_xy in [-1.85, 1.85] Nm, tau_z in [-1.0, 1.0] Nm.
7. **Altitude profile**: min/max of `X_DS(3, :idx)`. NED convention: negative = above ground. Positive values mean UAV went underground.

### Step 3: Topic-specific analysis

#### thrust
- Plot or report `U_DS(4,:)` statistics: mean, std, percentiles
- Count samples outside [0, 60] N -- these are physically infeasible
- Check for sign flips (indicates ground effect singularity bug)
- Compare logged thrust with expected: `T_expected = norm(m * I_a_cd)` where `I_a_cd = D_DS(4:6, :)` and `m = 2.114`
- If mismatch, check ground effect formula: the old code used `1/(1-(r/(4*x_c(3)))^2)` which blows up when `|z| < r/4 = 0.01875m`. The fix uses `z_ge = -max(abs(x_c(3)), r)`.

#### trajectory
- Position error over time: `X_DS(1:3,:) - x_t(1:3,:)` in each NED axis
- Velocity error over time: `X_DS(8:10,:) - dx_t(1:3,:)`
- Distance to target over time: `norm(pos_error, axis=0)`
- Check if trajectory is monotonically converging or oscillating

#### attitude
- Extract Euler angles from quaternion: `X_DS(4:7,:)` using scipy Rotation
- Check max roll/pitch angles -- should stay small (< 30 deg) for safe landing
- Angular velocity magnitude: `norm(X_DS(11:13,:), axis=0)`

#### convergence
- Time to reach various distance thresholds: 1m, 0.5m, 0.2m (landing)
- Settling time for position error < 0.1m
- Overshoot in altitude (does z go positive / below ground?)
- Terminal velocity at landing: `norm(X_DS(8:10, idx))`

#### observer / funnel internals (controllers 3, 4)
- For Zhang2026 (ctrl 3): AEDO divergence check -- compare `I_a_cd` magnitude over time. If it grows without bound, the observer is unstable.
- For Lin2023 (ctrl 4): check the circle-moment feature `s_t_lin` and the steady lateral image offset. Lin 2023 has no disturbance-rejection integrator, so a persistent lateral offset under wind is expected (never precise), with FoV loss on moving targets.

### Step 4: Known issues checklist

Check against these known bugs discovered during development:

1. **Ground effect singularity** (all controllers 2-5): Old code `1/(1-(r/(4*x_c(3)))^2)` has no altitude clamping. When `|z| < r/4 = 0.01875m`, denominator goes negative, flipping thrust sign. Fix: `z_ge = -max(abs(x_c(3)), r)`. Check if result file was generated before or after fix by comparing `U_DS(4,:)` with `norm(m*I_a_cd)`.

2. **AEDO feedback loop** (Zhang2026): `F_c_prev = m*(I_a_cd + g)` feeds back unsaturated acceleration. If AEDO diverges, `Fd_hat` grows, making `Fc` larger, which makes `I_a_cd` larger, feeding back into `F_c_prev`. The saturated thrust doesn't help because `I_a_cd` is computed before saturation.

3. **Lin2023 feature sign + conditioning**: the literal area feature `an = sqrt(a*/a)` requires the `+k1` virtual-velocity sign (image-feature dynamics carry an inversion vs position dynamics; the `-k1` mirror of Lin 2022 makes the UAV climb/drift). `k1` must be per-axis (literal depth error ~12 vs lateral ~0.03). The steady lateral offset is structural (no disturbance-rejection integrator) -- not a tuning bug.

4. **Logging vs saturation mismatch**: `I_a_cd = Fc/m` is logged BEFORE thrust saturation. So `norm(m*I_a_cd)` can exceed T_max even though the actual applied thrust is clamped.

5. **Delay offset**: With `delay=1`, `U_DS(:,idx) = u_2_buf(:, idx-1)`. The logged control at step i was actually computed at step i-1.

### Step 5: Summary

Provide a concise assessment:
- Overall: converged / diverged / oscillating / broke early
- Position accuracy at termination
- Control effort (mean thrust, peak torques)
- Any anomalies found
- Recommended fixes if issues detected

## Key constants (from Constants.m and InitVar.m)

```
m = 2.114 kg          r = 0.075 m (rotor radius)
J = diag([0.0256, 0.0256, 0.0440])
g = [0; 0; 9.81]     (NED)
T_max = 60 N          T_min = 0 N
tau_xy_max = 1.85 Nm   tau_z_max = 1.0 Nm
dt = 0.01 s            tend = 40.0 s
delay = 1 step         GE = 1 (enabled)
sigma_pos = 0.01 m     sigma_vel = 0.02 m/s
zf = 0.20 m (landing altitude); horizontal tol = 0.30 m
Initial: [2, 2, -5] m  (5m above origin)
```

## File locations

- Result files: `MATLAB/Comparison/result_ctrl_{1-5}.mat`
- Controller sources: `MATLAB/Comparison/ctrl_*.m`
- Main sim: `MATLAB/Comparison/visualControl_comparison.m`
- Constants: `MATLAB/Comparison/Constants.m`
- Init: `MATLAB/Comparison/InitVar.m`
- Gains: `MATLAB/Comparison/InitGains_Comparison.m`
