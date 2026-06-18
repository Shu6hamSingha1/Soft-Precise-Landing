---
name: PLASMC parameter impact reference
description: Per-parameter role in PLASMC math + empirical impact from multi-init tuning. Source of truth for manuscript parameter discussion.
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
PLASMC = funnel-transformed adaptive sliding mode (outer loop) + 2-DOF roll/pitch PID + yaw ASMC (inner loop).
Outer-loop math reference: `visualControl_IBVS_adaptive_temp.m` lines 350-395.
Observations gathered from multi-init tuning on Static, Linear, Sinusoidal, and Lissajous targets, ICs ∈ {[0,0,-5],[0,0,-7],[0,0,-3],[2,2,-5],[-2,-2,-5]}.

## Outer-loop (funnel + sliding mode)

### Visibility funnel (image-feature error `zeta_1`)

| Param | Role | Empirical impact | Current |
|---|---|---|---|
| `gamma_1` | Visibility funnel decay rate `p_1(t)=(p_10-p_1inf)e^(-gamma_1 t)+p_1inf` | Not retuned. Faster decay = aggressive visibility convergence requirement. | `[0.2, 0.2]` |
| `p_10` | Visibility funnel initial bound | Inherited from `K` struct (image-plane reach). | from `K.p_10` |
| `p_1inf` | Visibility funnel terminal bound | Tight terminal bound forces image features inside narrow envelope at landing. | `[0.2; 0.2]` |

### Outer-loop PID on transformed visibility error `zeta_1`

| Param | Role | Empirical impact | Current |
|---|---|---|---|
| `zp` | P-gain on `zeta_1` → drives `dh_d` (desired optical flow) | **5.0 → 4.0 (Linear Run 5 IC=[-2,-2,-5] lands; was failing via cone-clamp climb-coupling at t≈11.5s).** zp is the dominant bandwidth knob for the lateral chase demand. Run 5 has the worst chase geometry (opposite to target motion → max horizontal travel) so it's the canary for cone-clamp saturation. Lowering zp directly softens the lateral demand → cone clamp stops saturating → no climb → vertical funnel never saturates. Other 4 runs unaffected (slack to spare). Static unchanged 5/5. | `diag(4.0,4.0)` |
| `zi` | I-gain on `zeta_1` | Not retuned. Small (0.1) — only mild bias rejection. | `diag(0.1,0.1)` |
| `zd` | D-gain on `zeta_1` (uses raw differenced velocity → noise-prone) | **1.2 → 1.6 (Run 4 diverges via noise spike → cone clamp → climb). 1.2 → 1.3 (5/5 land, Run 4 final_xy 0.062→0.030, climb 1.31→0.82 ✓).** Sweet spot is 1.3 — adds enough lateral damping to halve touchdown xy error without provoking the noise-spike cone-clamp failure. Above 1.3, velocity-derivative noise dominates. | `diag(1.3,1.3)` |

### Optical flow funnel (`zeta_2 = h - h_d` transformed)

| Param | Role | Empirical impact | Current |
|---|---|---|---|
| `gamma_2` | Optical flow funnel decay rate (3D) | Not retuned. | `[0.2, 0.2, 0.2]` |
| `p_20` | Optical flow funnel initial bounds — sets allowable startup error envelope. Affects xy and z in *opposite* ways through the cone-clamp coupling. | **Vertical (`p_20(3)`):** 8.0 → 5.0 → 3.5 → 5.0. Tighter z bound → more vertical authority → less climb. But went too tight: at 3.5 the barrier saturated on Linear Run 5 (G_2 singularity at t=1.76s). Reverted to 5.0 which is safe under the current cone clamp (35°). **Horizontal (`p_20(1:2)`):** 12 → 16 → 12. The 16 attempt fixed Linear Run 5 but broke Static Run 4 in 0.53s — single knob couldn't satisfy both. The real Linear fix turned out to be `zp: 5→4` (lateral bandwidth) instead. Reverted xy bound to 12. **Tuning rule (refined): `p_20(3)` ≈ 5 is a safe vertical authority level; for horizontal cone-clamp coupling, prefer reducing `zp` over loosening `p_20(1:2)`.** | `[12; 12; 5.0]` |
| `p_2inf` | Optical flow funnel terminal bounds | **Originally `[2,2,3]` → tightened to `[1.5,1.5,2.0]` for stricter terminal envelope. Improved touchdown precision without affecting stability.** | `[1.5; 1.5; 2.0]` |

### Sliding-mode equivalent + switching control

| Param | Role | Empirical impact | Current |
|---|---|---|---|
| `Omega` | **Direct P-feedback on `zeta_2` inside `u_eq = G_2*(-c + S_2*dp_2) - Omega*zeta_2`. NOT a damping term despite the name.** | **2026-04-08: Bumped `Omega(1:2): 0.005 → 0.02` (4×) → 5/5 → 3/5. Centered ICs unaffected (zeta_2≈0); off-axis ICs amplified initial xy demand → cone clamp coupling → climb 1.31→1.85m (Run 4) and divergence (Run 5). Reverted.** Use very sparingly; treat as fine-grained P-trim, not damping. | `diag(0.005,0.005,0.01)` |
| `Gamma` | Linear sliding-surface gain `u_sw = -Gamma*sigma - ...`. Sets sliding-mode bandwidth. | **`Gamma(1:2)`: 0.2→0.3 broke Run 4 at 7.55s; reverted to 0.25. `Gamma(3)`: 0.3→0.5 initially caused chatter on Run 4 with prior baseline. Reverted to 0.4. After locking `zp=4` and `p_20(3)=5.0`, retried `Gamma(3): 0.4 → 0.5` — strict win on all 3 trajectories. Sinusoidal Run 4 went from G_2 break at 10.63s to clean landing at 7.62s; Static precision improved (mean xy 0.014→0.005m).** Faster vertical sliding-mode = land before low-altitude optical-flow noise (`v/z` as z→0) can saturate the barrier. The earlier chatter was a symptom of the prior gain set's overall instability — once the lateral and funnel knobs were right, Gamma(3)=0.5 became safe. | `diag(0.25,0.25,0.5)` |
| `E` | Boundary-layer matrix on `sat(E\sigma)` — wider E ⇒ smoother control, narrower ⇒ more switching authority | Not retuned this session. `E(3)=0.5` keeps vertical switching crisp; `E(1:2)=2.5` smooths xy. | `diag(2.5,2.5,0.5)` |

### Adaptive law `dkappa/dt = Theta_norm·N·G·|sigma| − N·P·kappa`

| Param | Role | Empirical impact | Current |
|---|---|---|---|
| `N` | Growth gain on `|sigma|` — faster N ⇒ kappa rises faster | **Tested `N(3): 0.05→0.1` together with P/kappa_0 retune → divergence in 2s on off-axis ICs. Reverted.** Adaptive law extremely sensitive to multi-knob bumps. | `diag(0.02,0.02,0.05)` |
| `P` | Leakage rate on kappa — higher P ⇒ kappa decays faster (less integral wind-up) | **Tested `P(3): 5→3` together with N/kappa_0 retune → divergence. Reverted.** Cannot tune P alone without simultaneously rebalancing N. | `diag(1.5,1.5,5.0)` |
| `kappa_0` | Initial value of adaptive gain. Pre-loads disturbance compensation. | **`kappa_0(3): 0.1→0.2` → Run 4 stops oscillating at low altitude (climb 0.107m, lands cleanly 4/5 with conservative Gamma).** A pre-loaded kappa absorbs startup transient that the adaptive law's lag would otherwise miss. | `[0.1; 0.1; 0.2]` |

## Inner loop

### Roll/pitch attitude PID (2-DOF on `E2_e = E_cr(1:2)' - E2_crd`)

| Param | Role | Current |
|---|---|---|
| `ep` | Attitude P-gain | `diag(5,5)` |
| `ei` | Attitude I-gain | `diag(0.1,0.1)` |
| `ed` | Attitude D-gain | `diag(0.1,0.1)` |

Not retuned this session. Stable across all tested gain configurations.

### Angular-rate (rate-loop PID + feedforward)

| Param | Role | Current |
|---|---|---|
| `wp` | Rate P-gain | `diag(5,5,5)` |
| `wi` | Rate I-gain | `diag(0.01,0.01,0.1)` |
| `wd` | Rate D-gain | `diag(0.1,0.1,0.2)` |
| `ff` | Rate feedforward | `diag(0.1,0.1,0.1)` |

Not retuned. Highest yaw rate gains because yaw uses the ASMC channel for the *attitude* level only — the rate loop still runs the standard PID.

### Yaw adaptive SMC (`u_a = Gamma_a*sigma_a + sat(sigma_a/E_a)*kappa_a + Omega_a*e_a`, `sigma_a = e_a + Omega_a*ie_a`)

**Critical finding (user-reported 2026-04-08, empirically confirmed):** *Any overshoot in yaw control has huge impact on lateral and vertical control performance* — yaw error rotates the body frame, mis-aligning thrust and propagating into xy/z error. **Confirmed by experiment**: stiffening yaw (`Gamma_a 0.1→0.3`, `E_a 1.5→2.5`) eliminated Run 5's 1.09 m initial climb entirely and halved Run 5 touchdown velocity, with no other change to vertical/lateral gains.

| Param | Role | Empirical impact | Current |
|---|---|---|---|
| `Omega_a` | Integral gain inside `sigma_a` AND direct feedback in `u_a` (appears twice) | **2026-04-08: 1.0 → 1.5 on Circular** — improved Run 5 break time from 5.06s → 8.03s (~2x) by adding integral action against the rotating-target ramp yaw setpoint. `Omega_a=2.0` blew up at t=0.47s (integral wind-up at startup); 1.5 is the safe ceiling. **No regression on Linear/Static/Sinusoidal/Lissajous** — those have zero target yaw demand so the larger integral term has nothing to integrate. | `1.5` |
| `Gamma_a` | Linear sliding-surface gain on `sigma_a` (P-like) | **2026-04-08: 0.1→0.3 — direct linear yaw authority (replaces reliance on slow adaptive `kappa_a`). Combined with `E_a` widening, this eliminated Run 5 climb entirely (1.09→0.00 m) and cut Run 5 landing time by 1.4s. Empirically confirms yaw-thrust coupling: faster yaw → less initial thrust misalignment → less initial climb on off-axis ICs.** Tried 0.3→0.4 with `Omega_a=1.5` on Circular — *regression* (Run 5 dropped from 8.42s → 6.43s). Gamma_a and Omega_a are not independent: bumping both at once over-stiffens. | `0.3` |
| `n_a` | Adaptive law growth gain `dkappa_a/dt = n_a*|sigma_a| - p_a*kappa_a` | **2026-04-08 sweep on Circular (Omega_a=1.5):** 0.05 → 0.1 improved Run 5 from 8.03s → 10.40s (yaw-quiet trajectories: no benefit but no harm). 0.1 → 0.2 *regression* to 9.59s (over-aggressive late, amplifies optical-flow noise as z→0). **However, `n_a=0.1` broke Linear Run 5 (3.92s)** — same trade-off pattern as `E_a=1.5`: tighter yaw discipline helps rotating targets but is too aggressive for yaw-quiet trajectories where there's no real yaw error to correct, only noise. **Reverted to 0.05.** Cannot strict-win Circular without sacrificing Linear via this knob alone. | `0.05` |
| `p_a` | Adaptive law leakage rate — higher = `kappa_a` decays faster | **2026-04-08 sweep on Circular (n_a=0.1):** `p_a=1` (less leakage, kappa_a accumulates noise late) → Run 5 8.89s, regression. `p_a=3` (more leakage, kappa_a decays before disciplining) → Run 5 collapses to 4.43s, major regression. **Default `p_a=2` is the sweet spot in both directions** — leakage rate is finely balanced against `n_a` growth rate to prevent both wind-up and starvation. | `2` |
| `kappa_a_0` | Initial yaw adaptive gain — only affects t=0 transient | **2026-04-08 test on Circular (Omega_a=1.5, n_a=0.1):** 0.1 → 0.3 → Run 5 *regression* 10.40s → 8.66s. Over-aggressive yaw at t=0 perturbs the early off-axis transient on the worst-case IC. As theoretically expected, `kappa_a_0` only affects t=0 — late-landing failures are unreachable through this knob. | `0.1` |
| `E_a` | Yaw boundary layer | **2026-04-08: widened 1.5→2.5 — suppresses chatter-driven yaw overshoot. Wider boundary layer + larger linear `Gamma_a` is the right combination: Gamma_a provides authority, E_a prevents that authority from chattering at the surface.** | `2.5` |

## Tuning principles distilled

1. **Centered ICs are not a stress test.** Always validate against `[2,2,-5]` (Run 4) and `[-2,-2,-5]` (Run 5). Many gain bumps look fine on Runs 1–3 then break Run 4 catastrophically.

2. **Run 4 vs Run 5 asymmetry** under identical conditions ⇒ the failure is *noise-realisation-driven*, not deterministic. Need a margin against the worst-case noise spike.

3. **The cone clamp is the dominant nonlinearity.** Aggressive xy demand → `|a_xy| > |az|tan(30°)` clip → vertical thrust starves → UAV climbs → spiral failure. Almost every divergence in this session was a cone-clamp climb chain.

4. **Multi-knob adaptive-law tuning is unstable.** Bumping `N`, `P`, `kappa_0` together always diverged. Single-knob, conservative steps are mandatory for the adaptive law. Outer PID/Gamma can take larger steps.

5. **Climb vs landing-altitude noise robustness is a real trade-off.** Tighter `p_20(3)` cuts climb but pushes the barrier toward saturation. Current sweet spot: `p_20(3)=5.0`, `Gamma(3)=0.5`, `kappa_0(3)=0.2`, `zp=4` — 5/5 land on Static/Linear/Sinusoidal with ~1.0 m climb on off-axis ICs. Cut climb via lateral softening (lower `zp`) rather than tightening `p_20(3)`, which is the safer side of the trade.

6. **Damping knobs in this controller, ranked by noise quality:**
   - **Best:** widening `E` (boundary layer) — smooths chatter without amplifying noise
   - **Good:** lowering `Gamma` — proportional reduction
   - **Moderate:** raising `zd` — useful but caps at ~1.3 due to velocity-derivative noise
   - **Avoid for damping:** raising `Omega` — it's a P-gain in disguise, amplifies error not its derivative

7. **Yaw-thrust coupling is real and large.** Stiffening yaw (linear authority + boundary layer) gives lateral/vertical improvements as big as direct lateral/vertical tuning. Always tune yaw before declaring an outer-loop config "stuck" — yaw overshoot is often the hidden root cause of off-axis climb and slow xy convergence.

8. **`yaw_drift` is a symptom, not a cause, on failing runs.** When a run diverges, the UAV wobbles in 3D and yaw can swing several degrees regardless of yaw-loop tuning. A diverged run with `yaw_drift = -3.15°` looked like a yaw failure but the actual root cause was lateral cone-clamp coupling — fixing the lateral demand collapsed yaw_drift to 0.05° with no yaw gain change. **Diagnostic rule:** treat yaw_drift on failed runs as a divergence indicator, not a tuning target. Only treat it as a tuning target on *landed* runs that show large drift.

9. **Failure-by-timeout vs failure-by-divergence look similar in summary tables but need different fixes.** Always check `final_alt`, `final_xy`, `v_z`, `t_end` together. If `final_xy < 0.30 && v_z is sane && t_end ≈ tend && final_alt ≈ 0.25–0.5`, it's a *timeout*: the UAV is descending cleanly but ran out of time, usually from an early climb transient eating descent budget. Fix the climb (e.g. `p_20(1:2)`), not the convergence rate. If `final_xy >> 0.30 && v_z weird && t_end << tend`, it's a *divergence*: outer loop blew up. Fix the gain that's destabilizing (usually outer-loop or adaptive).

## Current best baseline (2026-04-08, Static + Linear + Sinusoidal + Lissajous, 5/5 each — 20/20 total)
```matlab
% outer loop
zp = diag(4,4);   zi = diag(0.1,0.1);   zd = diag(1.3,1.3);
gamma_2 = [0.2,0.2,0.2];   p_20 = [12;12;5.0];   p_2inf = [1.5;1.5;2.0];
Omega = diag(0.005,0.005,0.01);   Gamma = diag(0.25,0.25,0.5);
P = diag(1.5,1.5,5.0);   N = diag(0.02,0.02,0.05);
kappa_0 = [0.1;0.1;0.2];   E = diag(2.5,2.5,0.5);
% inner loop (defaults)
ep = diag(5,5);   ei = diag(0.1,0.1);   ed = diag(0.1,0.1);
wp = diag(5,5,5); wi = diag(0.01,0.01,0.1); wd = diag(0.1,0.1,0.2);
% yaw ASMC
Omega_a=1.5; Gamma_a=0.3; n_a=0.05; p_a=2; kappa_a_0=0.1; E_a=2.5;
% cone clamp
att_cone = deg2rad(35);
```
Static:     5/5 land, t∈[4.5,9.3]s,  mean final_xy 0.005m, max 0.010m.
Linear:     5/5 land, t∈[4.3,9.6]s,  mean final_xy 0.027m, max 0.038m.
Sinusoidal: 5/5 land, t∈[4.4,10.4]s, mean final_xy 0.059m, max 0.095m.
Lissajous:  5/5 land, t∈[4.3,9.5]s,  mean final_xy 0.031m, max 0.050m. (first-try, no tuning)

Recent changes vs prior baseline:
- `zp: 5 → 4` — softens lateral bandwidth → kills cone-clamp climb-coupling on Linear Run 5.
- `Gamma(3): 0.4 → 0.5` — accelerates vertical sliding-mode → lands before low-altitude optical-flow noise (`v/z` blowup as z→0) can saturate the barrier. Strict win on all 3 trajectories: Sinusoidal 4/5→5/5 (Run 4 G_2 break at 10.63s → clean landing at 7.62s); Static and Linear precision both improved (Static mean xy 0.014→0.005m, Run 5 hit exactly 0.000m).

---

## SO(3) geometric-inner-loop retune (2026-04-13, Static + Circular)

**Context**: Inner loop switched from PID cascade to direct geometric SO(3) (`tau = -kR*e_R - kOmega*B_w_c + w x Jw`). Outer loop preserved; all outer-loop gains retuned. Yaw ASMC law reinterpreted as a *heading-rate* generator (`psi_d += u_a*dt`) rather than a rate setpoint.

### Key findings

**1. `P` is the leakage, not `N` (adaptive law mislabeling).**
From `kappa_Solver.m`: `dkappa/dt = Theta_norm*N*G*|sigma| - N*P*kappa`. Both terms scale with `N`, so `N` sets the *time constant* (how fast κ reaches equilibrium). `P` alone controls the *ratio* (equilibrium κ_eq = Theta_norm*G*|sigma|/P). Raising N bumps *both* terms and barely changes steady state. Raising P cuts the ratio directly. When κ(3) was blowing up to 353, `N(3): 0.1→0.5` only got it to 263; `P(3,3): 5 → 50` bounded it to 51.
**Rule**: to bound a runaway kappa, raise P. To speed up adaptation, raise N. They are not interchangeable.

**2. `p_2inf(3)` must match the low-altitude image-noise floor.**
Tightening `p_2inf(3): 2.0 → 0.5` "for tighter terminal flare" causes **funnel-saturation bang-bang at z<1m**. `L_s^{-1}` scales as `1/Z`, so at Z=0.75m pixel-noise-amplified `V_h_e(3)` reaches ~1 m/s — way over a 0.5 floor → `sigma(3)` pinned at ±3.66 wall → outer loop fires `I_a_cd(3) ∈ {-3, -50}` bang-bang → thrust chatters 0↔60 N → UAV stalls in a limit cycle at ~0.55m. **Fix: revert `p_2inf(3)` to 2.0.** This is not a "loose" tuning — it's the physically necessary floor below which the funnel is incompatible with pixel-noise amplification.
**Rule**: `p_2inf(i)` has a lower bound set by `max(L_s^{-1}*pixel_noise)` at the smallest operational altitude. Tightening below that is structurally unstable.

**3. `h_rd` must be physically achievable against the cone clamp at the smallest operational altitude.**
`h_rd = −0.5` limit-cycles at 0.55m (demand unachievable). `h_rd = −0.1` stalls at 1.5m (demand too weak → UAV hovers). `h_rd = −0.4` bounces at 0.75m (marginal). `h_rd = −0.7` works cleanly once `p_2inf(3)=2.0` gives the funnel room to breathe.
**Rule**: once `p_2inf(3)` is set to its noise floor, `h_rd` can be pushed aggressively; the funnel no longer saturates, so `I_a_cd` commands stay smooth through touchdown.

**4. xy terminal precision is governed by `Gamma(1:2)` outer-loop adaptation rate, not funnel tightness.**
On Circular wz=1.0 with yaw ASMC active, terminal `xy_err` was stuck at ~10cm despite sub-3cm mid-cruise precision. The terminal growth came from yaw-lag-induced frame-rotation errors in the lateral commands, which the xy adaptive kappa was too slow (`Gamma(1:2)=[0.1875, 0.25]`) to reject. Raising `Gamma(1,1)=Gamma(2,2)=0.5` dropped terminal `xy_err` from 10.7cm → **2.85cm** (~4×) on the same run. No other knob touched, yaw loop identical.
**Rule**: when terminal xy_err is dominated by yaw-lag or other persistent disturbances, the right response is to accelerate the xy adaptive loop, not tighten the funnel or raise sliding-mode gain.

### Yaw ASMC under rotating targets (Circular wz stress test)

The heading-integrated yaw ASMC:
```
sigma_a = e_a + Omega_a * ie_a
u_a     = Gamma_a * sigma_a + sat(sigma_a/E_a) * kappa_a + Omega_a * e_a
psi_d  += u_a * dt
```

**Structural limit: image-moment α has period π.** Line 420 wraps `e_a = atan2(sin(2*e_raw), cos(2*e_raw))/2` to `[-π/2, π/2]`. For wz > ~1.5 rad/s, the target wraps the α-ambiguity faster than the yaw loop can slew, and `e_a` aliases — the controller chases measurement artifacts. **This is a hard sensor limit, not a tuning problem.** At wz=1.0 rad/s, the loop can still converge; at wz=1.5 rad/s it oscillates with 7+ zero crossings and lateral coupling diverges.

**Parameter interactions discovered at wz=1.0:**

| Knob | Safe range | Interaction |
|---|---|---|
| `Gamma_a` | 0.3–0.5 | Above 0.5 oscillates; below 0.2 too slow to catch startup transient. |
| `Omega_a` | 0.3–0.5 | At 0.8, integral wind-up triggers limit cycle with e_a bouncing ±20°. |
| `E_a` | 2.0–3.0 | `E_a=1.0` with large `kappa_a_0` creates ±kappa bang-bang on every sigma_a zero crossing. Widening smooths the sat term into near-linear. |
| `kappa_a_0` | Match wz | Pre-seed near expected DC rate. But: defeated by `p_a=2` leakage unless also lowered. |
| `n_a`, `p_a` | 1.0, 2.0 | Fast adaptation OK only with wide `E_a`. |

**Startup transient is structurally limited.** `psi_d` starts at 0; target rotates at wz. During the ~0.8s it takes `u_a` to ramp from 0 to wz, `e_a` accumulates `~wz*0.8` rad. No pure-tuning knob can reduce this below ~45° at wz=1.0. The two escapes (pre-seed `psi_d` from a prior estimate; pre-seed `kappa_a_0` with low `p_a`) are both fragile.

**Yaw–lateral coupling is the real killer.** When e_a stays >45°, the inertial-frame `V_h` command is computed through a wrong rotation matrix → xy outer loop commands point in the wrong direction → xy error grows → cone clamp fires → attitude swings → yaw disturbed further. On wz=1.5 this cascade diverges in <5s. On wz=1.0 it survives but terminal xy_err is yaw-lag-bounded.

### Current Circular wz=1.0 best (2026-04-13)

```matlab
% outer loop
zp = diag(5,5);   zi = diag(0.1,0.1);   zd = diag(1.3,1.3);
gamma_2 = [0.1,0.1,0.2];   p_20 = [12;12;5.0];   p_2inf = [1.0;1.0;2.0];
Omega = diag(0.005,0.005,0.01);   Gamma = diag(0.5,0.5,2.5);     % xy raised for terminal precision
P = diag(1.5,1.5,50.0);   N = diag(0.02,0.02,0.5);               % P(3) bounds runaway kappa(3)
kappa_0 = [0.125;0.125;5];   E = diag(0.5,0.5,0.5);
% geometric SO(3) inner loop
kR = diag(1.5,1.5,0.5);   kOmega = diag(0.3,0.3,0.1);
% yaw ASMC (heading-rate generator)
Omega_a=0.5;  Gamma_a=0.5;  n_a=1.0;  p_a=2;  kappa_a_0=2.0;  E_a=3.0;
% demand
h_rd = -0.7;   % achievable descent rate at lowest Z
% cone clamp
att_cone = deg2rad(35);   I_a_cd(3)_floor = -3;   I_a_cd(3)_cap = -50;
```

**Results (Circular wz=1.0, IC=[2,2,-5], single run):**
- Land time: **6.84s**
- Terminal xy_err: **2.85cm**
- e_a end: **−0.13°** (essentially converged)
- e_a peak: 48.7° (startup transient, structural lower bound)
- Zero crossings: 2 (stable)
- u_a max: 1.29 rad/s (well below wz headroom)
- `kappa_a` evolves 2.0 → 0.69 (leakage-bound equilibrium)

**Not yet validated**: multi-init Circular, higher wz, Linear/Sinusoidal/Lissajous with this new config. Some prior-baseline gains (`zp=4`, `gamma_2=0.2`, `Gamma(3)=0.5`, `kappa_0(3)=0.2`) were moved significantly; revalidate before claiming a strict improvement.
