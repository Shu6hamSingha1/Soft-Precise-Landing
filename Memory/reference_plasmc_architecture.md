---
name: PLASMC architecture and parameter contributions
description: Principled understanding of every PLASMC parameter — control architecture, mathematical role, and contribution to soft/precise/robust landing. Companion to the per-parameter empirical reference.
type: reference
---

This is the *theoretical* companion to `reference_plasmc_parameters.md`. The other file contains empirical impact + current values; this one explains *why* each parameter is in the controller and *what role* it plays in the architecture. Use this as the source of truth when writing the manuscript's controller-design section.

## Control architecture (3 nested loops + parallel yaw)

```
[image features s, h, w, dw]
        │
        ▼
  Visibility funnel (forces image features to stay in FOV)
        │
        ▼
  Outer-loop PID on barrier-transformed visibility error → V_dh_d (desired optical-flow rate)
        │
        ▼
  Optical-flow funnel (forces flow tracking error → 0 by landing)
        │
        ▼
  Sliding mode + adaptive law → V_a_cd (desired body-frame accel)
        │
        ▼
  Cone clamp (30°/35°) → I_a_cd (inertial accel command)
        │
        ▼
  Inverse kinematics → desired roll/pitch/yaw + thrust
        │
        ▼
  Roll/pitch PID + Yaw ASMC (parallel)
        │
        ▼
  Angular rate PID
```

The funnel-based design is the core idea: instead of asking *"is the error small?"* the controller asks *"is the error inside this shrinking time-varying envelope?"* The envelope is enforced via a logarithmic barrier transformation `zeta = (1/2) ln((p+e)/(p-e))` that goes to ±∞ as `e → ±p`. Sliding mode + adaptive law provides the authority to keep `e` inside `p(t)`. As `p(t)` shrinks toward `p_inf`, the error is forced to shrink with it — *prescribed performance*.

## Layer 1 — Visibility funnel (FOV preservation)

**Goal:** keep image features inside camera FOV throughout the entire landing. If features leave the image, the IBVS system is blind and the landing fails.

| Param | Math role | S/P/R contribution |
|---|---|---|
| `p_10` | Initial visibility bound (from current FOV reach) | **Robust:** initial slack — must be wide enough that the actual feature error fits inside it (else `e/p > 1` → `log(neg)` → NaN at t=0). |
| `p_1inf` | Terminal visibility bound | **Precise:** caps steady-state image-plane error. Tight = features must converge to a small image region by landing. `[0.2, 0.2]` → 0.4×0.4 normalized window. |
| `gamma_1` | Decay rate of `p_1(t) = (p_10 - p_1inf)e^(-gamma_1 t) + p_1inf` | **Soft vs Robust trade:** fast decay forces aggressive visibility convergence early but punishes any slow startup; slow decay gives breathing room. `0.2` is moderate. |

## Layer 2 — Outer-loop PID on barrier-transformed visibility error `zeta_1`

**Goal:** convert the constrained visibility error into a desired optical-flow command `dh_d`. This is the *reference generator* for the inner sliding-mode loop.

| Param | Math role | S/P/R contribution |
|---|---|---|
| `zp` | P-gain on `zeta_1` | **Precise:** sets visibility-loop bandwidth. Bigger = faster image-plane convergence. |
| `zi` | I-gain on `zeta_1` | **Precise:** removes biases (camera offset, target localization errors). Small (0.1) — funnel itself does most of the bias rejection. |
| `zd` | D-gain on `dzeta_1` (computed from velocity) | **Soft:** primary lateral *damping* knob. Without `zd`, optical-flow command is purely proportional and the UAV oscillates laterally. **Empirical limit:** `zd ≤ 1.3` — above that, velocity-derivative noise spikes propagate into the command and trip the cone clamp on Run 4. |

## Layer 3 — Optical flow funnel (tracking error envelope)

**Goal:** force the *velocity* tracking error to converge inside a shrinking envelope. This is the prescribed performance constraint that gives PLASMC its name.

| Param | Math role | S/P/R contribution |
|---|---|---|
| `gamma_2` | Decay rate of `p_2(t)` | Sets how fast the velocity envelope tightens. Faster = forces faster convergence but reduces noise margin. |
| `p_20(1:2)` | Initial bound on lateral optical-flow error | **Robust:** controls *initial xy aggressiveness* via the barrier. Bigger `p_20(1:2)` → smaller `e/p` → smaller `zeta_2` → softer initial command → less cone-clamp engagement. **Counter-intuitive:** loosening this *helps* off-axis ICs because it prevents lateral demand from stealing vertical thrust. |
| `p_20(3)` | Initial bound on vertical optical-flow error | **Soft:** controls *vertical authority*. Tighter = larger `zeta_2(z)` → more vertical command → less initial climb. **Opposite direction to xy.** |
| `p_2inf(1:2)` | Terminal lateral envelope | **Precise:** caps steady-state lateral velocity error. |
| `p_2inf(3)` | Terminal vertical envelope | **Soft:** caps the descent rate at touchdown. The actual touchdown `v_z ≈ 0.3-0.5 m/s` is well inside the `2.0` envelope. |

## Layer 4 — Sliding mode equivalent + switching control

**Goal:** provide the actual control authority that forces `zeta_2` into the sliding manifold, regardless of uncertainties.

```
sigma   = dot(zeta_2) + (something)*zeta_2     [sliding surface]
u_eq    = G_2 * (-c + S_2*dp_2) - Omega*zeta_2 [nominal model-based]
u_sw    = -Gamma*sigma - Theta_norm * sat(E\sigma) * G_2 * kappa  [robust]
V_a_cd  = -G_2\(u_sw + u_eq)                    [body-frame accel]
```

| Param | Math role | S/P/R contribution |
|---|---|---|
| `Omega` | **Direct linear feedback on `zeta_2` inside `u_eq` — NOT a damping term despite the name.** Stiffness on the transformed error. | **Precise:** small bias rejection on `zeta_2`. **Robustness hazard:** bumping it amplifies initial off-axis demand and feeds the cone-clamp coupling. Treat as fine-grained P-trim only. |
| `Gamma` | Linear gain on `sigma` in `u_sw` — sets sliding-mode bandwidth. | **Soft + Precise:** primary tracking-accuracy knob. Bigger = faster sliding-mode convergence ⇒ tighter tracking. **Failure modes:** too big = chatter, low-altitude noise blows up Run 4. Sweet spots: `Gamma(1:2)=0.25`, `Gamma(3)=0.4`. |
| `E` | Boundary-layer matrix in `sat(E\sigma)`. Wider E ⇒ smoother saturation, less switching. | **Soft (chatter suppression) + Robust:** trades tracking precision for noise immunity. `E(1:2)=2.5` (smooth xy), `E(3)=0.5` (crisp vertical — descent cannot afford slow correction). |

## Layer 5 — Adaptive disturbance compensation

**Goal:** the unknown bound on disturbances (wind, ground effect, model error, computational delay) is estimated online via `kappa(t)`. This is what makes PLASMC handle ground effect *without explicit modeling*.

```
dkappa/dt = Theta_norm * N * G_2 * |sigma|  -  N * P * kappa
```

The first term grows `kappa` when sliding-surface error is large; the second term leaks `kappa` back when the system is settling. The bounded `kappa` then enters `u_sw` as the magnitude of the switching control.

| Param | Math role | S/P/R contribution |
|---|---|---|
| `N` | Growth rate gain on `|sigma|` | **Robust:** how fast the controller learns the disturbance bound. Big N = fast learning but risk of over-estimating and chattering. Very sensitive empirically. |
| `P` | Leakage rate (forgetting) | **Robust:** prevents `kappa` wind-up when system is steady. Big P = strong forgetting (responsive but loses learned info); small P = persistent estimate. `P(3)=5.0` gives strong vertical forgetting because vertical disturbance changes drastically (ground effect). |
| `kappa_0` | Initial value of adaptive gain — pre-loads the disturbance estimate before any error has accumulated | **Robust startup:** absorbs the initial transient before the adaptive law has time to grow `kappa` from zero. **This is why the adaptive controller can start cold without an initial spike** — the pre-load gives immediate authority. |

## Layer 6 — Cone clamp (the dominant nonlinearity)

```
if I_a_cd(3,idx) >= 0:  I_a_cd(3,idx) = -1.0           # forces downward thrust
a_xy_limit = |I_a_cd(3,idx)| * tan(att_cone)            # max allowed lateral component
if |a_xy| > a_xy_limit:  scale a_xy down                # preserve vertical
```

This isn't a "tuned gain" in the classical sense — it's a *hard constraint* that maps the desired acceleration to an achievable one given that the UAV can only thrust along its body-frame z-axis. **It is the single most important nonlinearity in this controller for soft landing**, because nearly every divergence observed in tuning traces to: aggressive lateral demand → cone clamp scales lateral down → vertical stays unchanged → UAV can't track the (unfeasible) original demand → optical flow error grows → climb. The 30° cone is an implicit attitude bound. Widening to 35° gives more lateral authority on chasing ICs at the cost of bigger attitude excursions.

## Layer 7 — Roll/pitch attitude PID

**Goal:** track the roll/pitch setpoints derived from `I_a_cd` via inverse kinematics. Standard 2-DOF PID, classical and well-behaved.

| Param | Role | Contribution |
|---|---|---|
| `ep` | Attitude P-gain | Bandwidth of attitude loop. Must be much faster than outer loop (~10×) for cascade stability. |
| `ei` | Attitude I-gain | Trim/offset rejection (mass uncertainty, CG offset). |
| `ed` | Attitude D-gain | Damping of attitude loop. |

Robust enough that nothing else broke it during tuning.

## Layer 8 — Angular rate PID + feedforward

**Goal:** track the body angular rate `B_w_cd` from `dE_cd`. Innermost loop, fastest dynamics.

| Param | Role | Contribution |
|---|---|---|
| `wp, wi, wd` | Standard rate PID | Sets rate-loop bandwidth, must be ≥10× attitude-loop bandwidth. |
| `ff` | Feedforward of desired rate | Removes rate-loop tracking lag from the attitude loop's perspective. |

## Layer 9 — Yaw adaptive SMC (parallel to roll/pitch PID)

**Goal:** the same prescribed-performance / adaptive-SMC philosophy applied to yaw, because yaw uses radically different dynamics (no gravity bias, no cone clamp) but is **critical for thrust alignment**.

```
sigma_a = e_a + Omega_a * ie_a       [integral-augmented surface]
u_a     = Gamma_a * sigma_a + sat(sigma_a/E_a) * kappa_a + Omega_a * e_a
```

| Param | Role | S/P/R contribution |
|---|---|---|
| `Omega_a` | Integral gain inside `sigma_a` and direct error feedback in `u_a` (appears twice) | **Precise:** drives steady-state yaw error to zero. Heavy because uncompensated yaw bias misaligns thrust → couples into xy/z. |
| `Gamma_a` | Linear gain on `sigma_a` | **Robust:** linear yaw authority. Yaw stiffness directly buys vertical performance via thrust alignment. |
| `n_a, p_a` | Adaptive law growth/leakage | **Robust:** as for outer adaptive law. |
| `kappa_a_0` | Initial yaw adaptive gain | Pre-load for startup. |
| `E_a` | Yaw boundary layer | **Soft:** smooths yaw control near sliding surface. Combined with high `Gamma_a`, prevents chatter while keeping authority. |

## How the parameters jointly produce *soft + precise* landing

**Soft** (small touchdown velocity) requires:
- `gamma_2`, `p_2inf(3)` define the descent envelope shape (funnel design)
- `Gamma(3)` provides sliding-mode authority to track the envelope
- `E(3)` (small) keeps vertical correction crisp
- `kappa_0(3)`, adaptive law absorbs ground-effect transient near landing
- `zd(1:2)` damps lateral oscillation that would couple into vertical via cone clamp

**Precise** (small touchdown xy) requires:
- `p_1inf` forces image features into a small terminal window
- `p_2inf(1:2)` caps lateral velocity error
- `zp` provides outer-loop bandwidth
- `Gamma(1:2)` provides sliding-mode tracking authority
- `Gamma_a, Omega_a` keep yaw aligned so commanded xy is actual xy

**Robust** (across ICs, target motions, noise) requires:
- `p_20(1:2)` (loose) prevents aggressive xy startup → cone-clamp coupling
- `p_20(3)` (tight) prevents weak vertical authority → climb
- Adaptive law (`N`, `P`, `kappa_0`) handles unknown disturbance bounds
- Boundary layer `E` and yaw `E_a` filter measurement noise
- Cone clamp angle is the safety valve when lateral demand exceeds vertical headroom

## The two coupling lessons that dominate this controller

1. **Cone-clamp coupling:** lateral demand and vertical thrust share the same actuator (motors). Excessive lateral demand silently steals vertical authority. *Most failures observed in tuning were instances of this coupling, not direct loop instability.*

2. **Yaw-thrust coupling:** yaw error rotates the body frame, mis-aligning the commanded thrust direction. A 5° yaw error makes the UAV climb instead of descending in pure vertical commands. *Stiffening yaw alone has fixed runs that looked like outer-loop failures.*

Both couplings explain why off-axis ICs (especially the chasing direction on moving targets) are the universal stress test.

## Use this file when writing the manuscript

- **Controller design section:** use the layer-by-layer architecture diagram and the per-layer goal statements as the narrative spine.
- **Parameter table in the paper:** combine the math-role column from this file with the empirical-impact column from `reference_plasmc_parameters.md`.
- **Discussion section on coupling:** the two coupling lessons at the bottom of this file are the key insights to highlight — they're the "why this controller is non-trivial" story.
