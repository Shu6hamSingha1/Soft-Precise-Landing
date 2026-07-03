# PLASMC Control-Framework Review (PX4/Gazebo)

> ⚠ **STALENESS STAMP (2026-07-03):** this 2026-06-11 review concludes "control law sound + gain-exhausted;
> the binding limit is the perception front-end." That verdict is **SUPERSEDED** — the wall was a gain-parity
> bug + the dormant velocity-damping lever (`ζ_h`), and the terminal fly-away was substantially the Z_REG=0.01
> GT-FB harness artifact; with those fixed the control law reaches GT-FB 19/25 SP across IC1-5. The per-component
> map + the clamp audit remain accurate as an implementation reference. Live state: `px4/MEMORY.md` top +
> `PLASMC_TUNING_GUIDE.md` STATUS.

**Date:** 2026-06-11 · **Cal regime:** R3 (honest 8-run, `d60973a`) · **Scope:** `src/controller.py`, control-relevant `src/img_data.py`.

This is an implementation-and-performance review of the PLASMC controller as ported to PX4 SITL. It maps each
component, evaluates how it performed across the tuning campaign, and audits every clamp. It complements
[`PLASMC_TUNING_GUIDE.md`](PLASMC_TUNING_GUIDE.md) (the playbook) and [`CONTROLLER_PARITY.md`](CONTROLLER_PARITY.md)
(MATLAB↔Python diff).

> **Headline:** all three control components (lateral, yaw, descent) are correctly implemented and gain-exhausted.
> **Every binding failure traces to the close-range perception front-end, not the control law.** The clamp set is
> healthy; the only "masking" clamps are deliberately relaxed so they don't hide the real signal.

*Line numbers are accurate for `controller.py`; `img_data.py` cites are approximate (the file shifted under recent edits) but the clamp identities/values are current.*

---

## Architecture — the cascade

```
image s,h ─► OUTER (PPC funnel on s_e_n → ds_d) ─┐
                                                  ├─► h_d = ds_d + cross(w_i,s) + loom·s
measured optic-flow h ─────────────────────────── ┘
   ─► MIDDLE (barrier ζ→G, sliding σ, adaptive κ-ODE) ─► a_u (inertial accel, per-axis x,y,z)
   ─► YAW SMC (α-feature, κ_a, virtual compass ψ_d)
   ─► SO(3) handoff: R_d from a_u + ψ_d → e_R → body-rate w_u = −K_R·e_R + thrust B_T ─► MAVSDK
```

- **xy and z share** the middle SMC (one κ-ODE form, per-axis gains).
- **Yaw** is a separate scalar SMC feeding a virtual compass `ψ_d`.
- **Inner attitude/rate loop is PX4's** — we ship body-rate + thrust over MAVSDK.

---

## 1. LATERAL control — *law is sound, perception is the limit*

**Implementation**
- Outer PPC funnel: `s_e_n = (s−s_d)[:2]/p_10` (controller.py:531); barrier `ζ_s = log((1+r)/(1−r))`, `r=s_e_n/p_s`;
  `ds_d = G_s⁻¹(−K_rp·ζ_s − K_ri·∫ζ_s − K_rd·ζ̇_s) + S_s·ṗ_s` (534-565); EMA `TAU_DS=0.05` (588-597).
- `h_d = ds_d + cross(w_i,s) + (h_rd − dot(cross(w_i,s),e3))·s` (599-627).
- Middle SMC: barrier `ζ→G` (631-656), `σ=ζ+Ω∫ζ` (715), c-term regressor `θ` (717-733), κ-ODE (735-751),
  `a_u = −G⁻¹[Γσ + θ·diag(sat(σ/E))·G·κ + G(−c+Sṗ−Ωζ)]` (753-771).

**Performance**
- **Converges fast and tight when perception is clean** — early `|s_e_n|≈0.04`; dead-center 0.04 m hold at `E=2.5`;
  `KP=12/E=1.5` reached **0.34 m** (best ever). The law itself works.
- **Every failure is perception-driven.** The κ_xy-runaway (→7.26, a_u→631) is the off-screen virtual `s` (z_v→0)
  inflating θ_norm **multiplied by** the spurious `h` (ill-conditioned lstsq) breaching the funnel →
  `κ_eq=θ·G·|σ|/P=505`. All control levers (P, E, KP, γ_s, θ-freeze) were neutral or negative.
- `KP×E` coupling: `KP=9` with `E_xy≥2.5`; `KP=12` only with `E_xy≤1.5`.
- **`κ_xy` is UNCAPPED** (`KAPPA_MAX=1e6`) — the lateral runaway is deliberately unbounded to expose the cause.

**Verdict:** ✅ well-designed and **gain-exhausted** (`KP=9/E=2.5`≈3.8 m, `KP=12/E=1.5`≈0.34 m best). Binding limit
is the perception front-end.

---

## 2. YAW control — *sound and validated; IC failures are the test rig*

**Implementation** — `_yawCtrl` (773-887): α-feature 2π-wrapped error `e_a=atan2(sin Δα,cos Δα)` (786);
κ_a-ODE `dκ_a/dt=n_a|σ_a|−n_a p_a κ_a` (836-842); **conditional integration** — freeze `ie_a` when the heading-rate
is saturated (815-830); `u_a=Γ_a σ_a + sat(σ_a/E_a)κ_a + Ω_a e_a` (844-856), LPF `tau_ua=0.1`; virtual compass
`ψ_d=wrap(ψ_d+u_a·dt)` (878-887); `BODY_YAW_SOURCE=alpha` (drift-free, 926).

**Performance**
- **Sound + MATLAB-parity for the square-start** (the only start MATLAB validates).
- **Conditional integration fixed the windup→overshoot** (102°→−22°) — proper anti-windup replacing the removed
  fixed `_ie_a_clamp`.
- The IC2-5 "yaw runaway" is **EKF compass drift ~77° at descent start** (the rig gates on EKF yaw), **not the yaw
  law** — `alpha_start≈GT_yaw_start` every rep; three α-redesigns all hit 180° because the cause is the bad start.
  Fix the **test rig**, not the controller.
- **Yaw cal pending** (`cal_s[3]=1.0`) — inert for the stationary square board; needed for moving targets.

**Verdict:** ✅ correct and low-impact for stationary; moment-α is canonical (geometric swap reverted). IC2-5
failures belong to the rig.

---

## 3. DESCENT (z-axis) — *works; its role in failure is reaching the 1/Z zone*

**Implementation** — z shares the middle SMC with separate gains: `E_z=1.0`, `P_z=5`, `KAPPA0_Z=1.0`,
`KAPPA_MAX_Z=3.0`; loom ref `h_rd≈−0.30…−0.42` enters `h_d`; ring-divergence is the robust vertical signal.

**Performance**
- **`KAPPA0_Z=1.0` bootstrap fixed the no-descent** (descent starts in 0.4–0.9 s vs the 25–43 s hover).
- **Residual 1/5 hover:** `σ_z` stays inside the `E_z=1.0` boundary → `a_u_z≈1` too weak to start the fall.
  `E_z=0.5` fixes it **but un-bounds κ at touchdown** — the **one-knob-one-job** problem (E_z sets descent stiffness
  *and* κ-bound).
- **Descent is the *enabler* of the lateral failure:** by reaching low altitude it drives the system into the 1/Z
  perception-breakdown zone (spurious `h` + off-screen `s`) that triggers the lateral κ-runaway. Descent itself
  rarely fails.
- `KAPPA_MAX_Z=3.0` caps the z-runaway (a backstop).

**Verdict:** ✅ functional (bootstrapped, `E_z=1.0` baked); the rare hover + the `E_z` stiffness↔κ-bound trade are
unresolved, but descent's main coupling to TLs is *reaching* the perception-breakdown altitude.

---

## 4. Clamp audit — by role

### A. Load-bearing, canonical PLASMC (keep)
| clamp | value | role |
|---|---|---|
| `S_MARGIN` | 0.05 | barrier-log finiteness (outer + middle) |
| `izeta_clamp` / `iV_s_e_n` | 5.0 | SMC + outer-PID anti-windup |
| `sat(σ/E)`, `sat(σ_a/E_a)` | ±1 | SMC switching saturation |

### B. Load-bearing, PX4-physics (keep — MATLAB has none of these)
| clamp | value | role |
|---|---|---|
| `W_U_MAX` | 1.0 rad/s | **body-rate cap — LK loses corners >1.7 rad/s** |
| `W_I_MAX` | 5.0 rad/s | caps noisy image `w_i` into `cross(w_i,s)` |
| `DH_D_MAX` | 50 | 1/Z `ds_d` spike guard — **kept at 50 (not 5) to EXPOSE failures** |
| ring/corner lstsq | ±10 rad/s | bounds raw flow-solve output |
| `dw` | ±30 rad/s² | caps `cross(dw,s)`→θ_norm |

### C. Backstops (bound the *symptom*, not the cause)
- `KAPPA_MAX_Z=3.0` — bites at touchdown (z-runaway). **`κ_xy` is uncapped** (deliberate — exposes the lateral
  runaway rather than masking it).
- **cbf2 FoV cone** (`THETA_FLOOR=THETA_CAP=60°`) — kept **relaxed** so `d_min` can't collapse and strangle
  terminal correction. Methodology rule: *if the cone bites in normal ops, the control law is failing* — it's a
  safety net, not a controller.

### D. Proper anti-windup (replaced fixed band-aids)
- Conditional integration in `_yawCtrl` (replaced `_ie_a_clamp`); the `izeta`/`iV_s_e_n` integral clamps.

### E. Disabled / reverted (net-negative — all off-screen-`s`/spurious-`h` attempts, 2026-06-10/11)
| clamp | default | why off |
|---|---|---|
| `PLASMC_VIRT_GUARD` (global `_getVirtualPts` clamp) | **OFF** | clamps genuine in-FoV `s` → under-correction → far drift (29/91 m) |
| `FLOW_NCORN_SWITCH` (n≤4→ring) | **OFF (0)** | ring has ~0 lateral → drift (regressed E_z=1.0 0→4 TL) |
| `FLOW_GATE` (EKF innovation gate) | **OFF (1e9)** | self-defeats on a sustained spike (P-growth opens the gate) + over-rejects genuine flow |

### F. Kept-on, neutral
- `FEAT_FOV_CLIP` (#1 marker-LOST extrapolation clip → ±(p_10+δ)) — conditional, supersedes the old ±5 phantom clamp;
  keeps the FC quat valid through marker-LOST.

---

## Overall verdict

All three control components are **correctly implemented and well-tuned** — lateral converges tight when fed clean
features, yaw is validated for the square-start, descent bootstraps reliably. **Every binding failure traces to the
close-range perception front-end, not the control law.** The clamp set is healthy: the canonical and PX4-physics
clamps are load-bearing and correctly placed; the only "masking" clamps (cbf2 cone, `κ_z` cap) are deliberately
relaxed/scoped so they don't hide the real signal — which is exactly why the diagnosis could reach the perception
root (off-screen `s` via z_v→0 + spurious `h` via ill-conditioned lstsq + ArUco decode-fail). The three disabled
clamps are correctly off (documented dead-ends).

**Structural asymmetry worth noting:** `κ_xy` is uncapped while `κ_z` is capped at 3.0. That's intentional (it
exposes the lateral runaway's cause), but it means the *only* thing between a perception glitch and a fly-away is
the perception quality itself — consistent with the conclusion that the **target / perception front-end is the
remaining lever** (textured / multi-marker target; the code-only corner-enrichment levers — GFT-density,
pyramidal-LK — are exhausted because the low-altitude corner starvation is *detection*-limited).

**Best baseline:** FeatClip `E_z=1.0` → **0 TL / 1.46 m** (n=5, IC1; `#1`-on, gate-off, n-switch-off; baked).
