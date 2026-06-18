---
name: feedback_cog_adaptive_feedforward
description: "Thrust-scaled adaptive CoG feedforward — WORKING IC5 fix candidate (prototype, default-off, not yet baked/IC-gated)"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ce5926b8-f1ea-4718-84c8-9a29e27ac4c6
---

The IC5 noisy failures are CoG-offset (parametric) driven ([[feedback_ic5_cbf_strip_mechanism]],
[[feedback_ch_kinematics_correction]] confirmed it's NOT the c-term). Exact mechanism from
`UAVDyn_robust.m`: `τ_d = r_cog × f_b`, `f_b=[0;0;−T]` ⇒ **`τ_d = T·[−δy; δx; 0]`** — a purely
THRUST-PROPORTIONAL body torque, xy-only, zero yaw, |r_cog|≤5 mm.

**Why a plain integral term (kI_R) failed:** it integrates `e_R` regardless of thrust → ignores
the `T` regressor, lags, destabilizes the fast loop. **The fix is a thrust-scaled adaptive
feedforward** (Lee-style geometric adaptive): estimate the per-thrust coefficient
`θ̂ ≈ [−δy; δx]`, cancel with `τ_ff = −T·θ̂`, adapt on the thrust-scaled composite attitude
error `θ̂̇ = Γ·T·(e_Ω + c₂·e_R)_xy` (the `c₂·e_R` term makes the constant torque observable at
steady state where `e_Ω→0`). θ̂ clamped at ±0.02 (anti-windup; 4× the 5 mm bound).

**Implementation (default-OFF):** `visualControl_IBVS_adaptive.m`, globals `GAMMA_COG`
(0=off), `COG_C2` (default 2.0), `COG_MAX` (0.02). State `thetahat_cog` (2×1, xy). Added at
the inner-loop torque (`B_tau_cd += tau_cog`). Backup `Obsolete/..._pre_cogff_20260615.m`.
Harness `cogff_ic5.m` (12 matched seeds, γ sweep).

**RESULT 2026-06-15 (IC5 [2,2,−3] noisy, n=12):** **`GAMMA_COG=0.005` is the best IC5 result
to date.** SP 9→**10/12**; vel mean 3.14→**0.42**, max 35.2→2.0; xy mean 6.77→**0.16**. **Seed 4
(startup-tilt) FIXED** (didn't expect adaptation to catch a t=0.4s failure — it does). **Seed 6
(CBF-strip) contained but not cured**: 78 m fly-away → 1.7 m near-miss TL. Clear sweet spot —
`γ≥0.02` OVER-adapts (SP→0: lands soft ~0.8 m/s but a small steady lateral bias pushes xy above
the 0.08 m box; γ=0.02 also adds seed 8). Low gain is right.

**STATUS: prototype, NOT baked.** Pending before any default change: (1) refine γ ~0.003–0.008,
try to fully recover seed 6; (2) **mandatory IC1-5 no-regression gate** ([[feedback_ic_validation]],
run_ic_validation.sh equivalent) — γ must not regress IC1/centered; (3) PX4 port (same τ_d there);
(4) if it holds, candidate manuscript robustness contribution ([[project_manuscript_windows_ch_todo]]).

**PX4 PORT — NOT APPLICABLE (decided 2026-06-15, item 3 CLOSED).** The CoG-FF is a MOMENT/torque
feedforward (`B_tau_cd += τ_cog`); PX4's controller.py is rate-mode (`w_u=−K_R·e_R`, ships body-rates,
PX4 owns the rate→torque loop) and a torque/actuator refactor is standing-rejected ([[feedback_thrust_torque]]),
so there is no moment command to inject `τ_cog` into. It is also REDUNDANT + INERT in PX4: (a) PX4's
rate loop has integral action (`MC_ROLLRATE_I`/`MC_PITCHRATE_I`) that already rejects a steady
thrust-proportional CoG torque — the MATLAB FF only exists because MATLAB's SO(3) law is
proportional-only with NO integral rate loop; (b) because PX4's rate-I absorbs the torque, in steady
state `e_Ω→0` and `e_R→0` as seen by controller.py, so the adaptation `θ̂̇=Γ·T·(e_Ω+c₂·e_R)` has NO
signal → `θ̂` stays ~0 → inert; (c) PX4 SITL doesn't inject the MATLAB robustness model's ±5 mm random
`r_cog` anyway. So the CoG-FF stays a MATLAB-only robustness contribution; do not re-attempt the PX4 port.
Sign was derived Lyapunov-correct and confirmed by the result (no divergence).

**2026-06-15 — the 2 non-SP at γ=0.005 DIAGNOSED + 2 candidate fixes FALSIFIED → γ=0.005 is
the converged optimum.** Trace capture (`capture_cogff_diag.m`, `analyze_cogff_diag.py`):
- **Seed 4 (hard land vz=1.43, NOT a TL):** CoG-FF fixes the startup TL (tilt 12.6°→9.6°, stays
  in FoV), BUT θ̂ converges only at t≈3.36s while it lands at t=4.39 — under-compensated 0–3.3s,
  tilt swings to 25.8°, tilted thrust starves lift → fast descent → terminal vz=1.30. Residual =
  the TERMINAL DESCENT-SOFTNESS wall ([[feedback_descent_softness]],
  [[feedback_terminal_descent_loom_overreport]]), invariant: N_z=0.1 left vz=1.36 (and regressed
  nominal SP 10→6). Not CoG/z-gain resolvable.
- **Seed 6 (contained TL):** CoG-FF slashes the runaway (sen/ps 11.9→1.46, lat 78m→2.8m, tilt
  55°→16°) but the Y-axis breach still tips over at t=2.12s (peak sen/ps just 1.46, barely >1),
  θ̂ converging at t≈2.13 races the breach and loses. = the CBF-strip observability boundary
  ([[feedback_ic5_cbf_strip_mechanism]]), "no lever self-targets". Not CoG/gain resolvable.
- **FALSIFIED fix A — faster convergence via σ-mod leakage** (`COG_LEAK` global, default-off):
  `θ̂̇=Γ T e_comp − σ θ̂`. Higher γ+leakage REGRESSES hard (γ=0.015,σ=1 → SP 10→2, seed 4 back to
  TL). Aggressive adaptation chases the noisy composite error → transient torque → worse precision.
  Leakage hook kept default-off (dead-end). **FALSIFIED fix B — N_z=0.1** (above).
CONCLUSION: γ=0.005/σ=0 is the robust optimum; the 2 residual non-SP are pre-existing
architectural walls (descent-softness terminal under-brake; CBF observability), NOT new issues.

**2026-06-15 — γ REFINE + IC1-5 GATE PASSED (validate_cogff.m, base vs γ=0.003 vs γ=0.005,
5 canonical ICs {[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}, noiseless n=1 + noisy n=5):**
γ=0.003 and γ=0.005 BOTH 10/12 on IC5-12seed (γ=0.003 softer vel 0.315, γ=0.005 more precise
xy 0.164; γ=0.007 regresses seed4). **GATE: NO-REGRESSION on all 5 ICs for both; IC1/centered
stays perfect (ny SP5/L5); IMPROVES IC3 (SP4→5), IC4 (SP4→5), IC5 (land4→5). TOTAL base SP27/L29
→ both candidates SP29/L30.** Both prerequisites for default-on now MET (refine + IC gate);
seed-6 full recovery is NOT γ-achievable (CBF boundary, documented). **BAKED + pushed:
GAMMA_COG default 0→0.005 (commit 2ec4477).** PX4 port CLOSED as NOT APPLICABLE (see above;
parity doc 81f96d3). CoG-FF is a MATLAB-only robustness contribution — DONE.
