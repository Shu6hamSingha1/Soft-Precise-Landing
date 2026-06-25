---
name: feedback_combined_surface_divergence
description: "⭐ RESOLVED (2026-06-21): the combined sliding surface WORKS in PX4 — the '2026-06-19 perception-gated, does NOT beat the wall' verdict below is WRONG/SUPERSEDED. Real cause = a GAIN-PARITY BUG: PX4 combined mode ran the BACK-MAPPED soft-config gains (GAMMA 2.0, KAPPA0 0.5, XI2 0.6), 3-5x too HOT for the combined surface -> a_u over-aggression -> fly-aways (mis-diagnosed as a perception/inner-loop-velocity ceiling). FIX = auto-align combined-mode gains to MATLAB VDF-ASMC vdf_params (GAMMA 0.4375/0.5, KAPPA0 0.125, XI2 0.2; commit 22cc732) + chi_r=0.5 (PX4-specific, vs MATLAB 0.85). With manuscript gains the fly-aways VANISH: IC2 n=5 -> 4/5 sub-meter, s_e_n->0, h_e bounded, only 1/5 stochastic terminal-1/Z. NOW BAKED DEFAULT-ON (PLASMC_COMBINED_BARRIER=1). The 2026-06-18 CONVERGENCE insight (combined surface = unified fix for the PX4 wall + MATLAB option-b) was CORRECT; only the 'perception-gated' conclusion was wrong (gains, not perception). Authoritative state: [[project_current_state]] (2026-06-21)."
metadata:
  node_type: feedback
  type: feedback
  originSessionId: 3553f974-8018-4a1e-ba00-93060aa8d9e4
---

**✅ CORRECTED VERDICT (2026-06-21): the combined surface WORKS — the 2026-06-19 "perception-gated" verdict below was a MISDIAGNOSIS.** The combined-mode fly-aways were caused by a **GAIN-PARITY BUG**: PX4 combined ran the BACK-MAPPED soft-config gains (GAMMA 2.0, KAPPA0 0.5, XI2 0.6 — tuned for the old form), which are **3–5× too HOT for the combined surface** → `a_u` over-aggression → fly-aways that *looked* like a brake-starved velocity ceiling. The "ṡ under-reports ~0.5× → under-braked" reading was an artifact of the over-hot gains amplifying the residual, NOT the binding limit. **FIX (commit 22cc732):** auto-align combined-mode gains to MATLAB VDF-ASMC `vdf_params` (GAMMA 0.4375/0.5, KAPPA0 0.125, XI2 0.2) + `chi_r=0.5` (PX4-specific divergence from MATLAB 0.85, for lateral-velocity arrest). With manuscript gains: **IC2 n=5 → 4/5 sub-meter (|xy| 0.72–0.99, td_lat med 0.92), h_e 100% bounded, s_e_n→0 (~96% inside p_r, mild terminal-1/Z breach ~1.05), 1/5 stochastic fly-away.** χ_r↑ was rejected ONLY under the old hot gains; under VDF gains chi_r=0.5 is the baked operating point. **NOW BAKED DEFAULT-ON** (`PLASMC_COMBINED_BARRIER=1` + VDS/DHD/DW-KF + yaw-alpha-filter). OPEN: final IC2-5 gate. Full state + rejected-levers list: [[project_current_state]]. ⬇️ Everything below is the SUPERSEDED 2026-06-19 perception-gated reasoning — kept for history, do NOT act on it.

---

**⚠️ [SUPERSEDED — see corrected verdict above] PX4 VERDICT (2026-06-19, n=5 IC2 A/B): combined surface does NOT beat the lateral wall — perception-gated, NOT control-gated.** Ported to PX4 `controller.py` (env `PLASMC_COMBINED_BARRIER`, default-off, aligned to MATLAB `a152479`). GT-check (decisive) on the IC1 smoke run: the ṡ feeding the blended `h_d` UNDER-REPORTS the dominant lateral velocity — tracks GT ~0.8× while slow but **collapses to ~0.5× during the high-velocity overshoot** (centroid-rate 0.48 ≈ LK 0.52 — both saturate, it's the LK dynamic-range ceiling, NOT filter noise). IC2=(2,2,5) A/B n=5 each: backmap median fin_lat 13.55 (5/5 TL), combined(χ_r=0.85) 13.58 (5/5 TL — SAME wall), chir(χ_r=1.3) 24.81 (5/5 TL, WORSE — more position authority + under-delivered brake → bigger overshoot + κ-runaway 5–9). All reps: s_e_n CONVERGES first (min 0.07–0.59) then DIVERGES to 1.5–2.3 → structurally sound, brake-starved. Confirms GT prediction: same inner-loop velocity front-end is the binding limit; real lever = perception (KLT corner-track/pyr-LK) = architecture. χ_r↑ REJECTED. Code stands as paper↔code alignment (default-off), not a wall-breaker. (Alignment gotcha: logged lateral signals are axis-swapped vs GT V-frame — ignore it and corr reads ~0.) Harness `scripts/run_cb_ic2_ab.sh`; data `test_data/CB_IC2_{backmap,combined,chir}`.

**The manuscript already specifies the structural fix; both code implementations diverge.**
⚠️ THE DESIGN ALREADY EXISTS — see [[project_stacked_barrier_backstepping]] (full design + Lyapunov +
manuscript edits DONE 2026-06-16; draft `Soft_Precise_Landing/Drafts/STACKED_BARRIER_BACKSTEPPING.md`;
**only the CODE is pending**), [[feedback_sliding_surface_relative_degree]] (why naive σ=ζ_h+λζ_s is
rejected but the 3-D combine is well-posed), [[feedback_sen_authority_analysis]] (why the outer PID
can't ensure s_e_n convergence). THIS memory's UNIQUE contribution = the **2026-06-18 CONVERGENCE**:
the MATLAB push d69ed78 (c̃_h option-(b) IC5 block) is the SAME old-form root as the PX4 lateral wall,
so the already-designed combined surface resolves BOTH. Chains off [[feedback_inner_loop_velocity_thread]]
[[feedback_ch_kinematics_correction]].

## The divergence (verified in both codebases)
- **`control_formulation.tex` (manuscript):** dual-funnel COMBINED SLIDING SURFACE. Position barrier
  `ζ_r` (hyperbolic-tan on normalized centroid error) enters the sliding surface DIRECTLY:
  `σ = ζ_h + χ·ζ_aug`, `ζ_aug=[ζ_r, ∫ζ_h3]` → lateral `σ_k = ζ_{h_k}+χ_{r_k}ζ_{r_k}` (PD: flow barrier
  = velocity coord, position barrier = position coord), descent `σ_3=ζ_{h3}+χ_z∫ζ_{h3}` (PI). **`h_d`
  = `w×s + [h_rd−(w×s)·ê3]s` — rotation+descent FEEDFORWARD ONLY, NO `ds_d`.** σ̇ carries the position
  barrier via `G_h⁻¹·χ·ζ̇_aug`; relative-degree-1 preserved. Text l.169: "Rather than back-mapping ζ_r
  into a desired feature rate, the barrier enters the optic-flow sliding surface directly."
- **PX4 `controller.py` + MATLAB `visualControl_IBVS_adaptive.m` (BOTH):** the OLD back-mapped form.
  `SEN_FUNNEL` back-maps `s_e_n → ds_d = G_s⁻¹(−Kp·ζ_s−Ki·izeta−Kd·dzeta)+S_s·dp_s`; `h_d = ds_d +
  cross(w,s) + (h_rd−…)s` (INCLUDES ds_d); `σ = ζ_h + Ω·∫ζ_h` (integral of the FLOW barrier on ALL
  axes — the position barrier `ζ_r` NEVER enters σ). MATLAB l.321/582/586 + PX4 confirmed identical.

## Why this is the unified root cause
In the OLD form the lateral CLOSING AUTHORITY lives in the FEEDFORWARD (the `w`-cross-products in
`c̃_h` / the back-mapped `ds_d`). In the manuscript's combined surface it lives in `ζ_r` DIRECTLY in σ.
So both observed failures are the SAME missing direct-position-authority:
- **MATLAB c̃_h option-(b) BLOCKED at IC5 (d69ed78):** the corrected (clean) `c̃_h` "lacks the closing
  authority the funnel guarantee needs — `s_e_n` breaches `p_s` 46% of steps, →6× p_s, no convergence"
  → because removing the feedforward cross-products removes the closing authority (old form).
- **PX4 lateral wall:** `s_e_n` converges-then-overshoots, `G_s⁻¹` collapse starves the integral
  recovery, `a_u` outward — same missing direct authority, same old form.
MATLAB chat recommended option (c) (present corrected model in paper, keep old-code results,
residual→`d_h`) — a PAPER WORKAROUND that LEAVES the code↔paper divergence. The real fix is the
combined surface (neither chat has implemented it in code).

## THE UNIFIED FIX = implement the combined surface
One change resolves all four: unblocks option (b) (closing authority → `ζ_r`, not the removed FF),
fixes the IC5 deficit, fixes the PX4 lateral wall, and aligns code↔paper (no `d_h` workaround). ⚠️
**MATLAB code is OFF-LIMITS (Windows-owned, user directive 2026-06-18) — do NOT edit it; PX4-side only.**

## ✅ PX4 PORT DONE (2026-06-19, controller.py, env-gated `PLASMC_COMBINED_BARRIER` default-off)
Implemented + ALIGNED to the canonical MATLAB realization (a152479 "blended sliding surface"), NOT the
a-priori spec below. py_compile clean; default-off byte-identical (chi_zeta_aug=Omega@zeta); combined-path
math finite+sign-correct. **SITL A/B (IC2 vs default + IC1 regression + IC2-5 gate) PENDING — user runs.**
CORRECTIONS to the spec below (MATLAB realization diverged from my guess):
1. **ζ_r is NOT the SEN ζ_s.** Build a SEPARATE barrier on `r_bar_e = s_e_n` (PX4 p_10=center/focal IS
   φ_max, so s_e_n already = FoV-normalized r̄_e) with its OWN funnel `p_r` (1.2→1.0 FoV-consistent,
   ξ_r=0.10) — NOT the SEN p_s (floor 0.35). Built in `_updatePerfFunc`+`_updateImgFeatureParam`.
2. **h_d KEEPS measured ṡ** (blended): `h_d=[s_dot_meas;0]+rot+descent`, s_dot_meas=smooth4 of d(s_e[:2])/dt.
   NOT strict FF-only. The back-mapped `ds_d` is what's dropped.
3. **s̈ dropped from dh_d:** differentiate `h_d_noS`(=rot+descent) so the 1/z-inflated s̈ never enters
   c_h (κ absorbs it). New member `self._h_d_noS`.
4. transport/descent follow `CH_CLEAN` (psi_dot_b vs w_i×s) to stay consistent with the c-term.
5. Combined tightens the lateral FLOW funnel floor `p_2inf_xy=0.5` (if not overridden).
Knobs: `PLASMC_COMBINED_BARRIER`(0) `PLASMC_CHI_R_X/Y`(0.85) `PLASMC_PR0_X/Y`(1.2) `PLASMC_PRINF_X/Y`(1.0)
`PLASMC_XIR_X/Y`(0.10). σ/chi_zeta_aug/Theta/a_v per the generalization (chi_zeta_aug=[χ_r·ζ̇_r; Ω_z·ζ_h3]
combined, =Ω·ζ_h back-mapped). Backups: src/controller.py.bak + .HEAD.bak.

## ⚠️ FIRST SITL RESULT (2026-06-19, IC1, combined ON, headless smoke, n=1) — lateral wall RECURS
Code SITL-STABLE (full flight, no crash/NaN) but **FLEW AWAY: xy=6.87 m, vel=4.84 m/s at IC1** (centered —
the back-map lands this). Combined confirmed engaged (p_inf=[0.5,0.5,0.5]). Authority/breach diagnostic
(Control_Data, `test_data/CombinedBarrier_smoke/Fri Jun 19 00-11-24 2026/`):
- **s_e_n CONVERGES (min 0.01 @ t=1.3s) then DIVERGES** (final 2.06, max 4.19); breaches p_s@5.9s (38%) and the
  active p_r@8.1s (20%). **Converge-then-overshoot — same lateral wall.** Authority CONVERGES it but can't KEEP it bounded.
- **h_e bounded in p_h the WHOLE flight (0% breach)** — but TRIVIALLY: the blended h_d (=measured ṡ+FF) makes
  h_e≈0 BY CONSTRUCTION → flow-funnel satisfaction is DECOUPLED from position. The h_e-funnel tells you nothing.
- σ blows up (→8.6), κ_y ratchets (→4.54), |a_u_xy|→1076 m/s² but `corr(a_u,−s_e_n)≈−0.5` (anti-restoring).
- **ROOT HYPOTHESIS:** combined h_d depends on the MEASURED centroid rate ṡ; SITL's LK flow UNDER-REPORTS velocity
  ([[feedback_inner_loop_velocity_thread]] / [[feedback_lateral_overshoot_root]]) → h_d under-commands the brake →
  overshoot. **The SAME inner-loop velocity-observability wall, now INSIDE the combined surface.** MATLAB (accurate
  ṡ) = 25/25; SITL = perception-gated. The combined surface gives the structural authority but can't beat the front-end.
- NEXT: (1) GT-verify the ṡ under-report (measured flow vs `tools/gt_optical_flow.py`) = DECISIVE. (2) The PX4 port
  uses `smooth4(finite-diff)` for ṡ — NOT the MATLAB Savitzky-Golay `CB_SDOT_FILT`; add the SG variant if ṡ-noise compounds.
  (3) χ_r↑ + IC2 A/B (n≥5; n=1 is noise but the mechanism is clear). If confirmed → real lever = perception (KLT/pyr-LK) = architecture.

## PX4 IMPLEMENTATION SPEC — CODE-LEVEL MAP (a-priori; SUPERSEDED above re ζ_r source + h_d)
Worked out by reading the exact blocks (the reuse-ζ_s assumption was WRONG — see corrections above).
**KEY INSIGHT: it is a PURELY LATERAL change.** Descent (z) stays byte-identical because (a) `ds_d`'s
z-component is ALREADY 0 (`raw_ds_d = concat([V_ds_d_xy,[0.0]])` l.679), and (b) descent keeps its PI
form `σ_3=ζ_h3+Ω_z·∫ζ_h3`. So dropping `ds_d` and swapping the σ-augmentation only touch x,y.

DATA ALREADY AVAILABLE (SEN_FUNNEL computes them in `_updateImgFeatureParam`, which runs BEFORE PLASMC):
`ζ_r = self._zeta_s[-1]` (2-vec, l.617) · `ζ̇_r = smooth4(self._dzeta_s_deque)` (2-vec, deque l.426, fed
l.628). Requires `self._sen_funnel` ON (default) — guard the combined path on `self._sen_funnel and len(self._zeta_s)>0`, else old path.

THE CLEAN GENERALIZATION (old form = special case): define two 3-vectors, lateral from ζ_r, descent from Ω_z·ζ_h:
- `zeta_aug = [χ_r·ζ_r_x, χ_r·ζ_r_y, Ω_z·∫ζ_h3]` → `σ = ζ_h + zeta_aug` (replaces l.826 `σ=ζ+Ω·∫ζ`).
- `chi_dzeta_aug = [χ_r·ζ̇_r_x, χ_r·ζ̇_r_y, Ω_z·ζ_h3]` → used in BOTH:
  - Theta `vector` (l.857-859): `-c + S·dp - G⁻¹·chi_dzeta_aug`  (was `- G⁻¹·(Ω·ζ)`).
  - `a_v` (l.885-888): `… - chi_dzeta_aug`  (was `- Ω·ζ`).
  (OLD form is exactly `chi_dzeta_aug=Ω·ζ_h`, `zeta_aug=Ω·∫ζ_h` on all 3 axes — so the descent rows are unchanged.)

`h_d` (`_updateOptFlow` l.724-736) — DROP `ds_d` when COMBINED; composes cleanly with CH_CLEAN:
`ds_d_term = np.zeros(3) if self._COMBINED_SURFACE else self._ds_d[-1]`; then keep the existing
`if self._CH_CLEAN: h_d = ds_d_term + h_ref_eff*s  else: h_d = ds_d_term + cross_ws + (h_ref_eff-dot(cross_ws,e3))*s`.
(ds_d present iff NOT COMBINED; cross_ws present iff NOT CH_CLEAN — orthogonal knobs.)

KNOBS (`__init__` near l.284 + registry near l.218): `self._COMBINED_SURFACE = os.environ.get("PLASMC_COMBINED_SURFACE","0")=="1"`;
`self._chi_r = float(os.environ.get("PLASMC_CHI_R","1.0"))`. **χ_r SIGN: positive is correct** (pos err +x → ζ_r>0
→ σ drives ζ_h<0 → −x closing flow → centers). Keep it SIGNED so SITL can flip if it regresses, like CH_CLEAN's `_r`.
NOTE: the design ([[project_stacked_barrier_backstepping]]) permits χ_r as a 2×2 LATERAL BLOCK (χ=blkdiag(χ_r∈ℝ²ˣ², χ_z));
a scalar χ_r·I₂ (above) is the simplest diagonal instance — start scalar, generalize to per-axis/2×2 only if needed.

WHY REL-DEG-1 IS PRESERVED (matches manuscript): `ζ̇_r ∝ ṡ ∝` flow `h` (a STATE, rel-deg-2 in u) — it's a
MEASURABLE feedforward, exactly cancelled by the `u_eq` term `−G⁻¹·χ·ζ̇_aug`. So `σ̇ = β·G·u + known` — SMC sees u via ζ̇_h only.

VALIDATE: A/B at IC2 + IC1 regression + IC2-5 gate (PX4 SITL stochastic; the deterministic MATLAB-first
validation the convergence calls for is the Windows chat's to run, NOT mine). For MATLAB I may provide a
TEXT spec only, never edit their files. Expected win: `s_e_n` gets DIRECT SMC authority (not routed through
the corrupted/saturating velocity loop) → closes the lateral wall AND removes the c̃_h closing-authority deficit.
