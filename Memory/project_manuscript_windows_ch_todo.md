---
name: project_manuscript_windows_ch_todo
description: "Manuscript (Windows) data + TODO — c_h correction, adaptive CoG-FF contribution, IC5 robustness; full transfer doc committed to repo"
metadata: 
  node_type: memory
  type: project
  originSessionId: ce5926b8-f1ea-4718-84c8-9a29e27ac4c6
---

Manuscript is written/compiled on **Windows**; manuscript TESTS run there too. The full,
precise transfer (equations, .tex line refs, result tables, code state) is committed to the
repo: **`Soft_Precise_Landing/MANUSCRIPT_DATA_HANDOFF.md`** (commit `f41e9ac`) — Windows reads
it after the push lands. This memory is the local-machine index of that doc. From the
2026-06-15 Ubuntu MATLAB runs. See [[feedback_ch_kinematics_correction]],
[[feedback_cog_adaptive_feedforward]], [[feedback_ic5_cbf_strip_mechanism]].

**THREE manuscript-relevant findings:**

1. **`c̃_h` kinematics correction (REQUIRED .tex fix).** `manuscript.tex` (L195–212) is CORRECT
   (`ṡ=h−ψ̇_b(ê₃×s)−(h·ê₃)s`, `ḣ=−βᵛa_d−ψ̇_b(ê₃×h)−(h·ê₃)h+d_h`, `d_h` has NO `ω_b×v_b/z`).
   `supplemental.tex` S1-B (L96–97, L101) + `control_formulation.tex` (L66/L97/L171) still carry
   the OLD full-`w` form → fix them to match. Justified analytically (transport theorem) +
   numerically (old residual 2.5–4× signal vs .mat GT; new at 2% floor). ⚠ DECISION POINT:
   25/25 results came from OLD full-`w` code; corrected form (C_SIMPLE) REGRESSES closed loop
   (IC5 SP 9→2) — don't silently swap. Recommended option (c): present corrected model, note
   the residual is absorbed into bounded `d_h` (→0 as s_xy→0).

2. **Thrust-scaled adaptive CoG feedforward (NEW contribution candidate).** Baked
   `GAMMA_COG=0.005` (`2ec4477`). `τ_d=T·[−δy;δx;0]`; law `τ_ff=−T·θ̂`, `θ̂̇=Γ·T·(e_Ω+c₂e_R)_xy`.
   IC1-5 gate: NO-regression, IC1 perfect, IMPROVES IC3/4/5 (base SP 27/30→29/30). If added:
   algorithm + IC1-5 table + Lyapunov note. [[feedback_cog_adaptive_feedforward]].

3. **IC5 = CoG-offset binding parametric uncertainty.** Ablation: noPARAM 12/12, noCOG 11/12;
   NOT c-term/funnel/gain. Two walls: seed-4 startup-tilt → descent-softness; seed-6 CBF-strip
   observability. Strengthens the robustness-section framing.

**Windows TODO:** (a) decide Finding-1 option a/b/c, apply `c̃_h`/`d_h` edits to supplemental +
control_formulation; (b) decide if CoG-FF is a paper contribution; (c) regenerate affected
figures/tables (numbers in the handoff doc are the reference); (d) update
`PX4_Gazebo/docs/CONTROLLER_PARITY.md` with the code↔paper c̃_h divergence.

**Transfer mechanism:** handoff doc + bake commits (`2ec4477`, `f41e9ac`) are **PUSHED to
origin/main** (the PX4 chat on the same working tree pushed them + `81f96d3` parity doc). Windows
gets them on `git pull`. NOTE: the CoG-FF PX4 port is CLOSED as not-applicable (rate-mode PX4;
[[feedback_cog_adaptive_feedforward]]) — CoG-FF stays a MATLAB-only robustness contribution.
