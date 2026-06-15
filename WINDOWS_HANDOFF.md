# Windows Handoff — manuscript edits + result tests (2026-06-15)

Resumes work for the **manuscript** (written/tested on Windows), following the Ubuntu MATLAB
robustness session. Counterpart to `UBUNTU_HANDOFF.md`.

> **PRIORITY: apply the `c̃_h` kinematics correction to the supplement + control_formulation
> (TASK 1), then decide whether the adaptive CoG feedforward becomes a paper contribution
> (TASK 2).** All supporting data — exact equations, `.tex` line numbers, result tables — is in
> **`Soft_Precise_Landing/MANUSCRIPT_DATA_HANDOFF.md`**. Read that doc first; this file is the
> action list.

---

## 0. Sync first

```bash
cd ~/Soft-Precise-Landing      # (Windows path as appropriate)
git pull origin main           # export PATH=$PATH:"/c/Program Files/GitHub CLI" only if gh missing
```

Brings the bake commits (`2ec4477` CoG-FF, `f41e9ac` data handoff, `81f96d3` parity note) and
the data doc. MATLAB controller changes are reference for the paper; they don't affect the
Windows manuscript build.

---

## 1. Context — what the Ubuntu MATLAB session established (all on the noisy robustness model)

1. **`c̃_h` was a mis-transplanted kinematics expression; corrected.** `manuscript.tex` already
   has the right form (`ḣ = −βᵛa_d − ψ̇_b(ê₃×h) − (h·ê₃)h + d_h`); the supplement + control_formulation
   still carry the old full-`w` form. Falsified analytically (transport theorem) + numerically
   (old residual 2.5–4× signal vs `.mat` GT; new at the 2% floor).
2. **IC5 noisy failures = CoG-offset (parametric) uncertainty**, not the c-term, funnel, or gains.
   Mechanism: `τ_d = T·[−δy; δx; 0]` (thrust-proportional body torque, `|r_cog|≤5 mm`).
3. **A thrust-scaled adaptive CoG feedforward fixes it** (`GAMMA_COG=0.005`, baked): IC1-5 gate
   passes with no regression and improves IC3/IC4/IC5 (base SP 27/30 → 29/30).

---

## 2. TASK 1 — Apply the `c̃_h` / `d_h` correction (REQUIRED)  ⭐ start here

Make the supplement + control_formulation consistent with `manuscript.tex`. Exact targets in the
data doc, §"FINDING 1":
- `supplemental.tex` L96–97 → `c̃_h = −ψ̇_b(ê₃×h) − (h·ê₃)h`; L101 → drop "cross products of
  w, ẇ, s, h" **and the `ω_b×v_b/z` Coriolis term in `d_h`** (own-state-free strengthening).
- `control_formulation.tex` L66 (`ṡ`), L97 (`c̃_h` text), L171 (`h_d`) → ψ̇_b form.

**⚠ Decide first (data doc has the full argument):** the 25/25 results came from the OLD
full-`w` code, and the corrected form regresses closed-loop, so don't silently swap the model.
**Recommended = option (c):** present the corrected model, note the residual is absorbed into the
bounded disturbance `d_h` (→0 as `s_xy→0`).

---

## 3. TASK 2 — Decide on the adaptive CoG feedforward as a contribution

Data doc §"FINDING 2" has the algorithm box, parameters, and both result tables (IC5 12-seed +
IC1-5 gate). If included: add the algorithm + the IC1-5 table + the `τ_d` mechanism + a one-line
Lyapunov note. If not: at minimum cite the CoG offset as the binding parametric uncertainty
(§"FINDING 3"). **PX4 port is NOT applicable** (rate-mode PX4; see `CONTROLLER_PARITY.md`,
commit `81f96d3`) — it's a MATLAB-only robustness result.

---

## 4. TASK 3 — Regenerate affected figures/tables

The numbers in the data doc are the reference values. Matched RNG seeds; expect minor stream
differences if the Windows RNG differs — direction-of-effect is what matters. Keep
`PX4_Gazebo/docs/CONTROLLER_PARITY.md` current with the code↔paper c̃_h divergence.

---

## 5. Code state (reference)

| item | state |
|---|---|
| `DH_D_CAP=20`, `GAMMA_COG=0.005` | BAKED (commits `8af734a`, `2ec4477`) |
| `C_SIMPLE` (manuscript c-term) | default-OFF; REJECTED (regresses closed loop) |
| `COG_LEAK` (σ-mod leakage) | default-OFF; dead-end |

Harnesses in `MATLAB/Multi_init_cond/`: `cogff_ic5.m`, `cogff_refine_ic5.m`, `validate_cogff.m`
(IC1-5 gate), `capture_cogff_diag.m`, `csimple_ic5.m`.
