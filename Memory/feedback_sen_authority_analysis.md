---
name: feedback_sen_authority_analysis
description: "SEN-funnel outer-loop per-term control authority quantified; full-gain back-map reshape (SEN_RECOVER_ST) is INERT → 3rd confirmation the breach wall is inner-loop, not outer authority"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ce5926b8-f1ea-4718-84c8-9a29e27ac4c6
---

Quantified the SEN-funnel outer-loop authority over `s_e_n` (analysis +
capture_sen_authority.m/analyze_sen_authority2.py, IC5 seed6 breach). Chain:
`s_e_n → S_s=s_e_n/p_s → ζ_s=log((1+S)/(1−S)) → PID dζ_sd → V_ds_d=G_s⁻¹·dζ_sd + S_s·ṗ_s`.
Two exact identities: **`G_s⁻¹=(p_s/2)(1−S²)`** and **`ζ̇_s=G_s(ṡ_e_n−S·ṗ_s)`**.

**Per-term authority over s_e_n:**
- **P** `−rp·ζ_s`: back-mapped demand `−(rp·p_s/2)·g(S)`, `g(S)=(1−S²)log((1+S)/(1−S))`.
  Marginal restoring gain `d(demand)/ds_e_n = −(rp/2)·g'(S)`, `g'(S)=2−2S·atanh-barrier`.
  **Peaks at |S|=0.648, ZERO there, and FLIPS POSITIVE (anti-restoring) for |S|>0.648** — the
  inward demand stops increasing with error and weakens. `G_s⁻¹` collapses ~13× (0.39→0.03)
  mid-funnel→boundary. Independent of gain magnitude (raising rp doesn't move the 0.648 flip).
- **I** `−ri·izeta·G_s⁻¹`: negligible (ri=0.1, izeta clamped, ×(1−S²)); demand ~0.01–0.03 vs
  P's tens; collapses at boundary. Effectively inert.
- **D** `−rd·ζ̇_s·G_s⁻¹ = −rd·(ṡ_e_n − S·ṗ_s)`: G_s CANCELS → constant-gain `rd` velocity
  damping, NO collapse — but acts on ṡ_e_n which →0 at the breach turnaround, so |D|→0 at the peak.
- **FF** `S_s·ṗ_s`: the ONLY structurally non-collapsing inward term (max at boundary), but small
  (~0.13) and `ṗ_s` decays with t.
Net: convergence of s_e_n is only guaranteed for **|S_s| ≲ 0.648**.

**FIX TESTED — SEN_RECOVER_ST (default-OFF hook):** on breach pin S_s at a small target ST via
`p_s_eff=|s_e_n|/ST` → P-demand `−rp·[g(ST)/(2ST)]·|s_e_n|`; `g(S)/S→2` as S→0 so small ST gives
~FULL −rp linear restoring gain (vs legacy 0.95-pin = 0.19·rp). Mathematically eliminates the
anti-restoring collapse. **A/B (IC5 12-seed, GAMMA_COG=0.005): INERT** — ST∈{off,0.5,0.3} gives
SP 10/12 unchanged, seed6 breach 1.46→1.40 (still TL), no regression.

**CONCLUSION — 3rd independent confirmation the breach wall is NOT outer authority** (after
SEN_RECOVERY_K + escalating-demand A/B, [[feedback_lateral_overshoot_root]]). Two reasons it
can't help: (1) inner-loop/perception can't DELIVER the braking flow (flow under-reports vel);
(2) seed6 is a SEN↔CBF DIRECTIONAL CONFLICT — SEN demands inward −Y, the visibility CBF STRIPS
−Y to keep the marker in frame ([[feedback_ic5_cbf_strip_mechanism]]), so no SEN gain can win;
seed4 doesn't even SEN-breach (br=0.69, geometric FoV loss). Keep SEN_RECOVER_ST default-off
(correct, harmless diagnostic — do NOT bake). Real lever stays inner-loop velocity observability.
