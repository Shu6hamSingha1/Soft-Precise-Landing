---
name: Baseline controllers fail on their own published gains
description: Lin/Zhang/Chen/Cho all catastrophically fail the comparison harness when run with the exact gains published in their papers; manuscript uses best-effort retuning only
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
On 2026-04-16, ran the 5-trajectory comparison harness with each baseline's paper-authoritative gains (Lin 2022 p7: k1=4.5, k2=2.0, ρ∞=0.05, l=0.5; Zhang 2026 Table III: Kc1=diag(0.2,0.2,0.6), Kc2=diag(1.8,1.8,2.6), Kc3=diag(0.8,0.8,2.4); Chen 2025 Sim 1: kr=8, k1=0.2, k2=20, k3=2, k4=0.4, ẑ*(0)=10m; Cho 2022 Table 2: λ=diag(2,2,5,0,0,0.2) sign-flipped to NED). Results:

- **Lin 2022**: PPC barrier saturates in 1–2 steps (p_xy stuck at initial 2.83m). k1=4.5 with IC offset 2.83m and ρ_inf_v=0.05 produces `vhat` far outside `ρ_v` on step 1 — `xi_v` clamps to ±0.999 instantly, no recovery. Paper tested near-zero offsets.
- **Zhang 2026**: crashes in ~2s at v_z=1.0–1.4 m/s, p_xy up to 1.1m. Paper's vertical ω_n~1.6 rad/s finishes descent before horizontal converges. Paper tested slow moving platforms.
- **Chen 2025**: diverges — v_xy up to 15 m/s, maxT saturated at 60N, p_xy=9.5m on Static. Pixel noise through finite-diff `de` amplifies to superhuman thrust (already documented in project_chen2025_limitation.md). Paper had no noise.
- **Cho 2022**: λ=-2/-2/-5 works on Static (p_xy=0.19m) but fails Linear/Lissajous moving targets.

**Why:** Paper gains are tuned for the paper's test scenario (small offsets, no noise, slow platforms). Our harness is adversarial: IC=[2,2,-5], depth-dependent pixel noise, mean wind 0.2 m/s + OU turbulence, parametric uncertainty, ship-deck trajectories (Lin heave + roll/pitch), lateral target speeds ~4× Lin's hardest test.

**How to apply:** In the manuscript, do NOT present paper-gains results — that would look like rigging the comparison ("author picks bad gains for baselines"). Instead present ONLY **best-effort retuned** gains, staying within each algorithm's structure (no algorithmic modifications). Frame the story as: "Each baseline retuned within its own published framework. Structural limits of each approach prevent closure of the robustness gap." This is both fairer and stronger — it isolates the robustness gap from the tuning axis.

**Best-effort intermediate tunes** (from 2026-04-16 earlier iteration): Lin k1=0.6/k2=4.0 with per-axis ρ_inf_v=[0.3,0.3,0.05]; Zhang xy sped up Kc1=diag(0.25,0.25,0.1), Kc2=diag(2.0,2.0,0.7), Kc3=diag(2.5,2.5,1.2); Cho λ=[-1.2,-1.2,-2.0], Kv=diag(3.5,3.5,2.0), k_sigmoid=0.02, v_sat(3)=0.7. Chen has no safe retune (structural limit — cite project_chen2025_limitation.md).
