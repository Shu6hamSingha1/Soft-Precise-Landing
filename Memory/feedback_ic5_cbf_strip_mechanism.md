---
name: feedback_ic5_cbf_strip_mechanism
description: "IC5 [2,2,-3] noisy seed-6 runaway root cause (MATLAB) = CBF -Y strip + d=0 myopia; DH_D_CAP=20 baked; observability-boundary IC"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ce5926b8-f1ea-4718-84c8-9a29e27ac4c6
---

⭐ MATLAB IC5 [2,2,-3] noisy seed-6 runaway — full root cause (2026-06-15, triangulated across ~15 matched-seed sweeps + 10 probes). Supersedes the older "flow under-reports velocity" framing.

**Mechanism (proven):** IC5 starts a marker corner at the FoV-box edge (33px margin — *half* of IC2's, because image error ∝ offset/Z and IC5 is IC2's offset from 40% lower altitude). To translate −Y the drone must lean −Y, but that lean *ejects* the edge-corner, so the corner-based CBF (cbf2_filter, alternating projection) **strips a mandatory ~fixed −Y**. Under noise the −Y command is weakened; when strip > |desired −Y| the QP **flips it +Y** (~t=0.8s) → unbraked lateral runaway → `s_e_n` breaches `p_s` (~t=1.6s) → `fov_fail`. The c-term's `V_dh_d` (numerical derivative of desired flow) spikes ~O(100) → ±5 m/s² command thrash that does the noise-weakening.

**Breach is IRREVERSIBLE:** the PPC barrier guarantees convergence only by *never* breaching (its unbounded demand exists only up to the boundary); once `S_s` clips at ±0.95 the demand SATURATES → no after-breach recovery. Confirmed: RECOVERED=0 across base/containment/(a)+(b). So the aim must be breach *prevention*, not recovery.

**NOT outer-loop or CBF tunable — observability boundary.** Every lever that delivers seed-6's −Y also breaks the 11 good seeds, because at IC5 they're *geometrically identical* (same corner-at-edge, same strip) and differ ONLY by noise (does the residual −Y survive). No in-the-loop signal separates them before the flip commits. Tried & failed (all hit this): funnel sizing/containment, K_rp/ri/rd, middle-loop Γ/E/P/κ0, kR/kΩ, theta_cap (it's a post-QP deliverable cap, NOT the CBF — varying it is non-monotonic), inset (inset=0 prevents seed-6 breach 12/12 but ejects good-seed startup corners → SP 9→2), directional inset / sign-preserving guard (no-op: flip is symptom not cause), DRIFT_RECENTER (seed CBF drift with −g·cr2 to fix `d=0`-at-start myopia — the *only* CBF-side breach-prevention, principled, but not self-targetable; g≥0.5 → 12/12 no-breach but SP→2), strip-ratio-gated drift (strip_frac high for ALL IC5 seeds → can't separate).

**BAKED this session:** `DH_D_CAP=20` (visualControl default 0→20) — PX4 DH_D_MAX analogue, hard cap (NOT LPF: lag regressed SP 9→5). Kills the `V_dh_d` thrash. Validated no-regression on canonical 5 ICs (noiseless+noisy n=5: SP=27/land=29 byte-identical to base; cap only engages on pathological spikes). Commit 8af734a. The CBF hooks (DRIFT_RECENTER/DRIFT_GATED/DIR_INSET_RELAX/CBF_NO_REVERSE) are committed default-off.

**Tooling:** matched-seed harness `MATLAB/Multi_init_cond/sen_ic5_seedtest.m` (12 fixed RNG seeds of IC5, breach ratio r=max|s_e_n|/p_s, NO-BREACH/RECOVERED metrics); `probe_*.m` traces; `validate_cap20.m` (5-IC bake gate). GOTCHA: cbf2-only globals silently become *locals* after the controller's top-of-file `clear` in a sweep loop → re-declare every iteration (3 CBF hooks were no-ops until fixed). Also: the controller defines `m`=mass, so never name a metrics struct `m`. Links: [[feedback_lateral_overshoot_root]] [[feedback_optic_flow_underreports_root]] [[project_cbf_visibility_design]]
