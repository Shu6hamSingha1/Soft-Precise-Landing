---
name: fix-causes-not-limits
description: "User directive (2026-06-03): when a state/parameter saturates, find the REASON and fix it via the manuscript control parameters — never by adding clamps or reducing limits. Clamps (DH_D_MAX, DSD_CLAMP, THETA_FLOOR) are not control parameters."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**User directive during the SP campaign:** "If some parameter or system state is getting saturated or reaching its limit, find out the reasoning for this saturation instead of reducing its limits. What you have tuned till now are not control parameters. These are [the manuscript VDF-ASMC set]. We need to tune these control parameters to make our controller achieve soft landing."

**Why:** the clamps I had added (DH_D_MAX=5, THETA_FLOOR=60, DSD_CLAMP) treat symptoms. A properly tuned controller doesn't saturate its internal states at all — the SP rep proved this (nothing saturated, no clamp was ever active). Clamps mask the mis-tuning and create new pathologies (e.g., DH_D_MAX's value feeding Θ; per-axis DSD clamps causing 49 m flyaway).

**The control parameters** (manuscript Table "Locked VDF-ASMC control parameters", supplemental.tex §S3.1): K_rp, K_ri, K_rd, h_rd | Ξ₂, p₂₀, p₂∞, ε_S | 𝒳, Γ, 𝒫, 𝒩, κ(0), ℰ | p₁₀, p₁∞, ξ₁, θ_cap | yaw set | k_R, k_Ω. Code-name mapping: 𝒳=Omega, Γ=Gma, p₁=rho_fov, ξ₁=l_fov.

**The saturation → cause → control-parameter-fix map (2026-06-03 diagnosis):**

| Saturating element | Root cause | Responsible control parameter | Fix (per-axis) |
|---|---|---|---|
| Cone clamp (d_min→0) | p₁∞=[80,80] px sized for MATLAB's tiny marker; PX4 board corners reach 165–234 px at touchdown (even on the perfect SP rep) | **p₁∞, p₁₀(v)** | RHOFOVINF_U×2.75, _V×3.75 (envelope = sensor edge); RHOFOV0_V×1.5 |
| Funnel barrier (ζ→3.66) | Ξ₂ τ=5 s assumes lag-free convergence; SITL lag leaves residual error past funnel contraction → constraint infeasible | **Ξ₂ (lateral)** | XI2_X×0.5, XI2_Y×0.5 (lateral τ→10 s) |
| κ runaway | Pure consequence of funnel saturation (σ pinned × leakage τ=33 s) | none needed | fixed by funnel fix |

**How to apply:** before proposing any clamp/limit/floor, write down which manuscript parameter's assumption is being violated and tune that instead. Batch 6 (b6A/B/C bundles) is the validation of this approach; the clamps were reverted to manuscript behaviour (DH_D_MAX=50 startup-guard, THETA_FLOOR=0) for it.


---

**CAMPAIGN CONCLUSION (2026-06-03, batches 6-7):** the saturation root-cause chase is complete.

| Fix attempted (all per-axis, manuscript params) | Result |
|---|---|
| p₁∞ → sensor edge (envelope) | ✅ **WORKS — keep.** Cone clamp geometric collapse gone. Trial 30: 0.78/0.66/0 TL |
| Ξ₂ lateral slower / p₂∞ wider | ❌ destabilizes — funnel width IS the gain (G⁻¹≈p/2) |
| 𝒩 ×0.1 (growth) | ❌ cancels in κ_eq = Θ·G·\|σ\|/𝒫; slower adaptation → longer saturation → worse |
| 𝒫 ×10 (leakage) | ❌ saturated numerator ≈1500 is 1000× normal; would need 𝒫≈9000 = no adaptation |
| K_rp/K_rd reductions (many forms) | ❌ frontier slide |

**The chain's true root is the loop lag** (h cannot track h_d transients; mid-flight tracking error → terminal 1/Z spike). No manuscript control parameter owns the lag — it violates the design assumption that the inner loop tracks commanded accel with negligible delay. Remaining fixes are architectural: [[dds-lag-fix-blocker]] (uXRCE-DDS rate path, built but blocked on rclpy) or MC_*RATE_P airframe-init edit. This completes — at the saturation-mechanism level — the historical conclusion in [[feedback_phase1_matlab_baseline]] / [[feedback-phase2-loop-latency]] that lag is the architectural ceiling.

---

**SUPERSEDED (2026-06-03 afternoon):** the conclusion above ("no control parameter owns the lag; only architectural
fixes remain") was WRONG — it missed that the lag can be accommodated by per-channel bandwidth matching
([[convergence-ordering]]) + the zf handoff (the controller's validated altitude envelope). Batch 9: 6 SP / 10 reps,
zero clamps, xy down to 9.6 mm. The lag is still there; the controller now just never demands more bandwidth than
the lagged chains can deliver, and never operates below the altitude where 1/Z noise exceeds the funnel.

**CORRECTION (2026-06-03, user):** the "zf handoff" referenced above was itself a limits-style mistake —
zf=0.2 m is the landing-gear height (MATLAB termination = gear contact = PX4 LandedState), NOT a controller
envelope. The controller must control through touchdown. Handoff reverted; Batch 9 results invalid.

---

**GENERALIZED LESSON (2026-06-03, bit twice — batches 6 and 11): the funnel width IS the gain, on every axis.**
In the barrier transform, G⁻¹ ≈ p/2 at small ζ — so widening any funnel component (p₂₀, p₂∞, or slowing Ξ₂)
to "make room" for a transient ALSO raises that axis's control gain proportionally, making the response to the
same error more violent. Lateral (batch 6): Ξ₂×0.5 → 2/5 catastrophic. Vertical (batch 11): P2INF_Z×2 →
touchdown vels 0.09-0.15 → 0.32-0.58. NEVER widen a funnel to absorb a transient. The admissible responses to
funnel saturation are: (a) reduce the demand feeding it (PID gains upstream), (b) reduce the response gain that
doesn't carry the coupling (Γ for that axis), or (c) accept brief saturation (per user: saturation hits are OK).

---

**SATURATION-AUDIT METHODOLOGY (user, 2026-06-03) + first results.** Complement failure root-causing with
`tools/analyze_saturation_audit.py`: measure the duty cycle of EVERY limit (code guards + manuscript limits)
across reps. Active limits = silent performance loss = a pointer to the parameter driving that signal out of
its linear regime. First audit (final-config family, n=25):

| Limit | SP reps | non-SP reps | implication |
|---|---|---|---|
| **cone clamp** | 3.8% | **15.8%** (up to 40% in worst reps; duty ~ monotonic with xy error) | the #1 silent loss; the SMC asks for more lateral authority than visibility allows |
| **σ outside ℰ** | 5.6% | **11.2%** | SMC in switching mode 11% of flight — manuscript intent is "enter boundary layer and stay"; **ℰ (boundary layer) has NEVER been tuned** |
| ζ barrier | 3.7% | 6.2% | funnel saturation (known) |
| **accel floor (−3)** | 0.2% | 3.7% (**19× ratio**) | z-SMC commands non-upward thrust — z-axis over-aggression |
| dh_d, w_u clamps | 0% | 1-2% | only active in failing reps (symptoms) |
| w_i clamp, yaw-int clamp, PID-int clamp, ∫ζ clamp | ~0% | ~0% | never fire — harmless, could note as dead guards |

The perfect rep (xy=0.048/vel=0.014) had ~0% on every limit — the "clean flight" signature. Tuning agenda from
the audit: (1) what makes the SMC ask >cone-allowed lateral accel; (2) ℰ_x/y widening (untested manuscript
parameter — trades chattering vs steady-state residual); (3) z-axis aggression (OMEGA_Z/P20_Z vs accel floor).

---

**EVENT-LEVEL ATTRIBUTION (user, 2026-06-03): "identify every time any saturation limit is hit, the reason
behind it, and if something can be done to avoid it."** Tool: `analyze_saturation_audit.py --events` —
lists every saturation event (limit, time, flight phase, duration) with an automatic reason snapshot.

First event-level findings (final-config family):

| Event signature | Where it appears | Reason (auto-attributed) | Avoidance |
|---|---|---|---|
| cone clamp "large ask", brief (≤110ms), terminal | even SP reps | SMC asks 4-8 m/s² for sub-0.2 errors — funnel G-amplification near contraction | inherent to funnel design; brief + harmless |
| cone clamp "small allowance" (d_min=0), LONG (≥1s), mid-flight | bad reps only | lateral error displaced marker to envelope edge → deadlock signature | prevent mid-flight lateral error growth (tracking variance — the open problem) |
| σ/ℰ z-axis, 100-600ms, at touchdown | every rep | gear-contact flow transient (h_e_z ratio 0.5-0.8) | touchdown physics; possibly ℰ_Z widening (untested manuscript param) |
| accel floor, ~500ms, terminal | hard-touchdown reps | z-SMC commands 50+ m/s² down in response to touchdown flow spike | z response gains: OMEGA_Z / E_Z (GAMMA_Z×0.5 already applied) |

**The diagnostic workflow going forward (standard procedure):**
1. `analyze_saturation_audit.py --glob <bundle>/rep*` — duty-cycle ranking, SP vs non-SP
2. `analyze_saturation_audit.py --events <worst reps>` — per-event reasons
3. For each frequent event type: trace the reason → the manuscript parameter that owns it → tune that
4. `analyze_explosion_chain.py` only for catastrophic failures (the chain analysis)