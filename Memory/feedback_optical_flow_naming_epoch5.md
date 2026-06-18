---
name: optical-flow-naming-epoch5
description: "Locked 2026-06-11: optical flow = measured pixel-velocity FIELD; h = image velocity, w = image angular velocity (extracted from the field via L_s pseudo-inverse); optical-flow compound names (funnel/dynamics/error/Theorem 1 title) KEPT"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 835634e5-9079-4b74-95af-775a75d60862
---

Naming epoch 5 (user ruling, 2026-06-11), supersedes the h-is-optical-flow usage of the 2026-06-10 single-file epoch and vindicates [[reference_optical_flow_vs_image_velocity]] (2026-06-09):

- **Optical flow** = the measured velocity field $\,^\mathcal{V}\dot{\hat{\boldsymbol{r}}}_i(t)$ of the image feature points (ṡ at centroid level). A raw image quantity, like the image features.
- **$\boldsymbol{h}$ = image velocity** ($\,^\mathcal{V}\boldsymbol{v}_\text{t/b}/\,^\mathcal{V}z_\text{t}$); **$\boldsymbol{w}$ = image angular velocity** — both EXTRACTED from the measured optical flow via the interaction-matrix pseudo-inverse. Never call h "the optical flow" or "translational optical flow".
- **Compound names KEPT** (user-explicit): "optical-flow funnel", "Optical-Flow Dynamics" (§II.B.4 heading), Theorem 1 "Adaptive Optical-Flow Funnel Invariance", "optical-flow disturbance d_h", "optical-flow error". Rationale: regulating h IS regulating the optical flow.
- Conceptual parallel (user's words): image features → image pose (position s + orientation α); optical-flow field → h, w. s/α regulate image features for precise lateral convergence; h/w regulate optical flow for soft touchdown.
- Objective item 2 pattern: "the optical flow is regulated through the image velocity h such that...".

**Why:** physically the field is what the camera measures (Hérissé's usage); h is a generator recovered from it. Calling h "optical flow" inverted the measurement chain.

**How to apply:** §III review must align "desired optical flow h_d" and any "optical flow h" phrasing to epoch 5 (h_d = desired image velocity, or "regulated through"), while preserving the compound names above. Abstract/§I "regulate optical flow" claims about prior work stay (they regulate flow). [[feedback_funnel_naming]] [[feedback_target_image_parameters]]

**Symbol-reduction addendum (2026-06-11, user approved "All"):** ~15 symbols removed from main text. GONE: B_v, B_ω, c_v, c_ω (Newton–Euler written explicitly), d_max, ε_c, a_dz^min/α_a/τ_a (safeguards → Appendix C), r̂_d (image point error r̂_e ≜ ^V r̂ directly), s_d/ṡ_e (PID outputs ṡ_d, desired lateral image-position derivative), componentwise twins γ_k/χ_k/η_k/ρ_k/ξ_2k ("diagonal X ≻ 0" phrasing; ε_k KEPT for proof), **c̃_h ELIMINATED** — plant (h dot) written with explicit terms −ψ̇ê₃×h −(h·ê₃)h; §III defines c_h ≜ those terms − ḣ_d directly. SUPERSEDES feedback_c_tilde_h_convention.md. Yaw gains introduced as "yaw counterparts of (Γ,X,N,P,E)". k_R/k_Ω/e_R/e_Ω now appendix-only. Appendix C = "Implementation Safeguards and SO(3) Attitude Tracker" (label 'implementation appendix: section'): acceleration floor + LPF eqs + R_d construction + SO(3) errors/torque.

**Addendum (2026-06-11, user ruling):** "virtual image feature points" → "image feature points" (drop virtual; frame prefix ^C/^V disambiguates). "virtual" is reserved STRICTLY for: virtual image plane, virtual camera frame, virtual-camera (abstraction). Supersedes epoch-2's "virtual image feature point"/"virtual feature point" vocabulary in feedback_no_landmark_term.md. Also: h_rd = "desired image radial velocity". "output error" (singular) = the collective regulated-error vocabulary in abstract + §I contributions (not "image parameter errors").
