---
name: abstract-gap-claim-validated
description: "Validated 2026-06-10 (broadened to full §I cite list same day); abstract sentence 2 gap claim — \"either/or, not both simultaneously\" + purpose attributions hold across all cited image-based landing works; depth/state-feedback and instability clauses overclaim"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 1d38ebb9-3a55-4a1a-9447-9b57f89010c8
---

## Validated claim (use this in abstract sentence 2)

> Most prior IBVS landing controllers either regulate image features or optical flow independently, without addressing both simultaneously.

This is validated against all four baselines.

## Rejected clauses and why

**"while relying on metric depth estimates or target state feedback"** — OVERCLAIM.
- Lin 2023 (`ctrl_Lin2023.m`): inputs are circle-moment features + UAV own velocity only. No metric depth, no target state. Circle-feature IBVS is inherently scale-free.
- Cho 2022 (`ctrl_Cho2022.m`): requires `C_s_tc` (target position in camera frame = metric depth) AND `I_v_t` (target velocity). True for Cho, but not universal.
- Herisse 2012: optical flow is depth-normalized; no metric depth required.
- Verdict: true for PBVS baselines (Lin 2022, Zhang 2026) and Cho 2022, false for Lin 2023 and Herisse 2012. Cannot use universally.

**"while suffering from instabilities induced by the coupled nonlinear landing dynamics"** — OVERCLAIM.
- PBVS controllers (Lin 2022, Zhang 2026): don't use image-based landing kinematics at all — coupling is irrelevant to them.
- Lin 2023: explicitly uses rotation-invariant circle features to *decouple* translation from rotation. Saying it suffers from coupling contradicts its stated contribution.
- Herisse 2012: treats vertical as 1D decoupled — this is a scope limitation, not a documented instability.
- "Instabilities" requires a citation showing divergence, not just absence of a coupling-aware proof.

## Broadened validation (2026-06-10, full §I citation list)

Final sentence validated with purpose attributions:

> Most prior image-based landing controllers either regulate image features for precise lateral convergence or optical flow for soft touchdown, but not both simultaneously.

- **Flow branch** (herisse2012, ho2018, izzo2011, baird2013, alkowatly2015, singhal2023, singhal2025): regulate optical flow only; none regulates image features. Hérissé's lone "centroid and optic flow" hit is in its own reference list (Hamel/Mahony hover work — not landing, not cited by us). singhal2025 confirmed flow-only by manuscript line 117 ("extends it to soft-precise landing").
- **Feature branch** (lin2023, cho2022, lee2012): regulate image features only; Lee 2012 text has zero optical-flow mentions; Lin/Cho confirmed via MATLAB port inputs.
- **Out of scope** (covered by "most" / "landing"): PBVS works (lin2022, zhang2026, zhao2021, bouazza2025) regulate metric pose, neither branch; kamath2026 (zero landing/touchdown mentions), salehi2021, xie*, jabbari2014, fink2017 are servoing, not landing controllers.
- **Known nuance**: Hérissé also regulates horizontal flow (velocity matching) but achieves no lateral *position* convergence — consistent with §I para 7's "without guaranteeing precise lateral convergence".
- **Own-velocity reliance (validated 2026-06-10 for §I para 8)**: both lateral-convergence IBVS baselines consume the UAV's own velocity — Lin 2023 port (UAV own velocity input) and Cho 2022 port (`I_v_c`, UAV velocity in NED). Claim "rely on the UAV's own velocity feedback~\cite{lin2023,cho2022}" is safe. VDF-ASMC needs neither vehicle's absolute state (own-state independence, from the Manuscript Notes pitch).
- **Latent counterexample class**: Hamel/Mahony "centroid + optic flow" hover/terrain-following line combines both signal types but for hover, not landing — "landing controllers" scoping is load-bearing; don't drop it.

## §I full citation validation (2026-06-10, fixes applied)

Key durable facts established (from archived reference texts + web):
- **cho2022 is cooperative**: ship velocity feed-forward comes from Kalman fusion of SHIP-MOUNTED GPS + onboard vision + ship dynamic model. Never write "estimated from onboard sensors" for Cho. Also has a heuristic FoV mechanism (adaptive IBVS gain to keep features in FOV) and consumes metric depth (port input C_s_tc; paper discusses depth estimation error).
- **zhao2021 is IBVS** (not PBVS/relative-pose): backstepping-like cascade + robust compensation, sonar for image depth, vision estimates the QUADROTOR's own position/velocity. Not prescribed-performance, not adaptive. Its text supports PBVS-calibration-sensitivity claims (says IBVS robust to calibration errors, PBVS unsuitable with monocular).
- **gain∝altitude is Ho 2018's finding, NOT Hérissé's** — Hérissé text has no gain–altitude statement; Singhal 2025 attributes it to its [13]/[15]. Ho 2018 detects height via self-induced oscillations at landing start (not direct measurement), sets gains proportional to detected height, reduces exponentially.
- **isilak2025 cooperative** (platform shares GPS position/velocity/orientation, Kalman-fused); **bouazza2025 cooperative** (platform IMU data over communication channel); **angelis2026 is RL** (policy on visual keypoints from landing surface → attitude+thrust).
- **zhao2012** = Singhal 2025's published reference [6] for the same rangefinder/ultrasonic-limitation claim (peer-review precedent).
- **"IBVS inherently scale-independent" is an overclaim** — softened 2026-06-10 to "reducing its dependence on metric scale information" (classic IBVS Jacobians need depth; Cho needs metric depth).
- **Table I rewritten from scratch + revalidated 2026-06-10**: rows = 8 §I-surveyed works; columns = Scale-Free / Own-State-Free / Stability Guarantee / Soft Touchdown / Precise Lateral / Target Visibility. All cells source-verified. Key evidence: Lin 2022 defers FoV to FUTURE WORK (own conclusion); Zhang 2026 text has ZERO soft/touchdown/FoV mentions; Lin 2023 full PDF has zero soft/touchdown hits and only claims to "potentially improve the camera visibility" (hedged, ✗); Hérissé text confirms camera+IMU "minimum sensor suite" + Lyapunov exponential proofs; Angelis success threshold is velocity error < 1.5 m/s (velocity matching, NOT soft) and has an FoV perception-reward term; Cho has adaptive-gain FoV mechanism. cho2022 + angelis2026 get $\circ$ (heuristic visibility mechanism, no guarantee) per caption legend. The old table's entries (pre-2026-06-10) were user-flagged unreliable and are fully superseded.

## Table I column definitions (dimensional litmus, 2026-06-10)

- **Scale-Free** = no metric translational quantity anywhere in the control loop (no depth/range, no metric altitude, no m/s velocity); all translational feedback must be dimensionless, radians, or 1/s (flow). Lin 2023's features are depth-free but its `e_v = I_v_c − v̂` loop is in m/s → ✗. Property of the whole loop, not the feature.
- **Own-State-Free** = no UAV inertial position or translational velocity feedback. IMU attitude/body-rates exempt (everyone uses them). Ho 2018 is the separating example: Own-State ✓ (flow-only sensing) but Scale ✗ (metric height estimate drives the gain schedule).

## How to apply

- Use the validated either/or claim alone as sentence 2 in the abstract.
- If the coupling argument is needed, it belongs in §I para 5 (where it explains *why* the problem is hard), not retroactively as a universal failure mode of prior controllers.
- If the depth/state limitation is relevant, qualify it: "Many prior controllers additionally rely on metric depth estimates or target state feedback" (cites: lin2022, zhang2026, cho2022 — NOT lin2023).
