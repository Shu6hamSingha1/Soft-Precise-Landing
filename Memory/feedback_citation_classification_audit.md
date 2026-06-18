---
name: Verify each cited paper's classification (PBVS / IBVS / optic-flow / etc.) before quoting it
description: Locked 2026-05-02. Caught a real defect — `\cite{cho2022}` was sitting in §I Para 2 under "PBVS approaches rely on explicit 3-D pose reconstruction" when cho2022 is in fact a feed-forward IBVS controller. Rule: every citation in a framework-classifying sentence must be cross-checked against `project_naming_decisions.md` and the comparison-study labels.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## The defect (2026-05-02)

§I Para 2 of `manuscript.tex` originally read:

> "PBVS approaches rely on explicit 3-D pose reconstruction from LiDAR, stereo cameras, or GPS \cite{salehi2021, cho2022}, and degrade in GPS-denied environments..."

`\cite{cho2022}` is the **feed-forward IBVS** controller (Cho et al.), part of the four-baseline comparison study and consistently labelled "FF-IBVS" in `project_baseline_final_results.md`, `project_chen2025_limitation.md`, and the comparison harness. Citing it as a PBVS exemplar inverts its actual classification.

The user called this "a serious lapse" and explicitly asked for an audit of all citations to avoid such mistakes.

## Rule

**Every citation that asserts a paper belongs to a framework class (PBVS, IBVS, optic-flow regulation, learning-based, adaptive observer, etc.) MUST be cross-checked before being placed.** The check is:

1. What is the paper's framework? Look up:
   - `project_naming_decisions.md` (paper / controller name epoch)
   - `project_comparison_study.md`, `project_baseline_final_results.md`, `project_baseline_paper_gains_failure.md` for the four comparison baselines (lin2022 = PBVS, zhang2026 = PBVS, chen2025 = IBVS, cho2022 = IBVS).
   - Reference papers under `References/` if the cite is not in the comparison set.
2. What does the surrounding sentence assert? Specifically, does it group the cited papers under one framework label?
3. If the assertion contradicts the paper's actual framework, fix immediately — either move the cite to the correct framework's sentence, drop it, or restructure the sentence.

## Known classifications (verified 2026-05-02 — full PDF audit pass)

| Cite | Framework | Verified from PDF | Notes |
|---|---|---|---|
| `salehi2021` | **IBVS — barrier-function visibility on robot manipulator** | ✓ — abstract: "novel constrained image-based visual servoing (IBVS) approach... barrier function". Eye-in-hand on Baxter robot, NOT autonomous landing. | Place in Para 4 (visibility constraints) only. NEVER cite under PBVS or autonomous-landing IBVS. |
| `lin2022` | PBVS (performance-constrained) for vision-based UAV landing | ✓ — abstract: "low-complexity position-based visual servoing (PBVS) design ... vision-based landing". Monocular vision sensor. | Comparison baseline. ✓ for §I PBVS sentence (with vision-based pose framing). Also cited as PPC exemplar in Para 4. |
| `zhang2026` | PBVS (with adaptive extended disturbance observer) for UAV landing | ✓ — discusses "the resolution limitations of the vision camera". Vision-based pose. Backstepping + AEDO. | Comparison baseline. ✓ for §I PBVS sentence. Also in Para 4 adaptive-observer critique. |
| `chen2025` | IBVS (with adaptive observer) | per memory + comparison study | Comparison baseline. Place in Para 4 adaptive-control critique. |
| `cho2022` | IBVS (feed-forward) for ship-deck landing | ✓ — title: "feed-forward image-based visual servoing". | Comparison baseline. Place in IBVS sentence — NOT PBVS. |
| `chaumette2006` | Visual-servoing tutorial — covers BOTH PBVS and IBVS | ✓ — abstract: "the two archetypal visual servo control schemes: image-based and position-based". | Cite as IBVS foundational reference. |
| `jabbari2014` (Asl 2014) | IBVS (adaptive, perspective moments) for underactuated UAV | ✓ — title: "An Adaptive Scheme for Image-Based Visual Servoing of an Underactuated UAV". | §I "extended to aerial platforms" sentence. |
| `lee2012` | IBVS (adaptive sliding-mode, virtual features) for quadrotor | ✓ — title: "Adaptive Image-Based Visual Servoing for an Underactuated Quadrotor System". Uses "virtual features". | §I aerial extensions. |
| `fink2017` | IBVS (virtual camera, image moments) for quadrotor | ✓ — title: "Dynamic Visual Servoing for a Quadrotor Using a Virtual Camera". Strong virtual-camera-abstraction match. | §I aerial extensions. |
| `xie2016` (Xie 2016.pdf) | IBVS (input-saturated, virtual camera) for UAV — **visibility-constraint enforcement via roll/pitch saturation** | ✓ — abstract: *"the visual target potentially leaves the real camera's field of view (FoV). To keep the visual target in the camera FoV, an input saturated law is proposed to sufficiently bound the roll and pitch of the vehicle."* IEEE T. Mechatronics 2017. | §I Para 4 (visibility constraints). Third distinct visibility-constraint mechanism alongside `salehi2021` (barrier on image features) and `lin2022` (PPC on tracking error). Closest prior art to MDF-ASMC's funnel-margin cone clamp — Xie's bound is *fixed*, ours is *state-dependent*. |
| `xie2016_2` | IBVS (state-transformation, virtual camera) for UAV | ✓ — abstract: "state transformation... a particular solution reduces to an established virtual camera approach". Stationary visual target. | §I Para 2 aerial extensions. |
| `xie2020` (Xie 2019.pdf) | IBVS (adaptive output-feedback, virtual camera) for quadrotor | ✓ — abstract: "moment image features which are deﬁned using a virtual camera... adaptive and compensates for a constant force disturbance... thrust constant, desired feature depth, and mass". | §I Para 2 aerial extensions. |
| `herisse2012` | Optic-flow VTOL landing on **moving platforms** (stabilization + vertical landing) | ✓ — abstract: "stabilization of the vehicle relative to the moving platform... regulation of automatic vertical landing onto a moving platform". Camera + IMU. **Zero mentions** of "field of view" / "FOV" / "visibility" / "saturation" — has no visibility-constraint enforcement. | §I optic-flow paragraph. The "stationary target" generalization is wrong for herisse2012 — split with izzo2011. |
| `izzo2011` | Optic-flow lunar landing (1-D vertical, stationary surface, constant-flow guidance) | ✓ — bibtex title: "Constant-Optic-Flow Lunar Landing: Optimality and Guidance". | §I optic-flow paragraph. ✓ Fits "purely vertical descent on stationary targets". |
| `singhal2025` | Optic-flow soft vertical landing (own prior work, stationary surfaces, single vertical output) | ✓ — abstract: "scale-independent adaptive control strategy... soft vertical landing on stationary surfaces... adaptive sliding mode control... optic flow error converges exponentially". Has experimental laterally-stabilized landing but lateral control is separate, not unified with optic-flow framework. | §I Para 3: cited in S2 alongside `izzo2011` for "purely vertical descent on stationary targets" (locked 2026-05-02). The earlier separate "Our own prior work was restricted to..." sentence (S4) was dropped for page-limit; `singhal2025` now appears as one of the limited prior works rather than a standalone "own work" callout. |
| `paris2020` (Paris 2019.pdf) | Quadrotor moving-platform landing in turbulent wind — EKF + visual fiducials + RHC + boundary-layer SMC | ✓ — abstract: "extended Kalman ﬁlter using simulated GPS measurements... visual ﬁducial system... boundary layer sliding controller... in the presence of unknown, but bounded, disturbances". | Para 4 adaptive critique. ✓ Strong fit for "heavy perception pipelines" + "presuppose disturbance class". |
| `bouazza2025` | **Vision-aided STATE ESTIMATION (NOT control)** — relative pose via cascade observer + complementary filter on SO(3) | ✓ — abstract: "estimating the relative position, orientation, and velocity... cascade observer with a complementary filter on SO(3)... linear Riccati observer... assuming there is a suitable communication channel". | Para 4 — separated from adaptive/learning **control** schemes. Cited as state-estimation pipeline in its own clause: "Recent state-estimation pipelines \cite{bouazza2025} reconstruct relative pose through vision–IMU fusion". |
| `kamath2026` | Physics-Informed Koopman Neural Operator + NMPC for visual servoing of multirotors | ✓ — abstract: "Physics-Informed Koopman Neural Operator (PI-KNO)... combines physics-informed constraints with real-time observations... NMPC framework". Trained offline on physics + data. | Para 4 adaptive/learning critique. ✓ Strong fit for "heavy perception" + "pre-identify model offline". |
| `khalil2002` | Textbook (cascade ISS, Lyapunov) | — | Used for theorem references. |
| `lee2010` | Geometric SO(3) attitude tracker | — | Cited by `control_formulation.tex` for the inner loop. |
| `sanchez-cuevas2017` | Aerodynamic ground effect model | — | Used in §IV.A disturbance model. |

## Definitional point: PBVS uses VISION, not LiDAR/GPS

**PBVS = position-based VISUAL servoing.** The "V" is "Visual" — the loop closes on 3-D pose **reconstructed from vision** (monocular pose estimation via PnP / fiducial markers, stereo depth, etc.). LiDAR-based or GPS-based pose control are **not visual servoing**; they are different sensor-modality classes.

**Implication for §I Para 2 PBVS sentence (locked 2026-05-02):**
> "PBVS approaches rely on explicit 3-D pose reconstruction from visual measurements \cite{lin2022, zhang2026}, and do not guarantee the precise touchdown under sensing degradation such as calibration drift, scale drift, or marker occlusion."

- Sensor: "visual measurements" (NOT "LiDAR, stereo cameras, or GPS" — that earlier wording was definitionally wrong).
- Failure modes: calibration drift (intrinsics), scale drift (monocular), marker occlusion (vision-target loss). NOT "GPS denial" — GPS isn't a PBVS sensor.

## Audit history

- **2026-05-02 (initial pass):** Fixed `cho2022` mis-classification (PBVS → IBVS). Moved to the IBVS aerial-platform list. Added `lin2022, zhang2026` to the PBVS sentence.
- **2026-05-02 (Para 2 PDF verification pass):** Audited Para 2 cites against actual papers:
  - **`salehi2021` was mis-classified as PBVS.** Verified from the abstract: "constrained image-based visual servoing (IBVS) approach". The paper is eye-in-hand on a Baxter robot manipulator, not an autonomous-landing paper. Removed from Para 2 PBVS list.
  - **PBVS sensor list was definitionally wrong.** Verified that `lin2022` and `zhang2026` use vision-based pose (not LiDAR/GPS). Updated to "explicit 3-D pose reconstruction from visual measurements".
  - **Para 2 S1 scoped to autonomous landing**: "Existing solutions for autonomous landing fall into..." — clarifies the taxonomy and rules out manipulator-IBVS papers like `salehi2021`.
- **2026-05-02 (Para 3 + Para 4 PDF verification pass):**
  - **`herisse2012` mischaracterized in Para 3.** The "purely vertical descent on stationary targets" generalization was wrong for herisse2012, which explicitly handles moving platforms (verified from abstract: "stabilization of the vehicle relative to the moving platform... regulation of automatic vertical landing onto a moving platform"). Para 3 split into per-paper claims: izzo2011 → "purely vertical descent on stationary targets"; herisse2012 → "rely on an external heading reference and assume an unrestricted field of view during the descent" (verified by full-text grep: zero mentions of FOV/visibility/saturation in herisse2012; the "external heading reference" framing replaces an earlier "IMU-aided attitude" framing that was self-inconsistent — MDF-ASMC also uses IMU for body-rate measurement).
  - **`bouazza2025` mis-grouped as adaptive control scheme.** Bouazza2025 is a state-estimation paper (cascade observer with complementary filter on SO(3)), not a control framework with switching gain or aerodynamic model. Para 4 critique restructured: bouazza2025 cited separately as state-estimation pipeline ("vision-IMU fusion"); the SMC/learning critique applies only to paris2020, chen2025, kamath2026, zhang2026.
  - **Para 4 critique scope tightened.** "Presuppose an a priori upper bound on the switching gain or disturbance" → "presuppose an a priori characterization of the disturbance class or switching gain" — broader phrasing covers both AEDO-style schemes (zhang2026) and SMC-style schemes (paris2020, chen2025).
- **2026-05-02 (Para 3 page-limit compression):**
  - **Para 3 compressed from 4 sentences to 2** for page budget.
  - S2 (was old S2): dropped — was redundant with S1 (just spelled out the `h = v/z` mechanism). Mechanism belongs in §II/§III, not §I.
  - S2's scale-ambiguity tail ("circumvents scale ambiguity entirely") dropped — secondary benefit; covered by §III normalised-feature derivation and contributions list.
  - Old S4 ("Our own prior work was restricted to...") merged into S3's cite list as `\cite{izzo2011, singhal2025}`. Loses the explicit "our own prior work" framing but saves a full sentence; `singhal2025` is still credited as one of the limited prior works.
  - Para 3 S1 wording: "approach distance" → "relative altitude" for cross-paragraph parallelism with Para 1 clause (b)'s "altitude".
  - **S1 cite list re-added (2026-05-02 final lock):** `\cite{herisse2012, izzo2011, singhal2025}` restored to S1. `singhal2025` now appears in both S1 (foundational framing) and S2 (vertical-stationary critique cite list) — accepted double-cite for the "natural sensing modality" claim per academic citation norms.
- **2026-05-02 (Para 4 trim + Para 5 sync):**
  - **Para 4 PPC critique merged** (S4 + S5 → 1 compressed sentence). New form: "PPC, however, has so far been applied to the image-plane error \emph{alone}, with no mechanism to prescribe the rate at which the UAV--target relative velocity decays. It answers the visibility question but not the soft-touchdown question." Replaced "vanishes" with "decays" (per the locked "no overclaim" rule). Para 4 now 6 sentences.
  - **Para 5 gap (ii) synced with Para 4 critique:** "require an *a priori* bound on the switching gain or disturbance" → "require an *a priori* characterization of the disturbance class or switching gain". Matches the broadened phrasing locked in Para 4 S6.
  - "An orthogonal line of work" (Para 4 S1) was considered for replacement with "A complementary line of work" but user kept "orthogonal" — confirmed as a deliberate term-of-art choice signalling separate-problem-axis (visibility-constraint enforcement is independent of control-output choice).
- **2026-05-02 (Para 7 reorder):** Contribution-summary sub-clauses reordered from (a, c, b, d) to (a, b, c, d) for visual parallelism with the contributions list. Each sub-clause retains parallel verb structure ("closes" / "removes" / "closes" / "characterises").
- **2026-05-02 (Para 2 → Para 3 boundary shift):** *"None of these frameworks achieve the functional requirement of the soft touchdown."* moved from end of Para 2 to start of Para 3. Para 2 narrows to PBVS/IBVS taxonomy with IBVS-specific closing; Para 3 opens with the gap statement, then introduces optic flow as the answer.
- **2026-05-02 (Para 4 → split into Para 4 + new Para 5):** Old Para 4 contained two topic clusters (visibility-constraint enforcement S1–S4, recent adaptive/learning S5–S6). The two clusters are now in separate paragraphs:
  - **Para 4 (visibility-constraint enforcement only):** barrier-function constructions + PPC critique, ending with *"It answers the visibility question but not the soft-touchdown question."*
  - **NEW Para 5 (vision-IMU + adaptive/learning extensions):** Opens with *"Recent extensions introduce vision–IMU sensor fusion, adaptive observers, and learning-based control."* Then per-cite split: `bouazza2025` for state-estimation; `paris2020, chen2025, kamath2026, zhang2026` for adaptive/learning. Closes with the "None of these is available at flight time" gap statement.
  - The **earlier dropped sentence** *"PPC, however, has so far been applied to the image-plane error alone, with no mechanism to prescribe the rate at which the UAV-target relative velocity decays."* — was redundant with *"It answers the visibility question but not the soft-touchdown question."* and the broader Para 6 gap (i). Removed.
  - The **lead-in for new Para 5** went through three iterations: (i) "Other relevant work in autonomous landing addresses relative-state estimation and adaptive control" — flagged as misleading because Paras 2-4 already cover both topics; (ii) "Other relevant work covers..." — same problem; (iii) **locked**: "Recent extensions introduce vision--IMU sensor fusion, adaptive observers, and learning-based control" — accurately describes what's distinctive about the cited papers.
- **2026-05-02 (FoV abbreviation introduction order):** First occurrence of FoV in `manuscript.tex` was at L103 (Para 3) without abbreviation introduction; the introduction was at L105 (Para 4) — wrong order per `feedback_abbreviation_first_use.md`. Fixed: introduction moved to L103 (*"unrestricted field of view (FoV) during the descent"*); L105 uses abbreviation only. `supplemental.tex` similarly fixed (L163: *"inside the field of view (FoV)"*).
- **2026-05-02 (Para 4 expansion to three visibility-constraint mechanisms):**
  - **`xie2016` added to Para 4** as a third visibility-constraint mechanism. Verified from Xie 2016 abstract: *"To keep the visual target in the camera FoV, an input saturated law is proposed to sufficiently bound the roll and pitch of the vehicle."*
  - The three mechanisms now in Para 4: barrier on image features (`salehi2021`) → unbounded near boundary; PPC on tracking error (`lin2022`) → smooth funnel; input saturation on attitude (`xie2016`) → fixed roll/pitch bound. **`xie2016`'s fixed-bound framing positions it as the closest prior art to MDF-ASMC's state-dependent funnel-margin cone clamp.** Now also picked up in §III where the cone clamp is formally introduced.
  - Closing S5 of Para 4 generalised: *"It answers the visibility question..."* → *"These methods answer the visibility question..."* — covers all three mechanisms now.
  - **Lin 2022 PPC critique sharpened:** *"PPC forces the image error..."* → *"PPC bounds the tracking error..."* — accurate for `lin2022`'s PBVS-PPC on position error (the earlier "image error" framing was inaccurate; lin2022 is PBVS, not IBVS-PPC).
- **2026-05-03 (Para 4 final lock — direct visibility-constraint mechanisms only):**
  - **PPC sentence (lin2022) removed from Para 4.** lin2022's PBVS-PPC operates on the tracking error (position), not directly on visibility — visibility preservation is indirect. Per the user's scope rule for Para 4 ("frameworks which enforce visibility constraints **directly** in their control formulation"), lin2022's PPC was moved out of §I literature review.
  - **`lin2022` PPC technique now cited in `control_formulation.tex` §III-A2** (`ppc optic flow: section`) where the optic-flow funnel is built on the PPC paradigm: *"We follow the prescribed-performance control (PPC) paradigm \cite{lin2022}."* This properly attributes the PPC foundation where it is actually used in the proposed design.
  - **Para 4 S1 sharpened**: *"An orthogonal line of work enforces..."* → *"An orthogonal line of work **directly** enforces..."* — explicit scope marker that this paragraph is restricted to direct visibility-constraint mechanisms.
  - **Para 4 S3 critique on `xie2016` rephrased**: *"...but the bound is fixed rather than state-dependent."* → *"...but the fixed bound cannot tighten as features approach the FoV edge."* — names the specific deficiency (no FoV-margin responsiveness) without explicitly invoking "state-dependent" terminology, which is reserved for the §III formal introduction of the funnel-margin cone clamp.
  - **Para 4 final form (4 sentences):** S1 topic intro with "directly enforces"; S2 barrier (`salehi2021`); S3 input saturation (`xie2016`) with FoV-edge tightening critique; S4 plural closing ("These methods answer the visibility question but not the soft-touchdown question").
- **2026-05-03 (Para 6 — research-gap-level reframing):**
  - **Reframed gaps from "current solutions don't do X" (solution-implicit) to PROBLEM-LEVEL research gaps** (i.e., problems that remain unsolved, named without referencing any specific solution mechanism). The user's instruction: *"these points doesn't point out the research gaps in the problem statement, but it points out the gap between current solution and our solution."*
  - Hedge added: *"To the best of the authors' knowledge, the literature exhibits three research gaps."* — covers all three gaps once.
  - Each gap now reads at problem-statement level:
    - (i) "The soft-precise touchdown on a moving target using only monocular vision remains an open problem." — task-level, no solution mechanism named.
    - (ii) "Online identification of the full uncertainty on the optic-flow dynamics ... is unresolved." — sub-problem-level, names the system being solved (optic-flow dynamics) but not the leakage-adaptive SMC mechanism.
    - (iii) "A closed-loop stability guarantee that simultaneously certifies the soft touchdown, the precise touchdown, and target visibility is missing." — guarantee-level, no specific certificate structure named.
  - **"image-kinematic plant" replaced with "optic-flow dynamics"** — the canonical paper-internal term verified against `control_formulation.tex` §II.C L110: *"the unknown time-varying scale $\beta=1/\,^\mathcal{V}z_\text{t}$ multiplying the actuation in the optic-flow dynamics \eqref{h_e_dot_1: equation}, so the controller cannot pre-cancel it."*
  - Closing *"summarised qualitatively in Table~\ref{tab:comparison}"* cross-reference initially dropped during shortening, then restored at user request.
  - Para 6 now 5 sentences (S1 hedge intro, S2/S3/S4 = three gaps, S5 closing with table cross-ref).
  - **Mapping to Para 8 (gap-contribution) still aligned:** (a) closes (i)+(ii); (b) supports (i); (c) closes (iii); (d) characterises design margin.
- **2026-05-03 (§II.B section-title parallelism rename):**
  - **"Target Virtual Orientation" → "Virtual Target Orientation"** in `control_formulation.tex` §II.B.2 (subsection title at L70 + introduction sentence at L57).
  - Reason: structural parallelism with §II.B.1 "Normalised Target Position" — both subsection titles now follow the `[adjective] + "Target [Property]"` pattern.
  - Semantic support: *"virtual target"* can refer to the target's projection on the virtual image plane, and α is exactly the orientation of that projection (computed from the N virtual feature points). Both readings of "Virtual Target Orientation" — *"Virtual" modifying "Target Orientation"* and *"(Virtual Target)" + "Orientation"* — converge on the same physical meaning.
  - Memory `feedback_target_image_parameters.md` updated to use the new name.
  - **TODO:** `block_diagram_v3.pdf` sub-block label "Target Virtual Orientation" needs renaming on next regen.
- **2026-05-03 (Para 5 final lock — five sentences with parallel inline critiques):**
  - **Para 5 lead-in (S1) rephrased** to bridge from Paras 2–4 and match the S2/S3a/S3b structure: *"Beyond the above frameworks, recent work introduces vision--IMU sensor fusion, adaptive control, and learning-based control."*
  - **S3 split into S3a + S3b** for clean S1↔S2/S3a/S3b 1:1:1 mapping:
    - S3a (adaptive control): `paris2020, chen2025, zhang2026` — *"presuppose an a priori characterization of the disturbance class or switching gain"*. paris2020 placed in adaptive group based on its boundary-layer SMC + bounded-disturbance assumption.
    - S3b (learning-based control): `kamath2026` alone — *"depend on heavy perception pipelines and pre-identify an aerodynamic model offline"*. Single-paper sentence, defensible because kamath2026 is the only deep-learning-based UAV-landing cite in the bibliography.
  - **S2 (`bouazza2025`) inline critique added** for parallelism with S3a/S3b: *"...but require an inter-platform communication channel and known target geometry."* Verified against bouazza2025 abstract (*"assuming there is a suitable communication channel"* + planar-surface bearing/normal extraction).
  - **S4 sweeping closing** ("None of these is available at flight time on an unknown target with unknown depth scaling") now serves as a clean recap rather than carrying any single cite's failure mode alone — every cite has its inline critique named.
- **All §I cites now PDF-verified.** Introduction is editorially complete with 9 paragraphs:
  - Para 1: motivation + functional requirements (4 sentences)
  - Para 2: PBVS/IBVS taxonomy (4 sentences)
  - Para 3: optic flow (3 sentences)
  - Para 4: visibility-constraint **direct** mechanisms (4 sentences — barrier (salehi2021), input saturation (xie2016); lin2022's PPC moved to §III)
  - Para 5: vision-IMU + adaptive + learning extensions (5 sentences — clean S1↔S2/S3a/S3b 1:1:1 mapping with parallel inline critiques)
  - Para 6: three gaps (6 sentences)
  - Para 7: contributions list (enumerated)
  - Para 8: contribution-gap mapping (1 parallel-enumeration sentence)
  - Para 9: paper structure outline (4 sentences)
  - **All gap labels (i, ii, iii) and contribution labels (a, b, c, d) stable across the renumbering.** No paragraph-number references in §I, so cross-references intact.

## How to apply

- Before pasting a `\cite{...}` in any sentence that asserts a class label, look the paper up in this table.
- If the paper is NOT in this table, look it up in the source PDF / authors' abstract before using.
- If you change the §I citation pattern (move cites between sentences, remove cites, add new ones), update this table in the same edit.
- **For sensor-modality claims (LiDAR, GPS, stereo, monocular):** `Visual servoing = vision-based`. Never write "PBVS uses LiDAR" or "PBVS uses GPS" — those modalities are outside visual servoing. Use "visual measurements" as the umbrella sensor descriptor for both PBVS and IBVS.
- **For autonomous-landing scoped paragraphs:** restrict cites to autonomous-landing papers; `salehi2021` (manipulator) and similar non-landing references belong in mechanism-specific paragraphs (e.g., barrier functions in Para 4), not in the autonomous-landing taxonomy paragraph.

## Audit history

- **2026-05-02:** First audit pass. Fixed `cho2022` mis-classification (PBVS → IBVS). Moved to the IBVS aerial-platform list. Added `lin2022, zhang2026` to the PBVS sentence (correctly classified PBVS). No other §I citation defects found in this pass.

## Related conventions

- `project_naming_decisions.md` — paper / controller naming epochs.
- `project_comparison_study.md` — locked four-baseline comparison list.
- `project_baseline_final_results.md` — per-baseline results table.
- `feedback_validate_critiques_against_cited_works.md` — sweeping "no prior work does X" claims must be cross-checked against each cite.
