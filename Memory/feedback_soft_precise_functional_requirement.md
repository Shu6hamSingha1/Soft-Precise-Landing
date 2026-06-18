---
name: feedback_soft_precise_functional_requirement
description: Soft-precise landing functional requirement — qualitative two-clause statement in §I Para 1, numerical thresholds (0.08 m / 0.20 m/s / z_f=0.20 m, 3-D ‖v_rel‖) in §IV.A+Table II; defining-genitive "achieve the functional requirement of [property]" attribution
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

Consolidated soft-precise functional-requirement convention. Merges three feedback memories: the §I-vs-§IV location split, the four-sentence §I Para 1 chain, and the defining-genitive attribution pattern. Re-locked 2026-05-02.

## Numerical thresholds and the §I-qualitative / §IV-numerical split

Soft criterion is **3-D `‖v_rel‖`**, not vertical-only. The numerical thresholds live in §IV.A and Table II of `results.tex`; §I Para 1 carries the qualitative functional requirement only.

### §IV.A (results.tex) and Table II — numerical thresholds
> "A run terminates when the UAV altitude above the target first falls below `z_f = 0.20 m`. The touchdown is *precise* when `‖^I e_xy‖ ≤ 0.08 m` and *soft* when `‖v_rel‖ ≤ 0.20 m/s`; a touchdown is termed *soft-precise* when both criteria hold simultaneously."

Table II rows:
- Above-target gap `z_f` = 0.20 m
- Precise xy-tol. `xy_prec` = 0.08 m
- Soft speed bound `v_soft` = 0.20 m/s

### Abstract — neither location
The abstract uses "soft"/"precise" without redefining them. It may quote the `0.08 m` threshold in a benchmark comparison ("well inside the 0.08 m precise threshold") — that's a forward reference to §IV.A, not a definition. Do NOT redefine the thresholds inside the abstract.

### Why §I qualitative + §IV numerical
- §I describes the **functional requirement** (qualitative; what the controller must accomplish). Generic; doesn't bind a specific simulation.
- §IV.A operationalises the requirement with **specific simulation values**. Numbers live with the simulation that uses them. Matches the methodology-vs-simulation principle (`feedback_preliminaries_methodology.md`).

### Soft criterion is 3-D, not vertical-only
`‖v_rel‖` is the **3-D magnitude** of the UAV–target relative-velocity vector. MATLAB and `_analyze_results_for_tex.py:229` use `norm(...)` over the full 3-D vector. The four Python plot scripts also use 3-D (fixed 2026-04-30; previously `|v_z_rel|` 1-D vertical → false-positive triangles).

### Threshold value evolution
- Earlier drafts cited 0.10 m precise threshold.
- **Locked 2026-04-19** at 0.08 m ("50/50 precise+soft at 8 cm"). All current code and tex cite 0.08 m.

### How to apply (location)
- §I Para 1 should NEVER carry numerical thresholds — only the two-clause functional requirement.
- §IV.A intro paragraph carries the operational definitions (precise / soft / soft-precise) with numbers.
- Table II carries the threshold values as separate rows.
- Abstract may forward-reference numbers as benchmarks but should not redefine.
- Any sentence claiming "soft" or "precise" in the body should resolve to §IV.A operationalisation, not §I.

## The four-sentence §I Para 1 functional-requirement chain (locked 2026-05-02)

§I Para 1 of `manuscript.tex` is a four-sentence motivation→requirement chain (no numbers):

> S1 (context): "Unmanned Aerial Vehicles (UAVs) are increasingly tasked with autonomous landings on mobile platforms such as ships and ground vehicles."
>
> S2 (two failure modes with concrete consequences): "An imprecise landing risks missing the platform entirely or grazing its edge, while a hard landing risks airframe and payload damage on contact."
>
> S3 (dual demand + first italic introduction of terms-of-art): "Hence, the terminal landing phase demands both \emph{precise} positioning and a \emph{soft} touchdown, despite operating in the regime where wind, ground effect, and measurement noise are most pronounced."
>
> S4 (functional requirements): "For soft-precise landing, the functional requirements are (a) the smooth lateral approach of the UAV to a close vicinity of the target before touchdown, and (b) the simultaneous convergence of the UAV's altitude and velocity to those of the target at touchdown."

**Two-clause structure (S4):**
- (a) **Lateral position only** — the UAV's lateral position arrives close to the target's lateral position over the descent (before touchdown). Vertical position is *not* part of clause (a) because the precise criterion is `‖^I e_xy‖`, not 3-D position. NEVER say "relative position" / "UAV's position" alone in clause (a) — that implies 3-D.
- (b) **Altitude AND velocity, simultaneously, at touchdown** — rules out two failure modes: residual altitude with near-zero vertical velocity (incomplete landing) and non-zero vertical velocity at touchdown (crash). Both failures satisfy one axis in isolation but violate the coupling. The word "simultaneous" enforces that altitude and velocity reach their targets at the same moment.

Matches the dual-funnel architecture: target image funnel (`p_1`) closes the lateral chase during descent (clause a); optic-flow funnel (`p_2`) shapes descent rate so altitude and velocity arrive together at touchdown (clause b).

**Para 3 cross-paragraph parallelism (locked 2026-05-02):** §I Para 3 S1: "*Optic flow* inherently regulates both **relative altitude and relative velocity** to the target, making it a natural sensing modality for the soft touchdown." The "relative altitude and relative velocity" wording **deliberately mirrors** Para 1 clause (b). Earlier drafts used "approach distance" (from `singhal2025`) — imprecise in 3-D lateral-chase + moving-target context.

### How to apply (Para 1)
- **Para 1 must contain all four sentences in order.** Do not collapse S2+S3 — the failure-mode → requirement mapping is the motivation hook.
- **S2 names two distinct failure modes with concrete consequences.** Avoid abstracting back to "imprecise or hard landings risk damage" (lumped form).
- **S3 introduces \emph{precise} and \emph{soft} as italicized terms-of-art at first appearance.** Drop the italics anywhere else they are introduced first (e.g., abstract).
- **S4 clause (a)** must say "lateral" (or "lateral position" / "horizontal"); **clause (b)** must couple altitude and velocity; lead-in "For soft-precise landing, the functional requirements are (a) ... and (b) ..."; keep "requirements" plural.
- Numbers (`0.08 m`, `0.20 m/s`, `z_f = 0.20 m`) NEVER appear in §I Para 1; they live only in §IV.A and Table II.
- "Close vicinity of the target" is the canonical destination phrase for both clauses (not abstract "vicinity of the origin"). "At touchdown" anchors clause (b) to the moment, not a window.
- "Despite operating in the regime where wind, ground effect, and measurement noise are most pronounced" is the locked disturbance-regime tail in S3 ("While" was rejected — near-duplicate connector with S2).

### Wording history (do not regress)
| Date | Sentence form | Status |
|---|---|---|
| pre-2026-05-01 | Numerical thresholds + `z_f = 0.20 m` directly in §I | Superseded — moved to §IV.A |
| 2026-05-01 (interim) | "(a) the UAV–target relative position to a small vicinity of the origin before landing, and (b) the 3-D relative velocity to a small vicinity of the origin..." | Superseded — "relative position" wrong (only lateral); "vicinity of the origin" too abstract |
| 2026-05-01 (transitional) | "(a) the UAV's lateral position to a close vicinity of the target before touchdown, and (b) the UAV's velocity to a small vicinity of the target's velocity..." | Superseded — no altitude–velocity simultaneity at touchdown |
| 2026-05-02 (interim) | "Soft-precise landing therefore requires (a) the smooth lateral approach..., and (b) the simultaneous convergence of the UAV's altitude and velocity to those of the target at touchdown." | Superseded — clause content correct but surrounding motivation collapsed S2+S3 |
| **2026-05-02 (locked)** | Four-sentence chain S1–S4 above. | Current |

### Flags accepted by user as final wording (2026-05-02) — do NOT re-raise
1. **S3 "precise positioning" vs "soft touchdown" parallelism slip.** "Positioning" (state) vs "touchdown" (event) is a mild non-parallelism. User's verdict: acceptable as-is; precise→position / soft→touchdown mapping is intentional.
2. **S3 "despite operating in the regime..." participle.** "Operating" has no explicit subject. User's verdict: acceptable as a free participle; do not propose tightening to "despite a regime where...".

The user's reasoning (2026-05-02): explicitly identified two failure modes the previous wording could not rule out (residual altitude with near-zero vertical velocity = incomplete landing; negative vertical velocity at touchdown = crash) and asked for the simultaneity to be made explicit. Verbatim: "Does the final version conveys this message that the relative velocity and relative altitude should simultaneously convergence to origin. Because Deviations, such as residual altitude with near-zero vertical velocity indicating an incomplete landing or negative vertical velocity upon touchdown indicating a crash, are unacceptable."

## Defining-genitive attribution: "achieve the functional requirement of [property]" (locked 2026-05-01)

When attributing a touchdown property (precise, soft, soft-precise) to a controller or framework, write **"achieve the functional requirement of the [property] touchdown"** — NOT "achieve [property] touchdown" or "deliver a [property] landing".

**Why:** A touchdown qualifies as *precise* (or *soft*, *soft-precise*) only when its functional requirement — the smooth convergence pattern defined in §I Para 1 — is achieved. This is a *defining genitive*: "of the precise touchdown" identifies *which* functional requirement, not which token landing. User verbatim (2026-05-01): "We can claim that it is a precise touchdown only if its functional requirement is achieved. So it is correct to say achieve the functional requirement of the precise touchdown."

**Locked usages in `manuscript.tex` (re-locked 2026-05-02):**
- §I Para 2 (positive, IBVS): "These IBVS frameworks achieve the functional requirement of the precise touchdown through asymptotic image-feature convergence."
- §I Para 2 closing (**negative form** — paper-level gap statement MDF-ASMC fills): "None of these frameworks achieve the functional requirement of the soft touchdown."
- §I Para 1: the functional-requirement statement (S4, the *definition* of what must be achieved).
- Contribution (d) (~L114): "...achieving the functional requirements of soft-precise touchdown..."
- Para 7 / Conclusion (~L154): "MDF-ASMC achieves the functional requirements of both the soft-precise touchdown and target visibility without a range sensor or magnetometer..."

**Plural** ("functional requirementS") when crediting multiple properties simultaneously or listing the two Para-1 clauses ("achieves the functional requirements of soft-precise touchdown and target visibility"). **Singular** when crediting/denying a single property ("achieve the functional requirement of the precise touchdown").

**Why "achieve" (not "attempt") in the negative form:** "Attempt" is too strong — `\cite{lin2022}` *does* place a PPC envelope on velocity error (attempts bounded velocity, with the wrong simultaneity guarantee). "Achieve" is verifiable: §IV-D shows zero of the four baselines clears the soft criterion, regardless of design intent. Per the citation-verification discipline, sweeping "no prior work does X" claims must be checked vs each cite; "achieve" survives, "attempt" does not.

**Anti-patterns / what NOT to write:**
- "achieves precise landing" — too informal; doesn't anchor to the defining property.
- "delivers a soft touchdown" — "delivers" is too operational.
- "satisfies the precise criterion" — confuses functional requirement (qualitative) with operational threshold (numerical, §IV.A).
- "guarantees a precise touchdown" — only valid with a formal proof attached; "achieves the functional requirement" is the safer phrasing for empirical claims (e.g., backed by the 25/25 simulation).
- "None of these frameworks **attempt** to achieve the functional requirement of soft touchdown" — too strong; some PBVS papers (e.g., `lin2022`) attempt velocity bounding via PPC. Use "achieve".

## Related conventions
- `feedback_landing_marker_convention.md` — soft criterion is 3-D `‖v_rel‖`; never vertical-only.
- `feedback_preliminaries_methodology.md` — methodology in §II vs simulation in §IV.
- `project_termination_convention.md` — `z_f = 0.20 m` is the above-target gap (= landing-gear height), not absolute altitude.
- `feedback_soft_precise_hyphenation.md` — "soft-precise" hyphenated as a compound adjective.
- `feedback_no_overclaim_proven_properties.md` — never overclaim formal properties; "achieve the functional requirement" is the grounded form.
- `reference_optic_flow_soft_landing_principle.md` — never say "drives velocity to zero" / "constant optic flow".
