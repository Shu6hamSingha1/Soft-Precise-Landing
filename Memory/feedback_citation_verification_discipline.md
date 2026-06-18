---
name: feedback_citation_verification_discipline
description: Citation-verification discipline — PDF-verify every cite before claiming (7-mistake list + corrective checklist); scope sweeping critiques to each cited work (Hérissé/Singhal DO regulate optic flow; certificate-vs-demonstration)
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

Consolidated citation-verification discipline. Merges two feedback memories: the process-level "verify before claiming" guard (7 mistakes + checklist) and the "validate sweeping critiques against the cited literature" rule (scoping table + certificate-vs-demonstration). Separate from `feedback_citation_classification_audit.md` (the per-paper classification table — kept distinct, do not merge).

## Part A — Verify cited papers against their PDFs before writing (locked 2026-05-02)

**The failure mode:** asserting a claim about a cited paper without verifying it against the paper's actual content. Across the §I editing pass (2026-05-02) I made seven citation-related mistakes sharing this single pattern. The user called the first ("cho2022 under PBVS") a *"serious lapse"* and asked for the full §I to be re-audited; the PDF audit then surfaced six more. The mistakes propagated through the memory system because earlier inferred classifications were trusted as ground truth in subsequent edits.

### The seven mistakes
| # | Mistake | Sub-pattern |
|---|---|---|
| 1 | `cho2022` cited under PBVS sentence | Trusted bibliography key + author memory; never checked abstract |
| 2 | `salehi2021` cited under PBVS sentence | Trusted *my own memory file's* PBVS classification — itself an unverified inference |
| 3 | PBVS sensor list as "LiDAR, stereo cameras, or GPS" | Treated PBVS sensors as empirical instead of *definitional* (PBVS = position-based **visual** servoing → vision-based by definition) |
| 4 | `herisse2012` lumped under "stationary targets" generalization | Inferred from a sweeping "typically address" hedge; didn't verify Hérissé explicitly handles moving platforms |
| 5 | "IMU-aided attitude" critique against `herisse2012` | Failed *self-consistency*: MDF-ASMC also uses IMU for body rates, so the critique self-incriminates |
| 6 | `bouazza2025` grouped with adaptive control schemes | Lumped all "recent autonomous-landing" papers; bouazza2025 is a state-estimation pipeline (no switching gain, no aerodynamic model) |
| 7 | "PPC forces the image error" cited to `lin2022` | `lin2022` is PBVS-PPC on position error, not IBVS-PPC on image error |

**Single root cause — assertion before verification.** Each variant: claim derived from *paper category* without checking the paper; from the *paper title* without reading the abstract; trusted from *my own memory file* (transitive trust failure); derived from a *sweeping generalization* without per-cite verification; or a claim that fails *self-consistency*.

### Corrective checklist (apply before writing any literature paragraph)
When about to write a sentence of the form *"X papers do Y \cite{...}"*:
1. **PDF-verify each cite.** For every `\cite{paper}` in a class-asserting sentence, open the PDF (`pdf-extract` skill) and read the abstract. Confirm the paper actually fits the asserted class label. The verification is cheap; the cost of mistakes cascades through every subsequent edit and into memory.
2. **Test definitional claims at first principles.** Ask: is this definitional ("visual servoing uses vision" — yes by definition) or empirical (what specific papers do)? If the definitional answer is wrong, the empirical claim is also wrong — do NOT make it. Worked example: "PBVS uses LiDAR" — the V in PBVS is Visual → wrong regardless of what individual papers do.
3. **Self-consistency check.** For every critique of a cited work, ask "does MDF-ASMC also do this?" If yes, the critique is self-incriminating — drop or rephrase so the actual differentiator surfaces. Worked example: critiquing Hérissé for "IMU-aided attitude" when MDF-ASMC also uses IMU; the actual differentiator is the *external heading reference / magnetometer*, not IMU per se.
4. **Per-cite check on sweeping claims.** "all of [list] do Y" requires verifying each cite individually against Y; the first cite that fails Y kills or rephrases the claim. Worked example: claiming `bouazza2025, paris2020, chen2025, kamath2026, zhang2026` all "presuppose an a priori upper bound on switching gain" — bouazza2025 is a state estimator with no switching gain → split it out, narrow the critique to the four control schemes.
5. **Distrust own memory until PDF-anchored.** A memory entry like "X = PBVS" without an explicit "✓ verified from abstract" stamp is an unverified inference, not a fact. Re-verify before using as ground truth in new prose; add a verification stamp after PDF-checking.
6. **Audit before commit, not after.** After writing a literature paragraph, run a citation-classification audit (`prose-audit` skill, Rule 14) **before** asking the user to review. The user shouldn't need to be the audit.
7. **Watch for inferred-into-fact compounding.** Successive edits compound earlier inferences into seemingly-grounded claims. When restructuring an existing literature paragraph, re-verify the cite list — don't just rearrange sentences while trusting the existing framing.

User feedback (2026-05-02), after the seven mistakes: "I need you to go through the history and understand why have you made these mistakes? Learn to avoid such mistakes." — a process-level lesson, not just per-mistake corrections.

## Part B — Validate sweeping critiques against the cited literature (locked 2026-04-30)

Critiques in §I that claim broad missing capability ("rarely enforces", "does not address", "leaves X unconstrained", "no prior work does Y") must be **scoped to the specific cited works** they're meant to indict, and verified against each before being committed.

**Two foundational references that contradict naïve claims about the optic-flow line:**
1. **Hérissé 2012 `\cite{herisse2012}`** — explicitly regulates optic flow for soft landing. Cited in §II.B of *our* paper as the source of the optic-flow formulation we use. Any sentence saying "no prior work regulates optic flow" or "no prior work addresses soft touchdown" is **false** against this paper.
2. **Singhal 2025 `\cite{singhal2025}`** — your prior PLASMC work, explicitly soft-landing-focused, regulates optic flow. Cited at the bottom of the comparison table and tagged "Novel vs.~\cite{singhal2025}". Same caveat as Hérissé.

**Two correct ways to scope a critique:**
| Wrong | Right |
|---|---|
| "no prior work imposes a transient envelope on the relative velocity" | "no prior work imposes a *prescribed-performance funnel* on the optic-flow error" (Hérissé/Singhal regulate to a constant or to a setpoint, not via a transient funnel) |
| "the literature leaves optic flow unconstrained" | "the cited IBVS extensions [jabbari2014, lee2012, fink2017, xie2016_2, xie2020] regulate image-position features only" (named subset, doesn't contradict the optic-flow line) |
| "PPC has no mechanism for soft touchdown" | "PPC has so far been applied to the image-plane error alone" (true for [lin2022], doesn't claim against the entire literature) |

**Audit checklist before submitting an introduction critique:**
1. Identify which subset of the cited works the critique applies to. Name them with citation keys, not "the literature".
2. Cross-check each cited paper. If our paper itself uses one of the cited approaches (e.g., Hérissé's optic-flow definition in §II.B), the critique cannot say "no prior work does X" if X is what that approach does.
3. The novelty claim should be about **what our combination adds**, not what nobody has done before in any form. Frame as "we add a prescribed-performance funnel on optic-flow error to a system that previously regulated optic flow asymptotically."

**Where this came up (2026-04-30):** a draft critique read "leaving the optic flow that governs touchdown speed unconstrained". The user asked: is this true for the cited works? Answer: false against Hérissé/Singhal, true only against the IBVS-aerial subset. The line was rescoped to "regulate image-position features without imposing a prescribed envelope on the optic-flow error that governs touchdown speed" (Option C′) — accurate against both the IBVS subset and the broader literature.

**Theoretical-certificate vs experimental-demonstration (added 2026-05-14):** a cited paper's abstract often promises capability that the **theoretical certificate** does not actually cover. Distinguish:
| Source | What it shows |
|---|---|
| Lyapunov / stability theorem | The **certificate** — the formal scope of the proven property |
| Hardware experiment / simulation section | The **demonstration** — what works in practice, possibly outside the certificate |

Examples found in this paper's §I revision (2026-05-14): Hérissé 2012 landing certificate is 1-D vertical (target moves only vertically), lateral motion *"in practice"* only; Singhal 2025 landing certificate is 1-D vertical on a stationary surface, lateral stabilization shown only experimentally on a mini-quadrotor. **Rule:** scope critiques to the certificate, not the paper-as-a-whole. "None of these designs handles lateral motion" is undercut by Hérissé's hovering task and Singhal's experimental demo; "None of these designs *certifies* soft landing on a moving target with lateral motion" survives both. Use "certifies" / "the certificate covers" when the failure mode is theoretical, not practical.

## Related conventions
- `feedback_citation_classification_audit.md` — per-paper verified classification table; the *output* of running the verification process (kept separate from this discipline file).
- `prose-audit` skill Rule 14 — mechanical citation-classification detection patterns.
- `feedback_no_overclaim_proven_properties.md` — don't claim formal properties we don't prove.
- `feedback_no_legacy_comparisons.md` — don't reference superseded internal variants.
- `feedback_no_vague_this.md` — explicit subjects, not vague pronouns.
