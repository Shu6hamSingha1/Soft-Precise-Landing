---
name: Theorem proof placement — main-paper skeleton + supplement full algebra
description: Locked 2026-04-26. Theorem 1 and Theorem 2 each carry a "load-bearing" proof in the main paper (Lyapunov candidate, key dV bound, UUB conclusion) with the line-by-line algebra deferred to supplement §S2. Block tags use `\textit{Proof of Theorem~N.}` (NOT "Sketch of Proof").
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The convention (locked 2026-04-26):**

Each theorem in the main paper carries an inline proof block immediately after its statement. The proof block contains exactly three load-bearing equations + their connecting prose:
1. **Lyapunov candidate** $V$ (with explicit equation label, e.g., `V candidate: equation`, `V alpha candidate: equation`)
2. **Key derivative bound** $\dot V \le -\varphi_1 V + \text{const}\cdot\tilde d^2$ (label e.g., `Vdot bound: equation`)
3. **UUB conclusion** — derives the stated UUB bound from the bound above via a citation to the GUUB lemma in §S2.

Block tag is **`\textit{Proof of Theorem~N.}`** — explicitly NOT "Sketch of Proof" (user explicitly rejected that wording). The closing $\hfill\blacksquare$ is included.

**What lives where:**
- **Main paper** (`control_formulation.tex` §III.C): proof block for T1 (after Theorem 1 statement, before Corollary 1) and T2 (after Theorem 2 statement). Each ~5–8 lines + 2 inline equations.
- **Supplement** (`supplemental.tex` §S2): "Full Proof of Theorem~N" with the line-by-line $\dot V$ expansion, regressor/disturbance definitions, completing-the-square steps, and the GUUB lemma + GUUB definition.

**Why:**
- Self-contained main-paper proofs satisfy IEEE TAES referees who want rigor visible in the paper.
- Full algebra in the supplement preserves transparency without bloating page count.
- Reviewer concerns about "guarantee" claims are anchored to specific equations in the main paper (e.g., `\eqref{V candidate: equation}`, `\eqref{Vdot bound: equation}`), which can then be cross-referenced from any defense letter.

**How to apply:**
- For any new theorem added to the main paper, follow the same three-step structure (candidate, bound, conclusion).
- Never use "Sketch" or "Outline" — the user rejected that framing in 2026-04-26 review pass.
- Cross-references between main and supplement use plain text (e.g., "Section~S2 of the supplement"), not `\eqref{}` which fails when the supplement compiles separately. See `feedback_supplement_cross_refs.md`.

**Related:** `reference_notation_audit.md` (theorem statements, UUB symbol naming, $\varphi_1, \varphi_2$ conventions).
