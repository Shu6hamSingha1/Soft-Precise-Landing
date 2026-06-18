---
name: No vague "this" — always follow with a noun
description: In the DF-ASMC manuscript (and all associated tex), "this" must be followed by an explicit noun; bare demonstrative "this" without an antecedent noun is vague and must be fixed
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule:** Every use of *"this"* in manuscript, supplement, figure captions, remarks, theorem statements, and conclusions must be followed by an explicit noun that names what it refers to.

- **Bad (vague):**
  - "MDF-ASMC resolves this with two innovations..."
  - "This leads to..."
  - "This implies..."
  - "Because of this, the closed loop..."
- **Good (pointer + noun):**
  - "MDF-ASMC resolves this conflation with two innovations..."
  - "This two-funnel structure leads to..."
  - "This bound implies..."
  - "Because of this residual, the closed loop..."

**Why:** A bare "this" requires the reader to back-scan and infer the antecedent, which is especially costly in dense control-theory prose where multiple candidate antecedents (equations, objects, properties) sit in the preceding sentence. Naming the antecedent inline eliminates the inference and, often, ambiguity. Reviewers scanning a paragraph out-of-order lose the referent entirely unless it is tied down.

**How to apply:**
- Whenever writing or editing any tex file, never use "this" as a standalone pronoun. Always attach the noun it refers to: `this + <noun>`.
- When reviewing existing text, any `this <verb>` construction (e.g., "this is", "this implies", "this leads to") is a fix candidate — either insert the antecedent noun or recast the sentence.
- The same standard applies to other bare demonstratives: *these*, *that*, *those* — each should be followed by a noun in technical prose.
