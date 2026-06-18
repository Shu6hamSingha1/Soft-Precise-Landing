---
name: Audit anchoring context when relocating or merging paragraphs
description: Two failure modes after structural edits. (a) Bare definite phrases ("the de-rotation", "the cone clamp", "the funnel margin") that lose their antecedent when paragraphs move forward. (b) Definitional sentences whose semantic load was carried by a now-retired heading (e.g., "Virtual Image Orientation" naming what α is). After every relocation OR merge, audit for both failure modes; add inline back-pointers for orphaned definites; add explicit definitional sentences for symbols whose semantics used to live in the heading.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** When relocating a Remark, paragraph, or sentence to a more distant location in the document (e.g., from §II.B to §III.A.3 end), audit every **bare definite phrase** ("the de-rotation", "the cone clamp", "the funnel margin", "the projection", "the linearity") for whether the reader at the new location still has the antecedent in working memory. A definite article is a *promise of shared context* — if the relocation breaks the proximity to the antecedent, the promise breaks too.

**Why.** A Remark sitting one paragraph after the §II.B intro can say "the de-rotation" cleanly, because the reader was just told three sentences ago about "rotating out the UAV roll $\phi$ and pitch $\theta$" to obtain the virtual frame. The same Remark at the end of §III.A — pages later — leaves the reader asking "*the* de-rotation? *which* de-rotation?" The phrase needs to be re-anchored.

**How to apply.**

1. **Before relocating, list bare definite phrases in the moved text.** Mark each as a phrase whose anchor must hold at the new location.
2. **At the new location, check whether each anchor is still nearby (≤1–2 paragraphs back).** If yes, no change needed. If no, add an inline back-pointer.
3. **A good back-pointer combines three elements:**
    - **Symbol** — the formal mathematical object (e.g., $\,^\mathcal{V}R_\mathcal{C}$) that is unambiguous.
    - **Operational description** — a short phrase that names what the object *does* (e.g., "from the camera frame $\mathcal{C}$ onto the virtual frame $\mathcal{V}$").
    - **Section reference** — a back-pointer to where it was first defined (e.g., `Section~\ref{background: section}-\ref{image parameters: section}`).
4. **All three are usually needed**, because each compensates for one limitation of the others: the symbol is unambiguous but cold; the description is warm but ambiguous; the section ref is explicit but distant.

## Failure mode A: orphaned bare definite phrases after relocation

### Example (where this rule was applied, 2026-05-05)

The Remark "The de-rotation removes the body-rate-coupled rotational terms..." was moved from end of §II.B to end of §III.A.

**Before relocation (worked at §II.B end):**
> *Remark.* The de-rotation removes the body-rate-coupled rotational terms from the image-feature kinematics, so the resulting kinematics are linear in the commanded acceleration. ...

**After relocation (broken at §III.A end):** "the de-rotation" lost its antecedent — the §II.B virtual-frame description was 5+ pages of derivation away.

**Fixed by re-anchoring:**
> *Remark.* The de-rotation $\,^\mathcal{V}R_\mathcal{C}$ from the camera frame $\mathcal{C}$ onto the virtual frame $\mathcal{V}$ (Section~II-B) removes the body-rate-coupled rotational terms ...

Three anchors added at the front: symbol ($\,^\mathcal{V}R_\mathcal{C}$), operational description ("from $\mathcal{C}$ onto $\mathcal{V}$"), and section ref (`Section~II-B`).

## Failure mode B: orphaned semantic anchors after a heading retirement

When merging two subsections or removing a subsubsection, the **subsection title itself** may have been carrying definitional load that the body assumed. The body says "$\alpha = \tfrac12\tan^{-1}(\ldots)$" without ever stating *what $\alpha$ is*, because the now-retired heading "Virtual Image Orientation" was naming the concept implicitly. After the merge, the symbol arrives at the reader with a formula but no semantics.

### Example (where this rule was applied, 2026-05-05)

Merging §II.B.1 (Virtual Image Position) + §II.B.3 (Virtual Image Orientation) into §II.B.1 (Virtual Image Pose) retired the heading `\subsubsection{Virtual Image Orientation}`. The body still computed $\alpha$ from image moments but no longer stated what $\alpha$ semantically is.

**Before merge (worked):**
> §II.B.3 *Virtual Image Orientation*  ← heading carries the semantics
> Extracted from the second-order centered image moments..., $\alpha = \tfrac12\tan^{-1}(\ldots)$.

**After merge (broken):**
> §II.B.1 *Virtual Image Pose*  ← heading no longer mentions orientation
> ... [position content] ...
> Extracted from the second-order centered image moments..., $\alpha = \tfrac12\tan^{-1}(\ldots)$.

**Fixed by adding an explicit definitional sentence:**
> The *virtual image orientation* $\alpha$ is extracted from the second-order centered image moments..., $\alpha = \tfrac12\tan^{-1}(\ldots)$.

### How to apply (failure mode B)

1. **For every subsection / subsubsection heading you retire**, read the body that used to live under it. Ask: did the body have a sentence of the form "*The X is …*" / "*X is defined as …*" / "*The Y …*"? If not, the heading was carrying semantic load.
2. **Add the missing definitional sentence** to the body, naming the concept the heading used to name. Use `\emph{...}` to mark the definitional term.
3. **Pair the sentence with parallel structure** to symmetric paragraphs in the merged subsection. (E.g., if §II.B.1's position paragraph says "*The homogeneous augmentation yields the virtual image position $\boldsymbol{s}$*", the orientation paragraph should mirror with "*The virtual image orientation $\alpha$ is extracted from …*".)

## Common bare-definite phrases to watch for

When auditing relocated MDF-ASMC prose, especially watch for:

- "the de-rotation" → anchor with $\,^\mathcal{V}R_\mathcal{C}$ + frame names + Section II-B
- "the funnel margin" → anchor with $d_\text{min}^\text{fov}$ + Section III-A3
- "the cone clamp" / "the funnel-margin cone clamp" → anchor with $\theta_\text{cone}$ + Section III-A3
- "the projection" → specify *which* projection (cone clamp? pinhole? Gram-Schmidt SO(3)?)
- "the linearity" → specify linearity *of what*, *in what*
- "the funnel" alone → ambiguous between target image funnel ($\boldsymbol{p}_1$) and optic-flow funnel ($\boldsymbol{p}_2$); always qualify

## Related conventions

- `feedback_no_vague_this.md` — bare "this" must be followed by an explicit noun; same spirit applied to bare definite articles after relocation.
- `feedback_section_ref_composition.md` — IEEE TAES `\ref{subsection}` returns only the letter; compose `\ref{section}-\ref{subsection}` for a proper section back-pointer.
- `feedback_kinematics_phrasing.md` — when the back-pointer references a kinematics equation, use the locked names ("image position kinematics" / "image orientation kinematics" / "optic-flow dynamics"), not improvised umbrella terms.
- `feedback_goal_first_when_mechanism_follows.md` — goal-first anchoring is reader-friendly; relocation-anchoring is the corollary (without it, even goal-first sentences float without context).
