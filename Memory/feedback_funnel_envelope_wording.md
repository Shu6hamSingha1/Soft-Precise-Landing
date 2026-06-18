---
name: Funnel wording — "rectangular envelope" (not "pixel-box"), image plane (not virtual image plane)
description: Target image funnel $\boldsymbol{p}_1(t)$ is a "rectangular envelope" defined on the "image plane" — do not use "pixel-box bound" or "virtual image plane"
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
When describing the target image funnel $\boldsymbol{p}_1(t)$ in prose / captions / abstract:

**Canonical phrasing:**
- "rectangular envelope" (not "pixel-box", not "pixel-box bound", not "box")
- Defined on the "image plane" (not "virtual image plane")
- Use "exponentially-shrinking" or "exponentially-decaying" as the shape modifier

**Correct:** "the target image funnel $\boldsymbol{p}_1(t)$ (exponentially-shrinking rectangular envelope) is defined on the image plane."

**Why:**
- §III-B.4 definition text already uses "rectangular envelope centred on the image centre and collapsing exponentially" — all other prose must match.
- Although the funnel is mathematically carried on the virtual image plane (since all features are expressed in $\mathcal{V}$), the reader-facing convention in this paper calls it simply "the image plane" to match the §III-B.4 definition: "pixel coordinates of the four target feature points on the image plane".
- "Pixel-box" is not a physics term used anywhere in the manuscript — never introduce it.

**How to apply:** Before touching any funnel-related prose, grep for the existing usage (`rectangular envelope`, `image plane`) and mirror those exact terms. Do not invent synonyms like "pixel-box bound" even if they feel more descriptive — internal consistency beats local vividness.
