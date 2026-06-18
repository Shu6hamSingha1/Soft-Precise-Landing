---
name: manuscript-not-implementation-replica
description: "Locked 2026-06-11: manuscript math need not mirror how quantities are computed in MATLAB; prefer the readable frame-prefix form (^V a_d) over computation chains (^I R_V^T ^I a_d)"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 835634e5-9079-4b74-95af-775a75d60862
---

The manuscript does not need to be a replica of the implementation (user's words). When MATLAB computes a quantity through a chain (e.g., rotating an inertial vector: `I_R_V' * I_a_cd`), the paper should state the quantity in its natural frame using the leading-superscript convention ($\,^\mathcal{V}\boldsymbol{a}_\text{d}$), not reproduce the computation ($\,^\mathcal{I}R_\mathcal{V}^\top\,^\mathcal{I}\boldsymbol{a}_\text{d}$).

**Why:** readability and symbol economy — the rotation-matrix chains exist only because of how the code happens to store vectors; they add symbols ($^\mathcal{I}R_\mathcal{V}$, $^\mathcal{I}R_\mathcal{B}$ both eliminated this way) without adding meaning.

**How to apply:** in §III/§IV reviews, when an equation mirrors a code path rather than the cleanest mathematical statement, propose the frame-prefix/natural form. State the prefix convention once at first use ("expressed in the frame indicated by its leading superscript"). The same physical vector may appear with different prefixes ($^\mathcal{V}\boldsymbol{a}_\text{d}$ in the plant, $^\mathcal{I}\boldsymbol{a}_{\text{d},xy}$ in the cone clamp) — that is the convention working as intended. [[feedback_optical_flow_naming_epoch5]]
