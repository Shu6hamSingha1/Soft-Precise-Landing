---
name: Optic-flow soft-landing principle (bounded h, not constant or zero)
description: Reference for the optic-flow soft-landing argument used in the paper's introduction. The general principle is "bounded h drives ‖v_rel‖ → 0 as z → 0" — Hérissé/Izzo's "regulate to a constant" is one special case (1-D vertical); our funnel scheme is another (3-D). Locked 2026-04-30.
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The general principle (use this in §I):**

The optic flow $\boldsymbol{h} = \,^\mathcal{V}\boldsymbol{v}_{\text{t/b}}/\,^\mathcal{V}z_\text{t}$ satisfies
$$\|\boldsymbol{v}_\text{rel}\| = \|\boldsymbol{h}\| \cdot \,^\mathcal{V}z_\text{t}.$$
If $\|\boldsymbol{h}\|$ is **bounded** by some $M < \infty$ throughout descent, then
$$\|\boldsymbol{v}_\text{rel}\| \le M \cdot \,^\mathcal{V}z_\text{t} \;\;\xrightarrow{\;\,^\mathcal{V}z_\text{t} \to 0\;}\;\; 0.$$

This is the *general* statement. **Boundedness** is sufficient — neither *constancy* nor *zero* is required.

**Two special cases — both are "bounded":**

| Scheme | What's bounded | Application |
|---|---|---|
| Hérissé 2012 | $\boldsymbol{h}$ regulated to a **constant** | 1-D vertical landing on a **vertically-moving** platform (theoretical certificate); lateral motion shown in practice only. Separate hovering task handles 3-D moving platform but is not a landing certificate. |
| Izzo 2011 | $\boldsymbol{h}$ regulated to a **constant** | 1-D vertical descent on stationary lunar surface (theoretical + experimental scope match) |
| Singhal 2025 | $\boldsymbol{h}$ regulated to a **constant** | 1-D vertical landing on stationary surface (theoretical certificate); lateral stabilization shown experimentally only on mini-quadrotor in unstructured settings. |
| VDF-ASMC (our paper) | $\boldsymbol{h}_\text{e} = \boldsymbol{h} - \boldsymbol{h}_\text{d}$ confined inside the **time-varying funnel** $\boldsymbol{p}_2(t)$ (Theorem 1); $\boldsymbol{h}_\text{d}$ itself bounded by design | 3-D image-kinematic regulation on a maneuvering target |

**Theoretical-certificate vs experimental-demonstration distinction (added 2026-05-14):**

When characterizing the optic-flow prior art in §I:
- **Theoretical certificate** = what the Lyapunov / stability proof actually covers (Hérissé landing: 1-D vertical; Singhal: 1-D vertical on stationary; Izzo: 1-D vertical on lunar).
- **Experimental demonstration** = what the paper *shows in hardware/sim* without a matching proof (Hérissé and Singhal both demonstrate lateral capability in practice without certifying it).

In critiques, scope the failure mode to the **certificate**, not to the paper-as-a-whole, otherwise the claim is undercut by the experimental section. Verified phrasing: *"None of these designs certifies soft landing on a moving target with lateral motion"* (manuscript L114, locked 2026-05-14).

**Phrases that work for both special cases (use these in §I):**
- "keeping it bounded drives the relative velocity to zero as altitude vanishes" ✅ — current §I wording.
- "bounding the optic flow drives `\|v_rel\| → 0` as the UAV approaches the target" ✅
- "a bounded-optic-flow regulator yields a vanishing relative velocity at touchdown" ✅

**Phrases that overspecialise to one case:**
- "regulating it to a constant" — only the 1-D Hérissé/Izzo case. The 3-D `\boldsymbol{h}_d` of our paper is *not* a constant (it has time-varying lateral terms; only the radial component `h_rd` is constant).
- "forcing it to zero" — neither case. Forcing `h = 0` would make `v_rel = 0` instantly, preventing descent. Hérissé/Izzo regulate to a non-zero constant; our `h_d` has `h_rd > 0` for descent.
- "regulating the descent velocity" — the descent component is one axis; we regulate the full 3-D vector.

**Where this came up (2026-04-30):**
- A draft of §I said "forcing it to zero drives the descent velocity to zero as altitude vanishes". User flagged two issues: (i) "descent velocity" is 1-D vertical, but the paper uses 3-D `\|v_rel\|`; (ii) when I suggested "regulating it to a constant" as a fix, the user pointed out that constancy applies only to the 1-D vertical case, not to the 3-D scheme MDF-ASMC actually uses. The phrasing "keeping it bounded" was chosen as the only formulation that's accurate for both.

**How to apply:**
- In §I, when motivating the optic-flow approach, use "bounded" or "kept bounded".
- In §III when describing what MDF-ASMC actually does, refer to the funnel `p_2(t)` (which is the *specific* form of boundedness used here).
- Never write that MDF-ASMC "regulates optic flow to a constant" — it doesn't.
- Never write that MDF-ASMC "forces optic flow to zero" — it doesn't, and doing so would prevent descent.

**Related conventions:**
- `feedback_funnel_naming.md` — target image funnel = `p_1`, optic-flow funnel = `p_2`.
- `feedback_no_overclaim_proven_properties.md` — what's empirical vs theorem.
- `feedback_validate_critiques_against_cited_works.md` — Hérissé/Singhal regulate optic flow; "no prior work regulates optic flow" is false.
- `feedback_landing_marker_convention.md` — 3-D `‖v_rel‖`, not vertical-only.
