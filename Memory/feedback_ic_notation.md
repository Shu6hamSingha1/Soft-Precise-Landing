---
name: IC notation convention — IC$_N$ for indexed, bare "IC" for generic
description: Indexed initial-condition references use IC$_N$ (subscript in math mode, $N \in \{1,\ldots,5\}$); bare "IC" stays as plain text for generic references. Never use IC$N$ (non-subscripted math), $\text{IC}_N$, or IC $[\ldots]$ (coordinates without index). Locked 2026-04-25.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The rule (locked 2026-04-25):**

Two forms are allowed, one each for different contexts:

1. **Indexed IC reference** (pointing at a specific initial condition from Table III):
   - Form: `IC$_N$` where $N \in \{1, 2, 3, 4, 5\}$
   - Renders as IC₁, IC₂, IC₃, IC₄, IC₅
   - With coordinates: `IC$_N=[\ldots]$~m` — e.g., `IC$_2=[2,2,-5]$~m`
   - With math continuation (approx, dash): `IC$_N\approx$`, `IC$_N$--IC$_M$`

2. **Bare / generic IC reference** (not pointing at a specific indexed condition):
   - Form: plain text `IC`
   - Example uses: "per-IC seeding", "Per-IC Performance Analysis", "no IC produces a visibility violation", Table III column header `\textbf{IC}`, phrases like "the IC stresses…"

**Banned forms** (do NOT use):
- `IC$N$` (without underscore) — non-subscripted; previously ubiquitous but hard-to-scan
- `$\text{IC}_N$` — verbose; use `IC$_N$` instead (same rendering, fewer characters)
- `IC $[\ldots]$` (coordinates floating after bare IC with no index) — ambiguous; replace with `IC$_N=[\ldots]$` for whichever N matches the coords
- `IC N` (space-separated) — use math subscript instead

**Canonical IC coordinates** (Table III):
- IC$_1 = [0, 0, -5]$ m (nominal drop)
- IC$_2 = [2, 2, -5]$ m (diagonal lateral offset — the adversarial IC used in §IV-C comparison)
- IC$_3 = [2, -2, -5]$ m (anti-diagonal lateral offset)
- IC$_4 = [2, 2, -7]$ m (high altitude + lateral)
- IC$_5 = [2, 2, -3]$ m (low altitude + lateral)

**Why:** User flagged 2026-04-25 that the paper used three different forms (`IC`, `IC2`, `IC_2`) non-uniformly. Subscript form reads better in IEEE-TAES prose, aligns with how per-coordinate quantities are subscripted elsewhere ($p_{1_k}$, $\zeta_{2_k}$), and disambiguates from a multiplication "IC × N".

**How to apply:**
- When writing a new IC reference, use `IC$_N$` for indexed, bare `IC` for generic.
- When editing existing prose, rewrite any `IC$N$`, `\text{IC}_N`, or bare-coords form to `IC$_N$`.
- If adding coordinates that match a Table III row, link them to the index: `IC$_1=[0,0,-5]$~m` not `IC $[0,0,-5]$~m`.
- Figure captions, table captions, table cells, supplement prose — all follow the same rule.

**Verification grep command:**
```bash
grep -nE 'IC\$[0-9]|\\text\{IC\}|IC \$\[' Soft_Precise_Landing/{manuscript,control_formulation,results,supplemental}.tex
```
Any hit is a violation; fix before commit.
