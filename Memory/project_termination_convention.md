---
name: Termination uses above-target gap, not absolute altitude
description: comparison harness fires touchdown on abs(UAV_z - target_z) <= zf; manuscript now defines z_f as "UAV altitude above the target"; ship-deck heave on Cases 2/5 makes absolute altitude ≠ above-target gap
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Fact (2026-04-20):** The MATLAB comparison harness (`visualControl_comparison.m` lines 383, 812) and Multi_init harness both terminate runs on `alt_above = abs(I_p_c(3) - x_t(3,idx)); if alt_above <= zf`. `alt_above` is the UAV altitude ABOVE THE TARGET, not absolute altitude above ground.

**Physical interpretation of `z_f = 0.20 m` (added 2026-05-01):** `z_f` corresponds to the **landing-gear height** of the UAV — the vertical distance from the airframe (where the camera and rigid body are co-located) to the bottom of the landing gear. When the above-target gap reaches `z_f`, the landing gear physically contacts the target surface — that is the moment of touchdown.

**Why this matters:** On Cases 2 (Linear) and 5 (Circular), Lin-2022's ship-deck heave oscillates the target z by $\pm 0.2$ m. At touchdown the UAV's absolute altitude can be as high as ~0.40 m while the above-target gap is exactly 0.20 m. Reading Table IV's `z_f` column as absolute altitude would be wrong on these two cases.

**Tex convention locked (2026-04-20):**
- `results.tex:4` defines: "A run terminates when the UAV altitude above the target first falls below $z_\text{f}=0.20$ m."
- `results.tex:21` parameter table: `z_f` labelled "Above-target gap" (not "Landing altitude") — short label for single-column width.
- Table IV `z_f` column values ARE the above-target gap at the terminating step; Proposed = 0.200 on every row reflects the threshold crossing, not absolute altitude.

**How to apply:**
- When auditing prose that mentions "landing altitude" or "$z_\text{f}$", verify the reader can resolve it to "above-target gap" via the `results.tex:4` definition. Captions/prose that just say "$z_\text{f}=0.20$ m landing altitude" are fine — they inherit the definition.
- When comparing across controllers, baselines stalling at $z_\text{f}\approx 1.08$ m (Chen) means 1.08 m above the target, NOT 1.08 m absolute. Keep this in mind when reasoning about "how much further Chen has to go".
- If you see an `.mat` file's X_DS(3,idx) at ~0.38 m on Linear/Circular Proposed, do NOT flag it as a data bug — it is consistent with above-target gap = 0.20 m plus heave offset.

**Relation to `project_indexing_convention.md`:** That memory covers `idx-1` vs `idx` (1-step drift); this memory covers absolute vs above-target (heave-dependent drift of up to 0.20 m on z readings). Both apply when syncing tex from `.mat`.
