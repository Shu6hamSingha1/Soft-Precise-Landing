---
name: project-thrust-sphere-bug-and-bake
description: The CBF deliverability bound was |I_a+g*e3|<=A_CAP, which bounds vehicle accel not thrust — it existed in TWO places, is fixed and BAKED default-on as CBF_SPHERE_TRUE_THRUST
metadata:
  type: project
---

**BAKED default-ON 2026-09-03** (`CBF_SPHERE_TRUE_THRUST`, commits `6b4fa02a` + `937db5a9`;
`=0` reverts to the legacy form).

## The bug

`I_a` is **thrust** acceleration in NED with hover at `I_a[2] = -g` (`controller.py`'s z-upright
guard, and the `B_T = m(I_a[2]+g)/cosφcosθ` mapping depends on exactly that). So
`I_a + g*e3` is the **VEHICLE's** acceleration — zero at hover — and the deliverability bound
`|I_a + g*e3| <= A_CAP` bounded nothing useful: at zero lateral it admitted
`I_a[2] ∈ [-23.41, +3.81]`, i.e. **2.39 g on a vehicle capable of 1.389 g**. A 2 g climb
(`|I_a|=19.60`) passed because its deviation from hover is only `g=9.80`.
The shift is not merely mis-signed — under the opposite reading (`I_a` as vehicle accel) the
thrust vector would be `I_a - g*e3`, so **no convention yields `+g`**.

## ⚠ It existed in TWO places — fixing one is not enough

1. `cbf_visibility.py`'s joint-QP sphere.
2. **`controller.py`'s downstream cap** — the LAST gate before the command is used.

With only #1 fixed, #2 still passed `I_a=[-2.85,11.08,-9.92]`, `|I_a|=15.14` (**111% of cap**),
because `|I_a+g*e3|=11.44` sailed under its threshold. **Root cause of the duplication:
`A_CAP`'s own defining comment specified the wrong quantity**, so both sites were coded
*correctly against a wrong spec*. Comment fixed; both sites now read ONE module-level flag
(`_TRUE_THRUST_SPHERE`) so a future edit cannot fix one and leave the other.

Swept for more: `cbf_visibility_aruco.py` does `del A_CAP, g` (no sphere) and the Hardware fork
has no deliverability cap — both clean. `controller.py:3381` / `Hardware:2709` use **minus** g
(`I_a = a_u - g*e3`), which is the definition establishing the convention, NOT a bug.

## Validation was an INVARIANT check, not a tuning result

This is the useful methodological point (user's framing, and it was the right one): the fix is a
*constraint-correctness* change, so its criterion is "does the command stay deliverable?" —
deterministic per rep, no statistics needed. Matched IC2-5 A/B, cross-marker
(`ICValidation/20260903-110221` legacy vs `-110618` fixed):

| | over-cap command samples | max \|I_a\| |
|---|---|---|
| legacy | **17** | 159.6% and 180.5% of cap |
| fixed | **0** | 81-97% of cap — never binds in normal flight |

**Mechanism** (from the earlier arm): baseline `|I_a|` sits ≤13.3 for the whole flight, then
escalates **1.4-5.7×** immediately after its first undeliverable command (IC5: 12.70 → 72.64 =
7.4 g). The breach is the *trigger*, not a symptom — the vehicle cannot follow it, tracking error
grows, κ ratchets. Clipping the first excursion prevents the runaway. **All breaches are terminal**
(last 1-7% of flight), and which ICs breach varies run to run — which is why the old bug was
intermittent and hard to attribute.

Landing outcomes corroborate but rest on **n=1/IC**: precise 0/4 → 4/4, xy 0.157-0.188 →
0.086-0.093. Treat as direction-of-effect + confirmed mechanism, not a measured improvement.

Offline harness: `tools/validate_thrust_sphere.py` (drives the REAL `cbf2_filter`; legacy emits
undeliverable commands in 31.3% of 4000 randomised cases, fixed in 0%).

## Related

`Control_Params.npy` now records the resolved config (`Config.overrides` + `Config.resolved`,
including `CBF_SPHERE_TRUE_THRUST`, `A_CAP`, `HD_KR`, `CBF_AZ_COST_GAIN`), so a rep is
self-describing — previously these were only visible in console banners that are not kept
per-rep. Still open PX4-side: `HD_KR=0.5` unswept, `CBF_AZ_COST_GAIN=5.0` picked-not-swept (and
now interacting with a sphere that actually binds), the joint QP's fixed 6×5 iteration with no
convergence test or residual logged, and no `ω_t` feedforward
([[project_20260901_rover_cross_perception_diagnosis]]).
