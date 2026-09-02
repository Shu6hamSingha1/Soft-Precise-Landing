---
name: project_20260903_controller_population_analysis
description: "318-run controller-side population analysis of cross-marker rover failures. REFUTES kappa-decay and sigma as the mechanism (kappa decays FURTHER in successes: end/start 0.212 landed vs 0.890 failed; sigma max is HIGHER in failures). CBF is inactive on these failures. The discriminator is LATERAL DIVERGENCE (2% landed vs 66% failed), and 36% of runs at >=95% detOK still diverge -> a control-bound residual. Also: correct funnel identities (there is NO visibility funnel), and zeta_r is unloaded BY DESIGN."
metadata:
  node_type: memory
  type: project
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-03T00:00:00.000Z
---

**2026-09-03.** Ran across every cross-marker rover run with `Control_Data.npy` (318 runs,
85 landed / 233 failed, footprint-based landing test). Recorded mainly so the REFUTED
hypotheses are not re-derived — this analysis was itself a detour (see the framing note at
the end).

## ⛔ REFUTED: kappa decay and sigma are NOT the mechanism

| | LANDED (85) | FAILED (233) |
|---|---|---|
| kappa end/start | **0.212** | **0.890** |
| kappa decayed (end<start) | 75/85 | 187/233 |
| sigma median | 0.199 | 0.230 |
| sigma max | 0.597 | **1.140** |
| flow funnel ever binds (r_h>0.5) | 21 % | 40 % |
| combined-barrier funnel binds (r_r>0.5) | 1/85 | 6/233 |
| **lateral diverged** | **2 %** | **66 %** |

**kappa decays FURTHER in the runs that LAND** (to 21 % of its start value, vs 89 % in
failures) — the opposite of "the gain leaks away so authority is lost". A converging run
legitimately needs less gain. **sigma max is HIGHER in failures**, so sigma does respond to
error; it is not inert. Both funnels bind MORE often in failures, not less.

An n=1 trace had suggested the opposite on all three counts. It was a two-run coincidence.

## ⛔ The CBF is NOT involved in these failures

`dtheta_az ~ 0` and `az_joint_delta = 0` throughout, in both landed and failed traces. It is
not clipping, rotating, or folding accel into `I_a` here. (This does NOT contradict the
2026-08-26 IC5 finding — that was a different IC and a mechanism since removed; see the
dtheta obsolescence stamps.)

## The discriminator, and the split it hides

**Lateral divergence: 2 % of landed vs 66 % of failed.** But "diverged" != "failed":

| detOK band | n | diverged | landed |
|---|---|---|---|
| 0-25 % | 42 | **19 %** | 5 % |
| 25-50 % | 40 | 72 % | 18 % |
| 50-75 % | 38 | 74 % | 5 % |
| 75-95 % | 46 | 67 % | 11 % |
| **95-100 %** | **157** | **36 %** | **44 %** |

- **Low detOK runs diverge LEAST (19 %)** — they abort early (`TARGET_LOST`) and drop
  straight down, so lateral never has time to grow. They fail WITHOUT diverging.
  Perception-bound; this is the band the span rescue moves runs out of.
- **36 % of runs at >=95 % detOK still diverge**, and only 44 % land. **Control-bound
  residual** — restoring perception is necessary but NOT sufficient. Cause NOT identified.

## ⚠ Correct funnel identities (I got these wrong twice; user corrected both)

- **There is NO "visibility funnel."** Visibility is the **CBF** (`cbf2`). Do not call `p_r`
  a visibility funnel.
- **`p_r`** = *combined-barrier position funnel* on `s_e_n` (`controller.py:4126`), feeding
  `zeta_r` into the surface `sigma = zeta_h + chi_r*zeta_r`. **This is where lateral authority
  lives** — [[feedback_sen_authority_analysis]] records the design decision to move `zeta_r`
  INTO the surface.
- **`p_s`** = *outer-loop* funnel on `s_e_n` -> `V_ds_d_xy`. **SUPERSEDED** by the above. ⚠ But
  still LIVE: `PLASMC_SEN_FUNNEL` defaults to `"1"` (`:383`), `Control_Params` confirms
  `sen_funnel: True`, and `:2429-2454` computes `zeta_s`/`G_s`/`izeta_s` with its own PID gains
  and an active integrator. A live vestigial path worth a look.
- **`p`** = flow funnel on `h_e` (floor `P2INF` xy 2.5 after the 2026-08-28 rebake).

## ⚠ `zeta_r` being unloaded is BY DESIGN — not a defect

Measured `r_r` max ~0.098 median. That is the intended operating point, not a symptom.
`controller.py:453` documents it: `PR0=10.0` BAKED 2026-06-29 — *"A WIDE initial funnel keeps
`S_r=s_e_n/p_r` tiny (~0.05) the whole descent so `zeta_r` never enters the steep barrier edge
-> no edge-forcing, no terminal 1/Z balloon"*, and with `XIR=0.10` the funnel decays 10->~4 so
**`PRINF=0.8` is INERT — it never binds**. Validated current: only two commits ever touched
that line, last `787cf2d3` (2026-06-29). **So "is PRINF too loose?" is the wrong question.**

## ⛔ FRAMING NOTE — why this whole analysis was a detour

**GT-feedback ALREADY LANDS on a moving rover, 3/3** ([[project_moving_rover_landing_works]],
2026-07-02, Linear @0.47 m/s, rel lat 0.044-0.284 m, and it RECOVERS a 0.74 m transient
mid-descent). Same controller, same surface, same gains. **So this is not a control TUNING
problem, and the record said so before I started.** The user pointed this out; the correct
move was to read the recorded result first rather than re-derive it across 318 runs.

Also corrected: I claimed target SPEED explained moving-rover failures (landed median
0.56 m/s vs failed 0.74). That was a weak median split read as causal — and the recorded 3/3
landings are at 0.47 m/s with a controller that recovers transients. **Speed is not the
explanation.** A moving A/B at 0.85 m/s (n=5/arm) gave detOK 25->67 % with the span rescue and
**0/10 landings either arm**, which says only that restoring perception above the speed
ceiling is not sufficient.

**The live question is what the GT-FB signal has that a 95 %-detOK perception signal lacks —
latency, per-frame accuracy, or gap structure — not the controller's gains.**
