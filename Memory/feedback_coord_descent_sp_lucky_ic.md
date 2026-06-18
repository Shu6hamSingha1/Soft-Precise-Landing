---
name: Coordinate-descent SP came from degenerate config + lucky IC (2026-05-25)
description: Per-axis random coord descent over 47 controller axes achieved SP at axis 32 via RHOFOV0×241.8. The "winning" config has THETACAP=2.75°, KP_Y near zero — degenerate, can't correct laterally. SP only because IC vh0 = 0.023 m/s (drone almost stationary). All 3 SPs ever observed had vh0 < 0.05 m/s.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## What happened

Per-axis random coordinate descent over 47 PLASMC controller knobs (each sampled log-uniform [0.0001, 10000]) achieved a SOFT+PRECISE landing on axis 32 of pass 1, via `RHOFOV0 × 241.8` acceptance.

- xy = 0.0521 m (under 0.08 PRECISE)
- vel = 0.0611 m/s (under 0.2 SOFT)
- **vh0 = 0.0227 m/s** at controller engagement — drone nearly stationary
- 3rd ever SP landing across the entire project

## The "winning" config is degenerate

7 axes were accepted before SP:
```
KP_Y       × 0.0005574  (Y-axis P-gain essentially zero)
RHOFOV0    × 241.8      (initial cone envelope ~70k px — disabled)
E_X        × 0.005938   (X boundary layer tiny)
LFOV       × 48.09      (very fast cone decay)
THETACAP   × 0.04592    (tilt cap = 2.75° — drone can barely tilt)
P2INF_X    × 0.04625    (terminal envelope tight on X)
E_Z        × 10.05      (wide Z boundary layer)
```

`THETACAP = 2.75°` is the smoking gun: the drone is physically capable of barely any lateral acceleration. This works at IC1 only if the drone is already near the target — which it was (vh0 = 0.023 m/s, vz0 = 0.008 m/s).

## Why the search converged here

The composite score `xy + 0.5·vel` rewards softness disproportionately at small xy. The coord descent kept accepting axes that disabled lateral correction (less correction → less velocity at touchdown). After 6 such acceptances, the drone was essentially in "vertical-only" mode. Then `RHOFOV0 × 241.8` on a lucky IC produced SP.

## Statistical pattern across all 3 SPs

| SP | Bundle | vh0 (m/s) | xy_end | vel_end |
|---|---|---|---|---|
| 1 | Interventions rep2 | 0.046 | 0.039 | 0.131 |
| 2 | NewDefaults rep5 | (small) | 0.026 | 0.148 |
| 3 | CoordDescent | **0.023** | 0.052 | 0.061 |

**All 3 SPs ever observed had vh0 < 0.05 m/s** — drone nearly stationary at engagement.  Across ~1300 reps in the project, no SP has occurred at vh0 > 0.06 m/s.

## What this confirms

The architectural-lag conclusion stands:
- SP at IC1 in this PX4 SITL is **IC-luck-driven**, not controller-tunable
- Random search can find SP by stumbling onto a lucky IC, but the "tuned" gains don't generalize
- Would catastrophically fail at IC2-5 (which have vh0 typically ≥ 0.5 m/s during their convergence to the offset IC)

## How to apply

- When user requests broad gain search: cite this finding.  Random or coord descent will eventually achieve SP at IC1, but the SP is from IC luck not gain quality.
- The "winning" coord-descent config is NOT generalizable — its degenerate axes disable lateral authority.  Don't merge to defaults.
- For real SP reproducibility, the architectural lag needs to drop (uXRCE-DDS / Smith predictor / different control architecture).
- For future searches: any composite score that rewards softness will collapse to the degenerate "low-authority" basin.  Consider a different metric (e.g., require both PREC AND SOFT simultaneously, not a composite).

## Data

- `~/ws/Test_Data/CoordDescent/20260525-201532/` — full bundle
- `state.json` — the 47-axis winning configuration
- `log.jsonl` — every axis test logged
- SP rep: `pass1/RHOFOV0_241.8_rep2/`
