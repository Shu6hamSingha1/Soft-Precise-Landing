---
name: project_20260923_comparative_baseline_campaign
description: First full 4-baseline x 10-case GT-FB comparative recording campaign (lin2022/zhang2026/lin2023/cho2022) under test_data/Final/<TAG>-GT/ -- results table + the cho2022 0/10 descent-stall finding.
metadata:
  node_type: memory
  type: project
  modified: 2026-09-23T01:43:38.624Z
  originSessionId: d4885eeb-2b11-4480-8a1e-f87fe4c8ec53
---

2026-09-23: ran the manuscript's 4 comparison baselines (`src/baselines.py`:
`lin2022`, `zhang2026`, `lin2023`, `cho2022` -- PLASMC_BASELINE env var, requires
PLASMC_GT_FEEDBACK=1) through the SAME 10 cases as [[project_20260923_manuscript_video_symbol_convention]]'s
VISTA-GT set (IC1-5 + Static/Linear/Sinusoidal/Circular/Lissajous), ONE attempt each
(no SoftPrecise retry gate -- baselines aren't required to land precisely, explicit
user direction). New tooling:
- `scripts/record_baseline_cases.sh <baseline> <case>` -- single-attempt recorder,
  mirrors `scripts/record_gtfb_cases.sh`'s env recipe but adds the IC1-5 stationary
  cases (`INITIAL_DRONE_ENU` per IC) and drops the SP retry loop.
- `scripts/run_baseline_campaign.sh` -- drives all 4x10=40 combos sequentially,
  promoting each rep into `test_data/Final/<TAG>-GT/<case>/` (dataset + onboard_cam +
  chase_cam + a montage via `tools/make_landing_montage.py`, raw-onboard PiP only, no
  s/alpha or h/w overlay PiPs -- that scope decision predated finding
  `tools/overlay_image_features.py` is reusable; not re-litigated) IMMEDIATELY after
  each run, so progress survives a mid-campaign interruption.
- Tag folders: `LIN2022-GT`, `ZHANG2026-GT`, `LIN2023-GT`, `CHO2022-GT` under
  `test_data/Final/`, parallel to `VISTA-GT`.

**Pre-flight gotcha:** a concurrent session was actively running SITL when this was
first attempted -- collided on port 8888 (MicroXRCEAgent bind error). Waited/polled
until clear before launching; see [[feedback_recurring_analysis_mistakes]]'s
concurrent-session caution. General rule: `ps aux | grep -iE "microxrce|gz sim|rover_drive"`
before ANY SITL launch, not just at session start.

## Results (40/40 combos recorded; landing NOT required/gated)

| Baseline | Landed | Notes |
|---|---|---|
| lin2022 | 6/10 | IC1 xy=0.209m, IC4 xy=0.372m, Linear xy=1.491m, Sinusoidal xy=0.248m, Circular xy=1.230m, Lissajous xy=0.726m -- none precise/soft. IC2/IC3/IC5/Static: descent-stall abort (`RuntimeError: descent stall: no >0.30m descent in 25s`) |
| zhang2026 | 10/10 | Landed everywhere but very imprecise/hard: xy 2.65-12.20m, rel_vel up to 18.5 m/s |
| lin2023 | 5/10 | IC1 xy=5.197m, Linear xy=3.474m, Sinusoidal xy=0.533m, Circular xy=0.992m, Lissajous xy=11.252m. IC2-5/Static: descent-stall abort |
| **cho2022** | **0/10** | EVERY case hits the identical `descent stall: no >0.30 m descent in 25s -- hovering, aborting`. Video+dataset still recorded/promoted for all 10 (shows the hover as a negative result) |

**cho2022 finding, unverified root cause:** this is a consistent, reproducible failure
(same error message, all 10/10 cases) -- looks like a real property of the FF-IBVS
baseline under this SITL/GT-FB harness (never initiates descent at all), not run-to-run
noise like the other baselines' scattered stalls. Worth investigating
`src/baselines.py`'s `Cho2022` class / `controller.py::_baselineStep`'s handling of it
specifically if this needs explaining for the manuscript -- NOT yet root-caused in this
session, just recorded as a finding.

No MANIFEST.md written yet for the 4 baseline tag folders (VISTA-GT has one; these
don't) -- do that before citing these results anywhere external.
