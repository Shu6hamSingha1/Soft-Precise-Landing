# Perception `s` vs GT `s` under GT-FB — findings (2026-09-23 → 09-25)

Question: with every other signal on Gazebo GT (GT-FB), can the image feature `s` (centroid xy)
come from PERCEPTION instead of the analytic GT? Per-rep numbers: `results.tsv` (256 reps). Raw
recordings (~7 GB) are local/gitignored; harness `scripts/run_sperc_gtfb_ab.sh`.

Thresholds: manuscript SP = xy ≤ 0.08 m AND rel_vel ≤ 0.2 m/s; "relaxed" = 0.15 m / 0.5 m/s.

## 1. Stationary target — the gap was the VALUE of perception s, not timing or gain
Cross-marker, IC1–5 × 5, GT-FB except `s`:

| arm | what changes | manuscript SP | xy mean |
|---|---|---|---|
| gt | full GT | 25/25 | 0.015 m |
| sperc (legacy detector) | s from perception | **6/25** | 0.059 m |
| gthold | GT s held at perception's live update times + latency | 25/25 | 0.015 m |
| gtbear | GT s as the TRUE bearing x/z instead of x/(z+0.2) | 25/25 | 0.009 m |
| sperc + stroke detector | s from new detector | **25/25** | 0.017 m (p=0.5 vs gt) |

* Staleness ruled out (gthold), the 1/z terminal gain ruled out (gtbear; the controller is fine with
  true-bearing s). Legacy perception s was accurate to ~0.5 m and wrong below (RMSE 0.036 → 0.078).
* Legacy live frame gap grows 16 → 96 ms below 1 m (detect() cost ∝ arm pixels) but this is harmless
  on a stationary target. A `.tolist()` speed-up of the detector helped offline (34 → 26 ms) and not
  live → reverted.

## 2. Moving target — legacy detector corner-locks the platform
rover_cross Sinusoidal / rover_cross_deck Circular, IC2:
* Legacy detector reads the raised platform's dark side face + shadow as a cross and returns the plate
  CORNER as the junction; the frozen s then runs the drone away (Sinusoidal xy median 2.5 m, 0/5).
* Stroke detector (below): Sinusoidal 0.17 m; no corner-locks.
* Deck-world GT marker height was 0.5 m; the deck_platform's marker is +0.201 m
  (`run_rover_landing.sh` default fixed) → every pre-09-25 GT-FB Linear/Circular flight (incl. Final/
  deck videos) aimed at a marker 0.3 m too high. Circular gt median 0.208 → 0.133 m after the fix.
  (An earlier claim here that the deck GT target was the rover was wrong.)

## 3. The stroke detector (`src/cross_stroke_detector.py`, now the default)
Multi-scale Hessian ridge strokes (polarity-agnostic, step edges cancelled), orientation Hough + refit,
X-junction confirm (bilateral support, ring topology at terminal range), Lab channel cascade, tracked
ROI + scale window + working-resolution. No absolute brightness gate, no mask-centroid proxy.
* Offline `test_data/PerceptionEvalSet` (23 tags, exact stamp pairing, true-bearing reference):
  confident-wrong 0–1.6 % (legacy 0.4–37 %), err median ~0.006, detOK 92–99 %.
* ⚠ Old `RobustnessFrameset/inv` is mis-paired; every prior `inv` number is invalid (use `rob_inv`).

## 4. Bounded loss handling (`PLASMC_S_LOSS_FADE`, default on)
s older than 0.2 s → fade position error s_e_n to 0 over 0.3 s at its source. Sinusoidal worst miss
1.276 → 0.211 m; stationary unchanged 25/25. Circular gap to GT is not loss-driven (fade active ~2 %).
Stroke stall guard: locked → no chroma cascade (it froze perception 0.3–0.4 s live).

## 5. Pure perception (no GT-FB): the soft-landing failure was the touchdown DETECTOR
IC1–5 n=1: stroke+fade precise (xy mean 0.030 vs 0.068) but 0/5 soft for BOTH configs (0.2–0.7 m/s).
Perception loom matched GT. The overfill touchdown path fires 4–12 cm above the ground (0.17–0.25 m vs
0.136 m resting contact) and landing_test disarmed on the spot → free fall; rel_vel was scored before
the fall. Fix `PLASMC_TD_SETTLE_S=1.0` (hold level until PX4 contact, bounded): stroke+fade 4/5 SP,
5/5 soft (speed mean 0.04 m/s); legacy 3/5 and drifted 0.28/0.58 m during settle.

## Defaults baked (all env-reversible)
`CROSS_DETECTOR=stroke`, `PLASMC_S_LOSS_FADE=1`, `PLASMC_TD_SETTLE_S=1.0`. One configuration for
stationary AND moving targets (nothing branches on target type).

## Open
* Moving-target gap to GT (Sinusoidal 0.14–0.17 vs 0.04 m; Circular 0.17–0.26 vs 0.13 m): live perception
  runs ~3× slower than offline (GIL/threads) → out-of-process perception is the next lever. Offline
  s-rate extrapolation helps > 1 m, hurts < 0.3 m (`CROSS_S_PREDICT`, off).
* Pure-perception gate is n=1 per IC; settle untested on moving targets in pure perception; ArUco path
  not re-gated (comparison-only).
* Linear deck: rover speed erratic (median 0.18–1.32 m/s per rep) — separate issue.
