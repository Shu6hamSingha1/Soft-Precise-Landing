# test_data/Final/LIN2022-GT — comparison baseline: Lin et al. 2022 (PBVS+PPC)

Code: main @ f8bcaf42 (recorded), 310c50c5 (montage fix applied). `PLASMC_GT_FEEDBACK=1
PLASMC_BASELINE=lin2022` (`src/baselines.py::Lin2022`, PBVS with prescribed-performance
control, fed ground-truth pose+velocity directly per the MATLAB comparison harness
convention), `WORLD=rover_cross[_deck] ROVER_MODEL=rover_cross[_ackermann]`,
`MARKER_TYPE=cross`, default params, ONE attempt per case (no SoftPrecise retry gate —
comparison baselines are not required to land precisely; see
`scripts/run_baseline_campaign.sh`).

Each `<case>/` holds `<case>_montage.mp4` (chase + onboard PiP + GT plot panel),
`<case>_onboard_cam.mp4`, `<case>_chase_cam.mp4`, and `dataset/` (Control_Data,
Control_Params, Ground_Truth, Img_Data, Img_Params, Telemetry_Data). No
`overlay_s_alpha`/`overlay_h` feature-overlay PiPs for baselines (scope decision
2026-09-23, predates finding `tools/overlay_image_features.py` reusable — not
re-litigated).

| Case | Outcome | xy_err (m) | rel_vel (m/s) | precise | soft | dur (s) | source run |
|------|---------|-----------|---------------|---------|------|---------|------------|
| IC1 | Landed | 0.209 | 3.320 | False | False | 17.2 | `Wed Sep 23 04-38-32 2026` |
| IC2 | **Aborted** (descent stall) | — | — | — | — | 34.2 | `Wed Sep 23 04-41-26 2026` |
| IC3 | **Aborted** (descent stall) | — | — | — | — | 34.2 | `Wed Sep 23 23-10-08 2026` (re-recorded) |
| IC4 | Landed | 0.372 | 1.990 | False | False | 25.4 | `Wed Sep 23 04-47-34 2026` |
| IC5 | **Aborted** (descent stall) | — | — | — | — | 32.7 | `Wed Sep 23 04-50-29 2026` |
| Static | **Aborted** (descent stall) | — | — | — | — | 34.1 | `Wed Sep 23 04-54-35 2026` |
| Linear | Landed | 1.491 | 1.544 | False | False | 5.0 | `Wed Sep 23 04-58-08 2026` |
| Sinusoidal | Landed | 0.248 | 2.085 | False | False | 13.7 | `Wed Sep 23 04-59-57 2026` |
| Circular | Landed | 1.230 | 9.520 | False | False | 13.6 | `Wed Sep 23 05-02-13 2026` |
| Lissajous | Landed | 0.726 | 4.208 | False | False | 20.0 | `Wed Sep 23 05-04-46 2026` |

**Result: 6/10 landed, 0/10 precise+soft.** "Aborted" = `RuntimeError: descent stall:
no >0.30 m descent in 25s — hovering, aborting` (the harness's own descent-stall
watchdog, not a crash). Dataset/video are still recorded/promoted for aborted cases —
the flight just stops progressing rather than terminating abnormally.

Notes:
- **`IC3` was RE-RECORDED 2026-09-23 (night)** because the original campaign rep never
  saved an onboard recording (so it had no montage). The new single attempt hit the same
  descent-stall abort, but this time onboard + chase videos were captured; the whole `IC3/`
  folder (dataset + videos) is now from that one rep. See
  `Memory/px4/feedback_montage_touchdown_argmin_bug.md` for the original gap.
- Montages for the aborted cases (IC2/IC3/IC5/Static) were regenerated 2026-09-23
  after a touchdown-detection bug fix in `tools/make_landing_montage.py` — the original
  versions truncated both the plots and the video itself early because they mistook a
  hover-noise altitude dip for touchdown. See
  `Memory/px4/feedback_montage_touchdown_argmin_bug.md` for the full root cause.
- All 10 cases use the same GT-FB rover-world cross-marker recipe as `VISTA-GT/`
  (`../VISTA-GT/`), so `precise`/`soft` are directly comparable against that set's
  numbers at the same 0.10 m / 0.2 m/s harness thresholds.

**Platform-landing audit (2026-09-23, added after checking the geometry):** the landing deck is
a 0.6 x 0.6 m box (`rover_cross/model.sdf`, top at z=+0.50 m), so a centre >~0.3 m from the
target with `alt_above_surface_end` well below 0 is a *ground impact beside the platform*, not
a deck landing. The harness's `landed` flag (PX4 LandedState / accelerometer spike) does not
distinguish these. Read "landed" in the table above as "reached a physical impact", not "landed
on the target". This baseline: 2 on the deck (IC1 xy 0.21 m, Sinusoidal 0.25 m); 2 borderline/edge (IC4 xy 0.37 m at deck height; Lissajous 0.73 m); 2 ground impacts beside the platform (Linear 1.49 m, Circular 1.23 m, both ~0.6 m below deck level); 4 aborted.

**Run-to-run variance check, all 4 baselines (2026-09-24):** IC1 and Lissajous re-run 3 more
times each per baseline, same code, nothing changed between runs.

| Baseline | Case | Campaign ; rep 1 ; rep 2 ; rep 3 (xy / rel_vel m/s; "(deck)" = on the 0.6 m deck) | Landed | On deck | xy range | Precise+soft |
|---|---|---|---|---|---|---|
| lin2022 | IC1 | 0.21 m / 3.32 (deck) ; 0.67 m / 1.46 ; stall abort ; 0.68 m / 4.53 | 3/4 | 1/4 | 0.21-0.68 m | 0/4 |
| lin2022 | Lissajous | 0.73 m / 4.21 ; 0.85 m / 4.17 ; 0.12 m / 1.54 (deck) ; 0.25 m / 1.99 (deck) | 4/4 | 2/4 | 0.12-0.85 m | 0/4 |
| zhang2026 | IC1 | 2.65 m / 4.28 ; 0.45 m / 1.90 ; 0.14 m / 0.31 (deck) ; 0.44 m / 0.51 | 4/4 | 1/4 | 0.14-2.65 m | 0/4 |
| zhang2026 | Lissajous | 12.19 m / 14.98 ; 10.64 m / 12.33 ; 11.17 m / 13.41 ; 10.02 m / 11.73 | 4/4 | 0/4 | 10.0-12.2 m | 0/4 |
| cho2022 | IC1 | stall abort x4 | 0/4 | 0/4 | — | 0/4 |
| cho2022 | Lissajous | stall abort x4 | 0/4 | 0/4 | — | 0/4 |
| lin2023 | IC1 | 0.44 m / 1.39 ; 0.28 m / 1.60 (deck) ; 5.11 m / 18.01 ; 0.18 m / 2.04 (deck) | 4/4 | 2/4 | 0.18-5.11 m | 0/4 |
| lin2023 | Lissajous | 0.93 m / 2.45 ; 0.28 m / 0.36 (deck) ; stall abort ; stall abort | 2/4 | 1/4 | 0.28-0.93 m | 0/4 |

**How far to trust a single campaign recording depends on the controller AND the case:**
- *Consistent failures are trustworthy at n=1:* cho2022 aborted 8/8 (every run, both cases);
  zhang2026 Lissajous missed by 10-12 m at 12-15 m/s in 4/4.
- *Everything else is not:* the same case can land on the deck, crash beside it, or abort.
  zhang2026 IC1's campaign run (2.65 m) was an outlier -- 3 repeats landed at 0.14-0.45 m, so the
  campaign UNDERSTATED zhang2026 on the static case. lin2023 IC1 spans 0.18-5.11 m.
- lin2022 is the steadiest landing baseline (7/8 landed, xy 0.12-0.85 m, no multi-metre outliers).
- **0/32 runs precise+soft** across all four baselines. Closest: zhang2026 IC1 0.14 m / 0.31 m/s,
  lin2022 Lissajous 0.12 m / 1.54 m/s (precise needs <=0.10 m, soft <=0.2 m/s).

Verification: all 7 no-result repeats were confirmed as the harness's descent-stall watchdog by
replaying its logic on the recorded altitude -- each ends 25.00 s after its last >0.3 m descent.
Nothing runtime-relevant changed between the campaign and the repeats (only commits since are this
session's own; the B_T saturation is actuator-equivalent to <=6e-5 throttle; `zf` is not on the
lin2022/zhang2026 code path). Only IC1 and Lissajous were repeated -- the other 8 cases per baseline
are single draws of unknown spread. Repeats stay untracked in `RecordBaseline_dev/` (not promoted,
to avoid cherry-picking). **For a paper table, use several runs per case, not these single recordings.**
