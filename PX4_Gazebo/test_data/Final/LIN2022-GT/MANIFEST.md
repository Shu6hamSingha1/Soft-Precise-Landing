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
