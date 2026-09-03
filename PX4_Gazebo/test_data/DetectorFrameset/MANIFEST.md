# DetectorFrameset — curated eval set for cross-marker detector robustness

Recorded 2026-09-02 (IMG_RECORD=1). Paired GT + raw frames for scoring detector
front-end variants offline with `tools/validate_detector_gt.py`:

    ~/ws/scripts/env2025/bin/python3 tools/validate_detector_gt.py --set test_data/DetectorFrameset --all

Each `<tag>/` holds `Ground_Truth.npy`, `Img_Data.npy`, `Control_Data.npy`,
`Img_Params.txt`, and `frames/f%05d.png`. ~230 MB, gitignored (data only; the
harness + this manifest are tracked).

| tag | world | IC (ENU) | frames | GT-FB | recorded landing | baseline detOK | why it's in the set |
|-----|-------|----------|--------|-------|------------------|----------------|---------------------|
| flat_IC2    | cross_marker         | 2,2,5  | 481 | no  | land 0.045 m   | 100%  | clean reference — MUST NOT REGRESS |
| flat_IC1    | cross_marker         | 0,0,5  | 344 | yes | land 0.038 m   | 100%  | clean centered reference |
| clutter_IC2 | cross_marker_clutter | 2,2,5  | 402 | yes | land 0.091 m   | 65%   | one dark box near marker — baseline collapses at altitude |
| clutter_IC1 | cross_marker_clutter | 0,0,5  | 358 | yes | land 0.044 m   | 63%   | dark box, centered descent |
| rover_IC2   | rover_cross          | 2,2,5  | 310 | no  | dive 2.28 m off| 45%   | offset — oblique-view detector collapse (wheels + plate edge) |
| rover_IC4   | rover_cross          | 2,2,7  | 685 | no  | stall @ 0.96 m | 78%   | offset+high — baseline detection mostly OK; the terminal-overfill case |

## Baseline vs `adapt` (CLAHE+adaptiveThreshold, CROSS_ADAPT_GATE=1), 2026-09-02

adapt lifts detOK where baseline fails (clutter 63->94%, rover_IC2 45->86%), is
detection-neutral where baseline works (flat, rover_IC4), BUT drops the
within-0.15 centroid hit-rate ~15-20 pts in every scenario (flat included:
80->66%). => not bakeable; the next variant must keep the detection recovery
AND restore accuracy. See [[feedback_cross_detector_contrast_not_darkness]].

## Worlds

- `cross_marker_clutter.sdf` (PX4 gz worlds dir): clean cross_marker + one dark
  0.6x0.6x0.3 m box VISUAL on the ground_plane link at world (1.2,1.2), V~=31.
  Box is a link visual, NOT a top-level model (a model shifts pose/info indices).
