---
name: project_20260924_stroke_terminal_ring_radius
description: "stroke detector's terminal (<0.3 m) refusals traced to a sigma-vs-pixel-width unit slip in the ring-topology radius; RING_K 2.5->4 fixes it offline (not yet in src)"
metadata:
  node_type: memory
  type: project
  originSessionId: b50a458b-5a2b-4939-9b33-eb63d1375bc0
  modified: 2026-09-24T16:36:12.760Z
---

2026-09-24, offline, `CROSS_DETECTOR=stroke` (commit 19cdabff). Follows [[project_20260924_stroke_detector_rewrite]] ("terminal <0.3 m refusals — next to fix") and [[project_20260924_rover_platform_corner_detection]].

**Mechanism (verified by drawing):** below ~0.3 m the arms are ~60-70 px wide. The stub bisects the two lower arms (45° each side), so those arms only separate from the stub beyond ~w/(2·sin22.5°) ≈ 90 px from the junction. When the junction sits off-centre, one arm's side has too little frame room (e.g. 130 px to the edge) and gets ZERO centreline support → bilateral balance = 0 → falls to the ring fallback (`_ring_x`).
- The ring radius is `2.5 * width`, and `width` is the ridge filter's **σ, not the stroke's pixel width** (a bar's ridge response peaks at σ ≈ 0.29 × width). So 2.5σ ≈ 0.7 stroke widths = INSIDE the merged junction blob → 0-2 transitions → `stroke_not_x_junction`. The docstring intends "2.5 stroke widths (outside the blob)".
- `vmin` (2σ) and the side `gap` (2σ) are in the same σ units. Raising `vmin` alone can't fix this (the partner line's short side becomes unverifiable too — why the other session's 4w→2w "did nothing").

**Offline result (scratch copy with `XP_RING_K`, full PerceptionEvalSet, 23 tags, 7,377 frames):**
- RING_K 2.5 (current) / **4** / 5 / 6: detOK 98.2 / **99.0** / 99.0 / 98.9%; poison 16 / **8** / 13 / 14; <0.3 m detOK 84.9 / **98.3** / 98.3 / 96.6%.
- At 4σ no tag gets more poisoned. 5-6σ add poison (darkbg, rover_circ_r3, inv). Best = **4σ ≈ 1.15 stroke widths**.
- IC3 rover (my RoverIC_raw, 3 reps): <0.3 m detOK 81 → 100%, poison 1 → 0.

**Status:** NOT applied to src/ (another session owns cross_stroke_detector.py and was running SITL on it). Proposed one-line change: ring radius default 2.5 → 4.0 as an env knob. Stroke is still default-off.

**Gotcha found on the way:** validate_detector_gt.py reads `meta.json` for `marker_dz`; a rover run without it defaults to 0.0 → the true-bearing reference is wrong (my first stroke-vs-legacy rover numbers were off: err 0.057 → 0.010 once marker_dz=0.5 was added). `test_data/RoverIC_raw/record_rover_ic_raw.sh` now writes meta.json + copies frames.tsv. Pairing of the 09-23 IC3 reps (tail-offset, no frames.tsv) checked by lag scan: best lag 0/0/−1.
