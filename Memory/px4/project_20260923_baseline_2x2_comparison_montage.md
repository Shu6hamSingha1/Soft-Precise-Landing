---
name: project_20260923_baseline_2x2_comparison_montage
description: "New tool tools/make_baseline_comparison_montage.py builds a 2x2 chase-cam+plots grid comparing all 4 baselines per target profile, with a controller-abbreviation + landing-status label; design decisions + current label styling recorded here."
metadata:
  node_type: memory
  type: project
  modified: 2026-09-23T03:57:50.974Z
  originSessionId: 2c3ac54e-b6e9-4806-8262-7d543a7bd011
---

2026-09-23, user request: a single video per target profile that puts all 4
comparison-baseline controllers side by side (2x2) instead of separate per-controller
videos, for the manuscript/presentation. Built on top of
[[project_20260923_comparative_baseline_campaign]]'s recorded data
(`test_data/Final/{LIN2022,ZHANG2026,LIN2023,CHO2022}-GT/`) and the fixed shared
montage tool from [[feedback_montage_touchdown_argmin_bug]].

**New tool:** `PX4_Gazebo/tools/make_baseline_comparison_montage.py`
(`--profile <name>` or `--all`), output -> `test_data/Final/Baseline_Comparison/
<profile>_2x2.mp4`. Only the 5 trajectory PROFILES (Static/Circular/Linear/
Lissajous/Sinusoidal) got a 2x2 -- IC1-5 are stationary-target camera-offset cases,
not target profiles, and were intentionally excluded from this "all target profiles"
request; don't conflate the two when asked for "all profiles" again.

**Panel sequence (fixed, per user spec):** top-left PBVS-PPC (LIN2022-GT), top-right
PBVS-AEDO (ZHANG2026-GT), bottom-left IBVS-PPC (LIN2023-GT), bottom-right FF-IBVS
(CHO2022-GT).

**Panel content, 3 revisions same day (chronological, latest wins):**
1. First cut reused the precomposited per-controller `*_montage.mp4` (which bakes in
   an onboard-cam PiP + plots via `make_landing_montage.py`) resized into 4 tiles.
2. User: "remove onboard view, just chase-cam + plots" -> rewrote to build each
   panel FRESH from `<profile>_chase_cam.mp4` + `render_plots()`/`load_series()`
   reused directly from `make_landing_montage.py` (import, not reimplementation),
   with the same sim-time chase sync scheme (chase spans exactly
   [descent-start, touchdown], mapped by `cfps_eff = (cN-1)/dur`). Titles switched
   to Pillow-rendered Times New Roman (`cv2.putText` has no TTF support; system font
   at `/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf`).
3. User iterated the label 3 more times same day -> current state: solid black
   banner (`bar_h = max(52, h*0.13)`), controller abbreviation in white + landing
   tag in YELLOW, both on ONE line side by side (title first, tag drawn at
   `title_x_end + 24px`), tag text upper-cased at ~077% of the title's font size as
   a small-caps approximation (Times New Roman has no real small-caps variant in
   Pillow). Both fonts sized off `bar_h` (title 0.62x, tag 0.48x) -- bigger than the
   very first (translucent-overlay, 2-line, colored-by-tag) version.

**Landing-status tag derivation** (`tag_for()`, mirrors
`tools/build_test_record.py::classify` collapsed to the user's 5 requested labels):
reads `Ground_Truth.npy['SoftPrecise']`; empty dict OR `target_lost=True` ->
`aborted` (matches the [[feedback_montage_touchdown_argmin_bug]] finding that empty
`SoftPrecise` = never reached touchdown detection, e.g. ALL of cho2022); else
`precise and soft` -> soft-precise, `precise and not soft` -> hard-precise,
`soft and not precise` -> soft-imprecise, else hard-imprecise. **Result across all
20 baseline reps used here: zero soft-precise, zero hard-precise, zero soft-
imprecise landings** -- every panel across all 5 profiles is either hard-imprecise
or aborted, consistent with the campaign memory's headline (zhang2026 lands
everywhere but very imprecise; lin2022/lin2023 land roughly half, hard-imprecise
when they do; cho2022 aborted 0/10 universally, so every CHO2022-GT panel in every
2x2 reads ABORTED).

**Sync across the 4 unequal-length panels:** each panel freezes on its own last
frame once its own descent+1s-tail ends; the overall 2x2 runs for
`max(nframes across the 4 panels)` so no panel gets cut short.

**Performance note:** re-rendering `render_plots()`'s matplotlib 3D+2-line figure
per frame, per panel (4x), is slow -- roughly 10-15 min per profile depending on
length (Circular's 1065 frames took ~14 min). Building all 5 sequentially took
~50 min total. Not worth optimizing unless this becomes a repeated task.
