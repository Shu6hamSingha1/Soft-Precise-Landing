---
name: project_20260923_manuscript_video_symbol_convention
description: "VISTA-GT + comparative-baseline landing videos now render manuscript-matching math symbols (nu not h, ᴵ calligraphic frame prefix, Computer-Modern mathtext font) and no redundant PiP captions."
metadata:
  node_type: memory
  type: project
  modified: 2026-09-23T01:43:20.127Z
  originSessionId: d4885eeb-2b11-4480-8a1e-f87fe4c8ec53
---

2026-09-23 session: the ICRA manuscript (`Soft_Precise_Landing/ICRA.tex`) renamed the
depth-normalized optic-flow feature `h` -> `\boldsymbol\nu`. The recorded PX4/Gazebo
landing videos under `test_data/Final/VISTA-GT/` (and later the comparative-baseline
folders) were updated to match, plus the plot panel's axis/legend symbols were aligned
to `Soft_Precise_Landing/Figures/rover_cross_circular_{3d,rpos,rvel}.png` (the
manuscript's own rendered figures), which use a left-superscript calligraphic ℐ
(inertial frame) and bold `r`/`v` with `_b`/`_t`/`_rel` subscripts.

**Tools edited (both committed, reusable):**
- `tools/overlay_image_features.py`: `h=(...)` HUD text -> `ν=(...)` (line ~477).
- `tools/make_landing_montage.py`:
  - `matplotlib.rcParams["mathtext.fontset"] = "cm"` added right after `matplotlib.use("Agg")`
    -- REQUIRED for `\mathcal{I}` to render as the flowing calligraphic script LaTeX's
    default Computer-Modern math font gives it. The matplotlib DEFAULT fontset
    ("dejavusans") renders `\mathcal` as a plain italic letter instead -- looks wrong,
    user caught this immediately on first review. Don't skip this rcParam when touching
    this file again.
  - 3D plot xlabel/ylabel/zlabel: `"X (m)"` etc -> `r"$^{\mathcal{I}}x$ [m]"` etc.
  - 3D plot legend: `label="UAV"`/`"target"` -> `r"$^{\mathcal{I}}\mathbf{r}_\mathrm{b}$"`/
    `r"$^{\mathcal{I}}\mathbf{r}_\mathrm{t}$"`.
  - rel-position/rel-velocity ylabels -> `r"$\|^{\mathcal{I}}\mathbf{r}_\mathrm{rel}\|$ (m)"`
    / `r"$\|^{\mathcal{I}}\mathbf{v}_\mathrm{rel}\|$ (m/s)"`.
  - bottom xlabel `"time since descent start (s)"` -> `r"$t$ (s)"`.
  - The `cv2.putText` calls drawing `pip_label`/`pip_label2` ("onboard"/"onboard2" or
    "s/alpha"/"h/w") under each PiP panel were REMOVED entirely (2026-09-23, separate
    user request) -- redundant with the PiP's own burned-in `s=`/`α=`/`ν=`/`w=`
    HUD text.
  - `--split` mode (`SPLIT_GROUPS = (("s","alpha"), ("h",))`) always sets `draw_w=True`
    for the h group, so `w=(...)` is now drawn on every regenerated case uniformly (some
    earlier IC1-5 recordings were missing it before this pass; VISTA-GT's Static/Linear/
    Sinusoidal/Circular/Lissajous already had it).

**Applied to:** all 10 VISTA-GT cases (IC1-5 + Static/Linear/Sinusoidal/Circular/
Lissajous) -- `overlay_h.mp4` and `montage.mp4` regenerated for each, approved by user
per-case starting with IC1. Also applied when generating the comparative-baseline
montages (though those omit the s/alpha and h/w overlay PiPs by earlier-agreed scope --
see [[project_20260923_comparative_baseline_campaign]] -- so only the 3D/plot-panel
symbol+font fix applies there, not the nu/w overlay text).

**Gotcha hit mid-task:** a CONCURRENT session replaced the `Lissajous` VISTA-GT
recording (old rep had landed on a parked rover -- not genuine tracking, see
`test_data/Final/MANIFEST.md`'s 2026-09-23-later note) while this symbol-convention
regen was in flight. The label-removal batch pass almost overwrote the fresh replacement
with a caption-removed version built from the now-STALE cached overlay sources. Caught
by checking file mtimes against the batch-job's own source timestamps before copying
anything over; redid Lissajous specifically from its new raw footage. General lesson
matches [[feedback_recurring_analysis_mistakes]] §20-style concurrent-session pollution
-- always re-verify source freshness right before a copy-over step in a long batch job,
not just at the start.
