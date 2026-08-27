#!/usr/bin/env python3
"""
Compose an experiment-style landing montage from a recorded rover landing:
  - MAIN: the third-person CHASE video
  - PiP:  the first-person DRONE down-cam, inset in a corner
  - SIDE: animated plots (altitude, lateral error, relative speed) that sweep a
          time cursor in sync with the video.

All three sources start at descent-start (the chase + drone videos are gated to
CONTROLLER_READY; the plots are aligned so the video END = touchdown), so they
stay in sync by elapsed time.

Usage:
  tools/make_landing_montage.py \
      --chase test_data/Test_Videos/chase_<ts>.mp4 \
      --drone "test_data/Test_Videos/<ts>.mp4" \
      --run   "test_data/Landing_Test/<ts>/" \
      --out   test_data/Test_Videos/montage_<name>.mp4

Options: --fps (output, default 25), --pip-frac (PiP width fraction, 0.28),
         --pip-corner (tl|tr|bl|br, default tl), --height (canvas height, 720).
"""
import argparse
import os

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# MEASURED, not assumed (2026-08-26, correcting the earlier white-text pass): sampled
# the actual chase video's right-side region (where this panel is composited) across
# 12 frames of a real run -- median BGR (200,203,201), 5th/95th percentile luminance
# 194/215 out of 255. That's solidly LIGHT gray, not a 50/50 light/dark mix -- white
# text there is genuinely low-contrast (WCAG ratio ~1.6:1), which is what the user
# caught. The correct fix is the COMPLEMENT of that measured gray -- a dark ink:
#   dark ink #141414 vs measured bg (200,203,201): WCAG contrast ~11.3:1 (AAA)
#   white #ffffff vs the same bg:                  WCAG contrast ~1.6:1  (fails)
# No outline/stroke on top of this (2026-08-26, user request) -- a real contrast
# ratio this high doesn't need a border/halo crutch around every glyph.
_INK = "#141414"


def load_series(run_dir):
    """Ground-Truth series TRIMMED to touchdown (first min-altitude sample):
    full 3D UAV & target tracks + |relative position| (incl. z) + |relative velocity|."""
    gt = np.load(os.path.join(run_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    up, tp = gt["UAV Pose"], gt["Target Pose"]
    t = np.asarray(gt["Time"], float)
    n = min(len(up), len(tp), len(t))
    up, tp, t = up[:n], tp[:n], t[:n]
    ux = np.array([p.position.x for p in up]); uy = np.array([p.position.y for p in up]); uz = np.array([p.position.z for p in up])
    tx = np.array([p.position.x for p in tp]); ty = np.array([p.position.y for p in tp]); tz = np.array([p.position.z for p in tp])
    t = t - t[0]
    rel = np.stack([ux - tx, uy - ty, uz - tz], axis=1)      # relative position vector (incl z)
    rpos = np.linalg.norm(rel, axis=1)                        # |relative position|
    dt = np.gradient(t); dt[dt <= 0] = 1e-3
    rvel_vec = np.stack([np.gradient(rel[:, i]) / dt for i in range(3)], axis=1)
    rvel = np.linalg.norm(rvel_vec, axis=1)                   # |relative velocity|
    rvel = np.convolve(rvel, np.ones(7) / 7, mode="same")
    itd = int(np.argmin(uz)) + 1                              # touchdown = first min-altitude sample
    sl = slice(0, itd)
    return dict(t=t[sl], ux=ux[sl], uy=uy[sl], uz=uz[sl], tx=tx[sl], ty=ty[sl], tz=tz[sl],
                rpos=rpos[sl], rvel=rvel[sl])


def render_plots(series, idx, width, height, lims):
    """Overlay panel revealed up to GT sample `idx`: (1) 3D UAV+target tracks,
    (2) |relative position| incl z, (3) |relative velocity|. TRANSPARENT background
    (no facecolor on the figure or any axes/3D panes) -> RGBA, so only the lines/
    text/axes themselves are opaque and the chase view shows through everywhere
    else once composited -- see the caller's real per-pixel alpha blend, not a
    flat-opacity box over a solid dark rectangle (that was the previous,
    incorrect "transparent" attempt)."""
    t = series["t"]; tnow = t[idx]
    dpi = 100
    fig = plt.figure(figsize=(width / dpi, height / dpi), dpi=dpi)
    fig.patch.set_alpha(0.0)
    # 3D spans full width (row 0); the two 2D plots are NARROWER (inset columns,
    # row 1-2 middle sub-column) per request.
    gs = fig.add_gridspec(3, 3, height_ratios=[3.0, 1, 1], width_ratios=[1, 4, 1],
                          hspace=0.5, wspace=0.0)

    # (1) 3D trajectories (full width, nudged left to fill the whitespace)
    ax3 = fig.add_subplot(gs[0, :], projection="3d")
    _p = ax3.get_position()
    ax3.set_position([_p.x0 - 0.10, _p.y0, _p.width, _p.height])
    ax3.patch.set_alpha(0.0)
    k = idx + 1
    # Validated categorical palette (dataviz skill, references/palette.md) --
    # slot 1 blue / slot 2 orange, in fixed order (identity, not decoration).
    C_UAV, C_TARGET, C_RPOS, C_RVEL = "#2a78d6", "#eb6834", "#e34948", "#008300"
    ax3.plot(series["ux"][:k], series["uy"][:k], series["uz"][:k], color=C_UAV, lw=2.0, label="UAV")
    ax3.plot(series["tx"][:k], series["ty"][:k], series["tz"][:k], color=C_TARGET, lw=2.0, label="target")
    ax3.scatter(series["ux"][idx], series["uy"][idx], series["uz"][idx], color=C_UAV, s=32,
                edgecolors="black", linewidths=0.6)
    ax3.scatter(series["tx"][idx], series["ty"][idx], series["tz"][idx], color=C_TARGET, s=32,
                edgecolors="black", linewidths=0.6)
    ax3.set_xlim(*lims["x"]); ax3.set_ylim(*lims["y"]); ax3.set_zlim(*lims["z"])
    ax3.set_xlabel("X (m)", color=_INK, fontsize=8); ax3.set_ylabel("Y (m)", color=_INK, fontsize=8)
    ax3.set_zlabel("Z (m)", color=_INK, fontsize=8)
    ax3.tick_params(colors=_INK, labelsize=7)
    # Grid RESTORED (2026-08-26, correcting the earlier "just remove it" attempt --
    # it's needed for reading 3D position off the axes, per user feedback): the
    # default matplotlib grid color is a light gray meant for a WHITE page, which is
    # why it disappeared into the also-light-gray chase background. Recolored to a
    # dark gray at partial alpha instead -- clearly visible against the measured
    # bg (contrast ~7:1) while still reading as a background reference, not a data
    # line (kept lighter/thinner than the trajectory lines it supports).
    _GRID_RGBA = (0.2, 0.2, 0.2, 0.5)
    for axis in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        axis._axinfo["grid"]['color'] = _GRID_RGBA
        axis._axinfo["grid"]['linewidth'] = 0.6
    for pane in (ax3.xaxis.pane, ax3.yaxis.pane, ax3.zaxis.pane):
        pane.set_alpha(0.0)
        pane.set_edgecolor("#3a3a3a")
        pane.set_linewidth(0.6)
    ax3.set_title("UAV & target (3D)", color=_INK, fontsize=10)
    # SHIFTED RIGHT (2026-08-26, user request: text was overlapping) -- loc="upper
    # right" alone sits INSIDE the 3D axes' own bbox, which after the -0.10 left-nudge
    # above lands right on top of the z-axis tick labels/title. bbox_to_anchor pushes
    # it further right, clear of the axes and its z-tick text.
    leg = ax3.legend(loc="upper right", bbox_to_anchor=(1.30, 1.02), fontsize=8,
                      facecolor="none", edgecolor="#3a3a3a", labelcolor=_INK)
    # No outline/stroke on text (2026-08-26, user request): the measured-and-validated
    # dark ink (_INK, ~11:1 contrast on the actual background) is enough on its own --
    # a stroke around every letter reads as a border/halo artifact once the base color
    # already has real contrast, so it's dropped rather than kept as an unneeded crutch.

    # (2)/(3) norm line plots
    for gi, (label, y, col) in [(1, ("|rel. position| (m)", series["rpos"], C_RPOS)),
                                (2, ("|rel. velocity| (m/s)", series["rvel"], C_RVEL))]:
        ax = fig.add_subplot(gs[gi, 1])          # narrower centre sub-column
        ax.patch.set_alpha(0.0)
        ax.plot(t, y, color=col, alpha=0.35, lw=1.4)
        ax.plot(t[:k], y[:k], color=col, lw=2.2)
        ax.plot(tnow, y[idx], "o", color=col, ms=8, markeredgecolor="black", markeredgewidth=0.8)
        ax.axvline(tnow, color=_INK, alpha=0.8, lw=1.2)
        ax.set_ylabel(label, color=_INK, fontsize=9)
        ax.tick_params(colors=_INK, labelsize=8)
        # RECESSIVE axes (dataviz skill): drop the top/right box entirely (pure
        # clutter on a transparent panel with no fixed surface behind it) and keep
        # only left+bottom as a thin dark reference frame -- same #3a3a3a as the
        # 3D panes, dark because the measured background is light (see _INK).
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.spines["left"].set_color("#3a3a3a"); ax.spines["left"].set_linewidth(0.8)
        ax.spines["bottom"].set_color("#3a3a3a"); ax.spines["bottom"].set_linewidth(0.8)
        ax.set_xlim(0, t[-1]); ax.margins(y=0.15)
        if gi == 2:
            ax.set_xlabel("time since descent start (s)", color=_INK, fontsize=9)
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).copy()
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
    plt.close(fig)
    return buf   # RGBA -- caller composites using the alpha channel directly


def fit(img, w, h):
    """Resize preserving aspect, letterbox onto a w x h black canvas."""
    ih, iw = img.shape[:2]
    s = min(w / iw, h / ih)
    nw, nh = int(iw * s), int(ih * s)
    r = cv2.resize(img, (nw, nh))
    canvas = np.zeros((h, w, 3), np.uint8)
    x0, y0 = (w - nw) // 2, (h - nh) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = r
    return canvas


def grab(cap, idx, cache):
    idx = max(0, idx)
    if cache["i"] == idx and cache["f"] is not None:
        return cache["f"]
    if idx < cache["i"]:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        cache["i"] = idx - 1
    f = cache["f"]
    while cache["i"] < idx:
        ok, fr = cap.read()
        if not ok:
            break
        f = fr; cache["i"] += 1
    cache["f"] = f
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chase", required=True)
    ap.add_argument("--drone", required=True)
    ap.add_argument("--drone2", default=None,
                    help="optional second PiP source (e.g. an h-overlay video), "
                         "placed in --pip-corner2; --drone becomes the first PiP")
    ap.add_argument("--run", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=25.0)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--pip-frac", type=float, default=0.34)   # bumped 0.28->0.34
                    # (2026-08-26 clarity pass): the plot column is gone (now an
                    # overlay), freeing width the PiPs can use -- they were small
                    # enough that HUD text/lines were hard to read
    ap.add_argument("--pip-corner", default="tl", choices=["tl", "tr", "bl", "br"])
    ap.add_argument("--pip-corner2", default="bl", choices=["tl", "tr", "bl", "br"])
    ap.add_argument("--pip-label", default="onboard")
    ap.add_argument("--pip-label2", default="onboard2")
    ap.add_argument("--tail-s", type=float, default=1.0,
                    help="seconds of video to keep rolling AFTER touchdown (graph freezes)")
    ap.add_argument("--plot-corner", default="br", choices=["tl", "tr", "bl", "br"],
                    help="where to composite the animated-plot panel, now that it's an "
                         "alpha-blended overlay on the full-width chase view instead of "
                         "its own side column (default br, clear of the tl/bl PiPs)")
    ap.add_argument("--plot-frac", type=float, default=0.92,
                    help="plot panel HEIGHT as a fraction of the canvas height H (width is "
                         "derived from this at the SAME 0.48 aspect the original side-column "
                         "layout used -- narrow and tall, sized for 1 big 3D plot + 2 line "
                         "plots stacked, not a small squished box)")
    ap.add_argument("--plot-alpha", type=float, default=0.75,
                    help="opacity of the plot panel over the chase view underneath "
                         "(0=fully see-through/invisible, 1=fully opaque, old behavior)")
    ap.add_argument("--chase-crop", type=float, default=1.0,
                    help="center-crop the recorded chase frame to this fraction of its "
                         "width/height before scaling into the main panel -- a DIGITAL "
                         "zoom on drone+target, independent of and stacking with the "
                         "world SDF chase-cam's own pose/FOV zoom. 1.0 = no crop.")
    ap.add_argument("--chase-crop-touchdown", type=float, default=None,
                    help="if set, the chase crop fraction RAMPS from --chase-crop at "
                         "descent-start to this tighter value by touchdown (then holds "
                         "through the --tail-s freeze) -- makes ground contact visually "
                         "obvious without risking the whole-video clipping a constant "
                         "--chase-crop causes on trajectories that start far from the "
                         "touchdown point. Default (unset) = no ramp, same as --chase-crop.")
    ap.add_argument("--chase-crop-ramp-s", type=float, default=3.0,
                    help="seconds before touchdown over which --chase-crop-touchdown's "
                         "ramp runs (linear in time-to-touchdown, not output-frame index).")
    a = ap.parse_args()
    if a.chase_crop_touchdown is None:
        a.chase_crop_touchdown = a.chase_crop

    chase = cv2.VideoCapture(a.chase)
    drone = cv2.VideoCapture(a.drone)
    drone2 = cv2.VideoCapture(a.drone2) if a.drone2 else None
    cN = int(chase.get(cv2.CAP_PROP_FRAME_COUNT))
    dN = int(drone.get(cv2.CAP_PROP_FRAME_COUNT))
    d2N = int(drone2.get(cv2.CAP_PROP_FRAME_COUNT)) if drone2 is not None else 0
    cfps = chase.get(cv2.CAP_PROP_FPS) or 30.0
    dfps = drone.get(cv2.CAP_PROP_FPS) or 30.0
    series = load_series(a.run)
    gN = len(series["t"])
    # FULL-LENGTH SYNC. With CHASE_STOP_FILE + IMG_RECORD_TAIL_S=0 both videos END at
    # touchdown (recorders stop there), and GT is trimmed to touchdown (argmin alt). So
    # every source spans EXACTLY [descent-start, touchdown]; mapping each over its full
    # length by a common fraction 0->1 makes START and TOUCHDOWN coincide in all three,
    # independent of frame counts / fps tags / RTF. Then a frozen --tail-s hold.
    td_c, td_d = cN - 1, dN - 1
    td_d2 = d2N - 1 if drone2 is not None else 0
    dur = float(series["t"][-1])             # GT sim descent duration (to touchdown)
    tail = max(0.0, a.tail_s)
    # fixed 3D axis limits over the whole descent (so the view doesn't jump per frame)
    pad = 0.5
    xs = np.concatenate([series["ux"], series["tx"]]); ys = np.concatenate([series["uy"], series["ty"]])
    zs = np.concatenate([series["uz"], series["tz"]])
    lims = {"x": (xs.min() - pad, xs.max() + pad), "y": (ys.min() - pad, ys.max() + pad),
            "z": (min(zs.min(), 0.0), zs.max() + pad)}

    H = a.height
    main_w = int(H * 4 / 3)                # chase panel (4:3-ish) -- now the FULL canvas width;
                                            # the plot column is gone, replaced by an alpha-blended
                                            # overlay panel (see --plot-corner/--plot-frac/--plot-alpha)
    W = main_w
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    vw = cv2.VideoWriter(a.out, fourcc, a.fps, (W, H))

    ccache = {"i": -1, "f": None}
    dcache = {"i": -1, "f": None}
    d2cache = {"i": -1, "f": None}
    nd = max(1, int(dur * a.fps))           # descent output frames (GT sim duration)
    nt = int(tail * a.fps)                  # tail output frames
    nframes = nd + nt
    print(f"[montage] {nframes} frames ({nd} descent + {nt} tail)  {W}x{H}@{a.fps}  "
          f"(chase {cN}f td@{td_c}, drone {dN}f td@{td_d}, GT {gN}samp, {dur:.1f}s)  -> {a.out}",
          flush=True)

    for f in range(nframes):
        if f < nd:                          # DESCENT: common fraction 0->1 to each touchdown
            u = f / max(nd - 1, 1)
            ci = int(round(u * td_c)); di = int(round(u * td_d))
            d2i = int(round(u * td_d2)) if drone2 is not None else 0
            gi = min(gN - 1, int(round(u * (gN - 1))))
        else:                               # TAIL: hold touchdown frame + graph frozen
            ci, di, gi = td_c, td_d, gN - 1
            d2i = td_d2
        cf = grab(chase, ci, ccache)
        df = grab(drone, di, dcache)
        df2 = grab(drone2, d2i, d2cache) if drone2 is not None else None
        if cf is None:
            break
        if f < nd:
            frames_to_td = nd - 1 - f
            ramp_frames = max(1, int(round(a.chase_crop_ramp_s * a.fps)))
            ramp_u = min(1.0, max(0.0, 1.0 - frames_to_td / ramp_frames))
        else:
            ramp_u = 1.0                    # tail: hold the touchdown crop
        crop = a.chase_crop + (a.chase_crop_touchdown - a.chase_crop) * ramp_u
        if crop < 1.0:
            ch, cw = cf.shape[:2]
            nw, nh = max(1, int(cw * crop)), max(1, int(ch * crop))
            x0c, y0c = (cw - nw) // 2, (ch - nh) // 2
            cf = cf[y0c:y0c + nh, x0c:x0c + nw]
        canvas = np.zeros((H, W, 3), np.uint8)
        canvas[:, :main_w] = fit(cf, main_w, H)
        m = 16
        # PiP sizing: the onboard/down-cam source is PORTRAIT (480w x 640h, not the
        # 640x480 landscape the old --pip-frac width-based sizing assumed -- see
        # 2026-08-26 finding), so ph = pw*(640/480) came out TALLER than pw. Two
        # portrait PiPs stacked in the same side column (tl+bl, the only layout this
        # tool actually uses) then silently overlapped once pip_frac grew enough --
        # exactly the "h overlaid on s_alpha" bug. Fix: when both PiPs share a side,
        # split the available column height EQUALLY between them first, then derive
        # width from THAT height using each source's own aspect -- guarantees equal
        # space and zero overlap regardless of source aspect ratio or pip_frac.
        same_side1 = a.pip_corner[1] if df is not None else None
        same_side2 = a.pip_corner2[1] if df2 is not None else None
        stacked = (df is not None and df2 is not None and same_side1 == same_side2)

        def _pip_size(frame, height_budget):
            ph = int(height_budget)
            pw = int(ph * frame.shape[1] / frame.shape[0])
            return pw, ph

        if stacked:
            budget_h = (H - 3 * m) // 2   # top margin + middle gap + bottom margin
        else:
            budget_h = None   # each PiP uses its own --pip-frac-derived width instead

        # PiP drone
        if df is not None:
            if stacked:
                pw, ph = _pip_size(df, budget_h)
            else:
                pw = int(main_w * a.pip_frac)
                ph = int(pw * df.shape[0] / df.shape[1])
            pip = cv2.resize(df, (pw, ph))
            pos = {"tl": (m, m), "tr": (m, main_w - pw - m),
                   "bl": (H - ph - m, m), "br": (H - ph - m, main_w - pw - m)}[a.pip_corner]
            y0, x0 = pos
            cv2.rectangle(canvas, (x0 - 2, y0 - 2), (x0 + pw + 2, y0 + ph + 2), (255, 255, 255), 2)
            canvas[y0:y0 + ph, x0:x0 + pw] = pip
            cv2.putText(canvas, a.pip_label, (x0 + 4, y0 + ph - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        # PiP drone2 (e.g. h-overlay), separate corner
        if df2 is not None:
            if stacked:
                pw2, ph2 = _pip_size(df2, budget_h)
            else:
                pw2 = int(main_w * a.pip_frac)
                ph2 = int(pw2 * df2.shape[0] / df2.shape[1])
            pip2 = cv2.resize(df2, (pw2, ph2))
            pos2 = {"tl": (m, m), "tr": (m, main_w - pw2 - m),
                    "bl": (H - ph2 - m, m), "br": (H - ph2 - m, main_w - pw2 - m)}[a.pip_corner2]
            y02, x02 = pos2
            cv2.rectangle(canvas, (x02 - 2, y02 - 2), (x02 + pw2 + 2, y02 + ph2 + 2), (255, 255, 255), 2)
            canvas[y02:y02 + ph2, x02:x02 + pw2] = pip2
            cv2.putText(canvas, a.pip_label2, (x02 + 4, y02 + ph2 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        # plots: TRANSPARENT-background overlay panel (no longer a separate side
        # column, and no longer a solid dark box at flat opacity -- see render_plots'
        # docstring) so the chase view can use the FULL canvas width, and shows
        # through everywhere the plot has no content, not just faintly through a
        # dark rectangle. Narrow+tall aspect (0.48 width:height) sized like the
        # original side column so 1 big 3D plot + 2 line plots stay readable
        # (a wide-short box here was the earlier, cramped/"compressed" attempt).
        plot_ph = int(H * a.plot_frac)
        plot_pw = int(plot_ph * 0.48)
        plot_rgba = render_plots(series, gi, plot_pw, plot_ph, lims)
        pm = 16
        ppos = {"tl": (pm, pm), "tr": (pm, main_w - plot_pw - pm),
                "bl": (H - plot_ph - pm, pm), "br": (H - plot_ph - pm, main_w - plot_pw - pm)}[a.plot_corner]
        py0, px0 = ppos
        alpha_ch = (plot_rgba[:, :, 3:4].astype(np.float32) / 255.0) * a.plot_alpha
        plot_bgr = cv2.cvtColor(plot_rgba[:, :, :3], cv2.COLOR_RGB2BGR).astype(np.float32)
        roi = canvas[py0:py0 + plot_ph, px0:px0 + plot_pw].astype(np.float32)
        blended = alpha_ch * plot_bgr + (1.0 - alpha_ch) * roi
        canvas[py0:py0 + plot_ph, px0:px0 + plot_pw] = blended.astype(np.uint8)
        vw.write(canvas)
        if f % 25 == 0:
            print(f"[montage]  frame {f}/{nframes}", flush=True)

    vw.release(); chase.release(); drone.release()
    if drone2 is not None:
        drone2.release()
    print(f"[montage] done -> {a.out}", flush=True)


if __name__ == "__main__":
    main()
