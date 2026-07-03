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
    """Side column revealed up to GT sample `idx`: (1) 3D UAV+target tracks,
    (2) |relative position| incl z, (3) |relative velocity|. -> BGR."""
    t = series["t"]; tnow = t[idx]
    dpi = 100
    fig = plt.figure(figsize=(width / dpi, height / dpi), dpi=dpi, facecolor="#111417")
    # 3D spans full width (row 0); the two 2D plots are NARROWER (inset columns,
    # row 1-2 middle sub-column) per request.
    gs = fig.add_gridspec(3, 3, height_ratios=[3.0, 1, 1], width_ratios=[1, 2, 1],
                          hspace=0.5, wspace=0.0)

    # (1) 3D trajectories (full width)
    ax3 = fig.add_subplot(gs[0, :], projection="3d")
    ax3.set_facecolor("#111417")
    k = idx + 1
    ax3.plot(series["ux"][:k], series["uy"][:k], series["uz"][:k], color="#39a7ff", lw=2.0, label="UAV")
    ax3.plot(series["tx"][:k], series["ty"][:k], series["tz"][:k], color="#ffb545", lw=2.0, label="target")
    ax3.scatter(series["ux"][idx], series["uy"][idx], series["uz"][idx], color="#39a7ff", s=28)
    ax3.scatter(series["tx"][idx], series["ty"][idx], series["tz"][idx], color="#ffb545", s=28)
    ax3.set_xlim(*lims["x"]); ax3.set_ylim(*lims["y"]); ax3.set_zlim(*lims["z"])
    ax3.set_xlabel("X (m)", color="w", fontsize=7); ax3.set_ylabel("Y (m)", color="w", fontsize=7)
    ax3.set_zlabel("Z (m)", color="w", fontsize=7)
    ax3.tick_params(colors="#9aa0a6", labelsize=6)
    ax3.xaxis.pane.set_facecolor("#111417"); ax3.yaxis.pane.set_facecolor("#111417"); ax3.zaxis.pane.set_facecolor("#111417")
    ax3.xaxis.pane.set_edgecolor("#3a3f44"); ax3.yaxis.pane.set_edgecolor("#3a3f44"); ax3.zaxis.pane.set_edgecolor("#3a3f44")
    ax3.set_title("UAV & target (3D)", color="w", fontsize=9)
    ax3.legend(loc="upper right", fontsize=6, facecolor="#111417", edgecolor="#3a3f44", labelcolor="w")

    # (2)/(3) norm line plots
    for gi, (label, y, col) in [(1, ("|rel. position| (m)", series["rpos"], "#ff5c5c")),
                                (2, ("|rel. velocity| (m/s)", series["rvel"], "#7CFC00"))]:
        ax = fig.add_subplot(gs[gi, 1])          # narrower centre sub-column
        ax.set_facecolor("#111417")
        ax.plot(t, y, color=col, alpha=0.25, lw=1.2)
        ax.plot(t[:k], y[:k], color=col, lw=2.0)
        ax.plot(tnow, y[idx], "o", color=col, ms=6)
        ax.axvline(tnow, color="w", alpha=0.35, lw=1.0)
        ax.set_ylabel(label, color="w", fontsize=9)
        ax.tick_params(colors="#9aa0a6", labelsize=7)
        for s in ax.spines.values():
            s.set_color("#3a3f44")
        ax.set_xlim(0, t[-1]); ax.margins(y=0.15)
        if gi == 2:
            ax.set_xlabel("time since descent start (s)", color="w", fontsize=8)
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
    plt.close(fig)
    return cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)


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
    ap.add_argument("--run", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=25.0)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--pip-frac", type=float, default=0.28)
    ap.add_argument("--pip-corner", default="tl", choices=["tl", "tr", "bl", "br"])
    ap.add_argument("--tail-s", type=float, default=1.0,
                    help="seconds of video to keep rolling AFTER touchdown (graph freezes)")
    a = ap.parse_args()

    chase = cv2.VideoCapture(a.chase)
    drone = cv2.VideoCapture(a.drone)
    cN = int(chase.get(cv2.CAP_PROP_FRAME_COUNT))
    dN = int(drone.get(cv2.CAP_PROP_FRAME_COUNT))
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
    dur = float(series["t"][-1])             # GT sim descent duration (to touchdown)
    tail = max(0.0, a.tail_s)
    # fixed 3D axis limits over the whole descent (so the view doesn't jump per frame)
    pad = 0.5
    xs = np.concatenate([series["ux"], series["tx"]]); ys = np.concatenate([series["uy"], series["ty"]])
    zs = np.concatenate([series["uz"], series["tz"]])
    lims = {"x": (xs.min() - pad, xs.max() + pad), "y": (ys.min() - pad, ys.max() + pad),
            "z": (min(zs.min(), 0.0), zs.max() + pad)}

    H = a.height
    main_w = int(H * 4 / 3)                # chase main panel (4:3-ish)
    plot_w = int(H * 0.52)                 # graph column width (narrower -> more room for chase)
    W = main_w + plot_w
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    vw = cv2.VideoWriter(a.out, fourcc, a.fps, (W, H))

    ccache = {"i": -1, "f": None}
    dcache = {"i": -1, "f": None}
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
            gi = min(gN - 1, int(round(u * (gN - 1))))
        else:                               # TAIL: hold touchdown frame + graph frozen
            ci, di, gi = td_c, td_d, gN - 1
        cf = grab(chase, ci, ccache)
        df = grab(drone, di, dcache)
        if cf is None:
            break
        canvas = np.zeros((H, W, 3), np.uint8)
        canvas[:, :main_w] = fit(cf, main_w, H)
        # PiP drone
        if df is not None:
            pw = int(main_w * a.pip_frac)
            ph = int(pw * df.shape[0] / df.shape[1])
            pip = cv2.resize(df, (pw, ph))
            m = 16
            pos = {"tl": (m, m), "tr": (m, main_w - pw - m),
                   "bl": (H - ph - m, m), "br": (H - ph - m, main_w - pw - m)}[a.pip_corner]
            y0, x0 = pos
            cv2.rectangle(canvas, (x0 - 2, y0 - 2), (x0 + pw + 2, y0 + ph + 2), (255, 255, 255), 2)
            canvas[y0:y0 + ph, x0:x0 + pw] = pip
            cv2.putText(canvas, "onboard", (x0 + 4, y0 + ph - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        # side plots
        canvas[:, main_w:] = fit(render_plots(series, gi, plot_w, H, lims), plot_w, H)
        vw.write(canvas)
        if f % 25 == 0:
            print(f"[montage]  frame {f}/{nframes}", flush=True)

    vw.release(); chase.release(); drone.release()
    print(f"[montage] done -> {a.out}", flush=True)


if __name__ == "__main__":
    main()
