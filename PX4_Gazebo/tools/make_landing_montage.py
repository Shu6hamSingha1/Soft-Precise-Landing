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
    """Physical time-series from Ground_Truth (altitude, lateral, rel speed),
    TRIMMED to touchdown (first min-altitude sample), and the sim->wall time-scale
    (RTF = sim_span / wall_span) measured from Control_Data's wall-clock 't' so the
    videos (recorded in wall time) align to the GT (sim time)."""
    gt = np.load(os.path.join(run_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    up, tp = gt["UAV Pose"], gt["Target Pose"]
    t = np.asarray(gt["Time"], float)
    n = min(len(up), len(tp), len(t))
    up, tp, t = up[:n], tp[:n], t[:n]
    uz = np.array([p.position.z for p in up])
    ux = np.array([p.position.x for p in up]); uy = np.array([p.position.y for p in up])
    tx = np.array([p.position.x for p in tp]); ty = np.array([p.position.y for p in tp])
    t = t - t[0]
    lat = np.hypot(ux - tx, uy - ty)
    dt = np.gradient(t); dt[dt <= 0] = 1e-3
    rvx = np.gradient(ux - tx) / dt; rvy = np.gradient(uy - ty) / dt
    rvz = np.gradient(uz) / dt
    spd = np.sqrt(rvx**2 + rvy**2 + rvz**2)
    spd = np.convolve(spd, np.ones(7) / 7, mode="same")
    # TRIM to touchdown = first global-min-altitude sample (logging may run on past
    # it; the videos then keep rolling too, which is what broke the old sync).
    itd = int(np.argmin(uz)) + 1
    t, uz, lat, spd = t[:itd], uz[:itd], lat[:itd], spd[:itd]
    # sim->wall scale from the controller's wall-clock 't' (perf_counter). ~1 for
    # headless SITL, but measured so a non-unity RTF doesn't reintroduce drift.
    rtf = 1.0
    try:
        cd = np.load(os.path.join(run_dir, "Control_Data.npy"), allow_pickle=True).item()
        wt = np.asarray(cd["t(t)"] if "t(t)" in cd else cd["t"], float)
        wall_span = float(wt[-1] - wt[0]); sim_span = float(t[-1] - t[0]) if len(t) > 1 else 0.0
        # use the FULL GT sim span vs full wall span (both cover the same flight)
        sim_full = float(np.asarray(gt["Time"], float)[-1] - np.asarray(gt["Time"], float)[0])
        if wall_span > 0.5 and sim_full > 0.5:
            rtf = sim_full / wall_span
    except Exception:
        pass
    return dict(t=t, alt=uz, lat=lat, spd=spd, rtf=rtf)


def render_plots(series, idx, width, height):
    """Render the side plots revealed up to GT sample `idx` (matplotlib -> BGR)."""
    t = series["t"]
    tnow = t[idx]
    panels = [("Altitude (m)", series["alt"], "#39a7ff"),
              ("Lateral error (m)", series["lat"], "#ff5c5c"),
              ("Rel. speed (m/s)", series["spd"], "#7CFC00")]
    dpi = 100
    fig, axes = plt.subplots(len(panels), 1, figsize=(width / dpi, height / dpi),
                             dpi=dpi, facecolor="#111417")
    if len(panels) == 1:
        axes = [axes]
    for ax, (label, y, col) in zip(axes, panels):
        ax.set_facecolor("#111417")
        ax.plot(t, y, color=col, alpha=0.25, lw=1.2)                 # full (faint)
        ax.plot(t[:idx + 1], y[:idx + 1], color=col, lw=2.0)         # revealed
        ax.plot(tnow, y[idx], "o", color=col, ms=6)
        ax.axvline(tnow, color="w", alpha=0.35, lw=1.0)
        ax.set_ylabel(label, color="w", fontsize=9)
        ax.tick_params(colors="#9aa0a6", labelsize=7)
        for s in ax.spines.values():
            s.set_color("#3a3f44")
        ax.set_xlim(0, t[-1])
        ax.margins(y=0.15)
    axes[-1].set_xlabel("time since descent start (s)", color="w", fontsize=8)
    fig.tight_layout(pad=0.6)
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
    plt.close(fig)
    return cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)


def detect_touchdown_frame(cap, nframes):
    """Last frame with significant inter-frame motion, in the ONBOARD down-cam.
    At touchdown the drone is rigid on the platform and the marker is fixed in its
    FoV -> inter-frame motion collapses (true for stationary AND moving platforms).
    Returns that frame index, or nframes-1 if no clear collapse (fallback)."""
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    prev = None; motion = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        g = cv2.cvtColor(cv2.resize(fr, (160, 120)), cv2.COLOR_BGR2GRAY).astype(np.float32)
        if prev is not None:
            motion.append(float(np.mean(np.abs(g - prev))))
        prev = g
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    if len(motion) < 8:
        return nframes - 1
    motion = np.asarray(motion)
    thr = 0.20 * float(np.percentile(motion, 90))      # 20% of the busy-phase level
    active = np.where(motion > thr)[0]
    if len(active) == 0:
        return nframes - 1
    # last active frame + 1 (the diff index i is motion between frame i and i+1)
    return int(min(active[-1] + 1, nframes - 1))


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
    # TOUCHDOWN-ANCHORED SYNC (robust to RECORD_S overshoot, wrong fps tags, RTF!=1):
    # both videos are gated to descent-start (frame 0 = GT t=0). Detect touchdown in
    # the ONBOARD down-cam (motion collapses when the drone is rigid on the platform —
    # true for a MOVING platform too, where the chase view keeps translating). Anchor
    # the graph cursor to reach its END (= GT touchdown sample) at that SAME wall time,
    # so footage and cursor hit touchdown together regardless of clock. Then keep the
    # video rolling for --tail-s seconds while the graph stays frozen at touchdown.
    td_d = detect_touchdown_frame(drone, dN)
    td_time = td_d / dfps                    # wall seconds to touchdown
    tail = max(0.0, a.tail_s)

    H = a.height
    main_w = int(H * 4 / 3)                # chase main panel (4:3-ish)
    plot_w = int(H * 0.62)
    W = main_w + plot_w
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    vw = cv2.VideoWriter(a.out, fourcc, a.fps, (W, H))

    ccache = {"i": -1, "f": None}
    dcache = {"i": -1, "f": None}
    nframes = max(1, int((td_time + tail) * a.fps))
    print(f"[montage] {nframes} frames  {W}x{H}@{a.fps}  (chase {cN}f@{cfps:.0f}, "
          f"drone {dN}f@{dfps:.0f}, GT {gN}samp; touchdown drone-frame {td_d} = "
          f"{td_time:.1f}s + {tail:.1f}s tail)  -> {a.out}", flush=True)

    for f in range(nframes):
        tw = f / a.fps                      # wall seconds since descent start (video clock)
        ci = min(cN - 1, int(round(tw * cfps)))
        di = min(dN - 1, int(round(tw * dfps)))
        # graph cursor: advance to touchdown (gN-1) over [0, td_time], then FREEZE
        u = min(1.0, tw / td_time) if td_time > 1e-6 else 1.0
        gi = min(gN - 1, int(round(u * (gN - 1))))
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
        canvas[:, main_w:] = fit(render_plots(series, gi, plot_w, H), plot_w, H)
        vw.write(canvas)
        if f % 25 == 0:
            print(f"[montage]  frame {f}/{nframes}", flush=True)

    vw.release(); chase.release(); drone.release()
    print(f"[montage] done -> {a.out}", flush=True)


if __name__ == "__main__":
    main()
