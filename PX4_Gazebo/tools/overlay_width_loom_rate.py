#!/usr/bin/env python3
"""Overlay the LINE-WIDTH LOOM-RATE measurement onto a recorded down-cam video,
in the same spirit as tools/overlay_image_features.py (which draws s / alpha /
h / w). This one visualises the pipeline behind `"Width Loom Rate"`:

  cross_marker_detector.detect()  ->  det.line_points_i / _j  (the pruned arm
      inlier points)  +  det.isolated_mask
  width_loom_from_detection(det):
      - per arm: PCA direction v of the arm points, normal nrm = [-v_y, v_x]
      - pick 5 station points spread over the middle 70% of the arm's extent
      - at each station: bilinear sub-pixel perpendicular walk through
        isolated_mask both ways, count the continuous on-mask run -> thickness
      - median across >=3 on-mask stations = that arm's width
      - median of the two arms, IF they agree within _WLOOM_ARM_AGREE_RATIO
  ln(width) -> a 2-state (value,rate) KF (_kf_step, Q/R from CROSS_WLOOM_KF_*)
      -> reported loom-rate = -x[1]   (Tz sign convention)

Per frame it draws:
  * each arm's fitted line (cyan / yellow) + its PCA direction
  * the 5 scan stations (filled dots; hollow = station itself was OFF-mask -> skipped)
  * the perpendicular scan segment at each on-mask station, length = measured
    thickness (green = counted, drawn from  p - s_minus*nrm  to  p + s_plus*nrm)
  * per-arm median width, the arm-agreement ratio, and the frame's final width
  * a bottom time-series panel: ln(width), the KF loom-rate, and GT loom
    (from tools/gt_optical_flow.py), with a cursor at the current frame

Needs the RAW IMG_RECORD frames (not the lossy mp4) so the detector sees exactly
what the live pipeline saw. Use it on an OverfillCapture-style rep.

Usage:
  tools/overlay_width_loom_rate.py \
    --raw  "test_data/Test_Videos/Tue Sep  8 13-38-07 2026_raw" \
    --run   test_data/OverfillCapture_IC1/rep1_data \
    --out   test_data/Test_Videos/wloom_rate_ic1rep1.mp4
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))
sys.path.insert(0, _HERE)
import cross_marker_detector as cmd            # noqa: E402
import cross_marker_perception as cmp          # noqa: E402
from cross_marker_perception import (          # noqa: E402
    _kf_step, _wloom_bilinear, width_loom_from_detection,
    _WLOOM_DS, _WLOOM_MAX_STEPS, _WLOOM_THRESH, _WLOOM_N_STATIONS,
    _WLOOM_MIN_QUORUM, _WLOOM_ARM_AGREE_RATIO,
)

try:
    import gt_optical_flow as gtof
except Exception:
    gtof = None

Q = float(os.environ.get("CROSS_WLOOM_KF_Q", "10.0"))
R = float(os.environ.get("CROSS_WLOOM_KF_R", "0.005"))
DT_UNC_MAX = 2.0

CYAN = (255, 220, 0)
YELL = (0, 220, 255)
GREEN = (60, 220, 60)
GREY = (150, 150, 150)
RED = (60, 60, 235)
WHITE = (255, 255, 255)


def _scan_split(mask, p0, nrm):
    """Re-walk _wloom_scan_thickness both ways, returning (s_plus, s_minus, total)
    so the measured run can be DRAWN, not just summed. Identical walk to
    cross_marker_perception._wloom_scan_thickness."""
    out = []
    for sign in (1.0, -1.0):
        s = 0.0
        for _ in range(_WLOOM_MAX_STEPS):
            sn = s + _WLOOM_DS
            x = p0[0] + sign * sn * nrm[0]
            y = p0[1] + sign * sn * nrm[1]
            if _wloom_bilinear(mask, x, y) < _WLOOM_THRESH:
                break
            s = sn
        out.append(s)
    return out[0], out[1], out[0] + out[1] + _WLOOM_DS


def _arm_stations(mask, pts):
    """Mirror _wloom_width_at_points' geometry so every intermediate can be drawn.
    Returns dict with v, nrm, and a list of per-station (p, on_mask, s_plus,
    s_minus, thickness) plus the arm's median width (or None)."""
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 4:
        return None
    c = pts.mean(axis=0)
    ctr = pts - c
    _, _, VT = np.linalg.svd(ctr, full_matrices=False)
    v = VT[0]
    nrm = np.array([-v[1], v[0]])
    tproj = ctr @ v
    order = np.argsort(tproj)
    n = len(order)
    lo, hi = int(0.15 * n), int(0.85 * n)
    if hi <= lo:
        idxs = order
    else:
        idxs = order[np.linspace(lo, hi - 1, min(_WLOOM_N_STATIONS, hi - lo)).astype(int)]
    stations = []
    th = []
    for k in idxs:
        p = pts[k]
        on = _wloom_bilinear(mask, p[0], p[1]) >= _WLOOM_THRESH
        sp = sm = t = np.nan
        if on:
            sp, sm, t = _scan_split(mask, p, nrm)
            th.append(t)
        stations.append((p, on, sp, sm, t))
    med = float(np.median(th)) if len(th) >= _WLOOM_MIN_QUORUM else None
    return dict(v=v, nrm=nrm, c=c, stations=stations, width=med)


def _panel(traces, cur, W, Hp):
    """Bottom time-series strip: dict name -> (y_array, color, y0, y1)."""
    img = np.full((Hp, W, 3), 28, np.uint8)
    n = len(next(iter(traces.values()))[0])
    if n < 2:
        return img
    xs = np.linspace(6, W - 6, n).astype(int)
    # zero line for the rate/loom traces
    for name, (y, color, y0, y1) in traces.items():
        yy = np.clip((y - y0) / (y1 - y0 + 1e-9), 0, 1)
        py = (Hp - 10 - yy * (Hp - 20)).astype(float)
        pts = np.array([[xs[i], py[i]] for i in range(n) if np.isfinite(py[i])], np.int32)
        if len(pts) > 1:
            cv2.polylines(img, [pts], False, color, 1, cv2.LINE_AA)
        cv2.putText(img, name, (8, 14 + 14 * list(traces).index(name)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
    cx = int(xs[min(cur, n - 1)])
    cv2.line(img, (cx, 0), (cx, Hp), (90, 90, 90), 1)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw", required=True, help="raw IMG_RECORD frame dir (f*.png)")
    ap.add_argument("--run", required=True, help="rep dir with Img_Data.npy (+ Ground_Truth.npy)")
    ap.add_argument("--out", required=True, help="output mp4")
    ap.add_argument("--fps", type=float, default=20.0)
    args = ap.parse_args()

    frames = sorted(glob.glob(os.path.join(args.raw, "f*.png")))
    if not frames:
        raise SystemExit(f"no f*.png in {args.raw}")
    img = np.load(os.path.join(args.run, "Img_Data.npy"), allow_pickle=True).item()
    it = np.asarray(img["Time"], float)
    nf = len(frames)
    off = max(0, len(it) - nf)
    tfr = it[off:off + nf]            # align-from-the-end (overlay_image_features.py convention)

    # ---- pass 1: detector + width + KF ----
    ts = {"last_bbox": None, "miss_count": 0}
    kf_x = np.zeros((1, 2)); kf_P = np.tile(np.eye(2), (1, 1, 1))
    kf_pt = None; kf_i = False
    per = []
    W_arr = np.full(nf, np.nan); lnW = np.full(nf, np.nan); rate = np.full(nf, np.nan)
    last_w = np.nan
    for k, fp in enumerate(frames):
        fr = cv2.imread(fp)
        det = cmd.detect(fr, track_state=ts) if fr is not None else None
        armA = armB = None
        wv = None
        if det is not None and det.ok and det.isolated_mask is not None:
            armA = _arm_stations(det.isolated_mask, det.line_points_i)
            armB = _arm_stations(det.isolated_mask, det.line_points_j)
            try:
                wv = width_loom_from_detection(det)
            except Exception:
                wv = None
        per.append((det, armA, armB, wv))
        if wv is not None and wv > 0:
            last_w = wv
        W_arr[k] = wv if (wv is not None and wv > 0) else np.nan
        t = float(tfr[k])
        z = np.array([np.log(wv)]) if (wv is not None and wv > 0 and np.isfinite(t)) else None
        if z is not None or kf_i:
            kf_x, kf_P, kf_pt, kf_i = _kf_step(kf_x, kf_P, kf_pt, kf_i, z, t, Q, R, dt_unc_max=DT_UNC_MAX)
        rate[k] = float(-kf_x[0, 1]) if kf_i else np.nan
        lnW[k] = np.log(wv) if (wv is not None and wv > 0) else np.nan

    # ---- GT loom aligned to frames ----
    gt_loom = np.full(nf, np.nan); gt_alt = np.full(nf, np.nan)
    if gtof is not None and os.path.exists(os.path.join(args.run, "Ground_Truth.npy")):
        try:
            g = gtof.compute_gt_flow(args.run)
            ti = tfr - g["start_time"]
            gt_loom = np.interp(ti, g["t_g"], g["loom"])
            gt_alt = np.interp(ti, g["t_g"], g["alt"])
        except Exception as e:
            print(f"[overlay] GT unavailable: {e}")

    # trace y-ranges (robust)
    def rng(a, pad=0.15):
        a = a[np.isfinite(a)]
        if a.size == 0:
            return (-1.0, 1.0)
        lo, hi = np.percentile(a, 2), np.percentile(a, 98)
        d = (hi - lo) * pad + 1e-6
        return (lo - d, hi + d)
    r_lnW = rng(lnW)
    _rr = np.concatenate([rate[np.isfinite(rate)], gt_loom[np.isfinite(gt_loom)]])
    r_rate = rng(_rr if _rr.size else np.array([0.0, 0.0]))

    # ---- pass 2: render ----
    H, W = cv2.imread(frames[0]).shape[:2]
    Hp = 150
    vw = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (W, H + Hp))
    for k, fp in enumerate(frames):
        fr = cv2.imread(fp)
        if fr is None:
            continue
        if fr.ndim == 2:
            fr = cv2.cvtColor(fr, cv2.COLOR_GRAY2BGR)
        det, armA, armB, wv = per[k]

        # faint mask tint
        if det is not None and det.ok and det.isolated_mask is not None:
            m = det.isolated_mask
            if m.shape[:2] == fr.shape[:2]:
                tint = fr.copy(); tint[m > 0] = (60, 40, 40)
                fr = cv2.addWeighted(fr, 0.75, tint, 0.25, 0)

        for arm, col, pts in ((armA, CYAN, getattr(det, "line_points_i", None) if det else None),
                              (armB, YELL, getattr(det, "line_points_j", None) if det else None)):
            if arm is None:
                continue
            c = arm["c"]; v = arm["v"]; nrm = arm["nrm"]
            p1 = (int(c[0] - v[0] * 400), int(c[1] - v[1] * 400))
            p2 = (int(c[0] + v[0] * 400), int(c[1] + v[1] * 400))
            cv2.line(fr, p1, p2, col, 1, cv2.LINE_AA)
            for (p, on, sp, sm, t) in arm["stations"]:
                pc = (int(round(p[0])), int(round(p[1])))
                if on and np.isfinite(t):
                    a1 = (int(round(p[0] - sm * nrm[0])), int(round(p[1] - sm * nrm[1])))
                    a2 = (int(round(p[0] + sp * nrm[0])), int(round(p[1] + sp * nrm[1])))
                    cv2.line(fr, a1, a2, GREEN, 2, cv2.LINE_AA)
                    cv2.circle(fr, pc, 4, (30, 30, 30), -1, cv2.LINE_AA)
                    cv2.circle(fr, pc, 4, WHITE, 1, cv2.LINE_AA)     # station = white ring
                else:
                    cv2.circle(fr, pc, 5, RED, 1, cv2.LINE_AA)       # hollow red = OFF-mask, skipped

        # HUD
        wa = armA["width"] if armA else None
        wb = armB["width"] if armB else None
        ratio = (max(wa, wb) / min(wa, wb)) if (wa and wb) else float("nan")
        hud = [
            (f"f{k}/{nf}  t{tfr[k]-tfr[0]:.1f}s  alt {gt_alt[k]:+.2f}m", WHITE),
            (f"armA {wa:.1f}px   armB {wb:.1f}px" if (wa and wb)
             else f"armA {wa if wa else '--'}  armB {wb if wb else '--'}", WHITE),
            (f"ratio {ratio:.2f}  (reject if >{_WLOOM_ARM_AGREE_RATIO:.0f})"
             if np.isfinite(ratio) else "ratio --", RED if (np.isfinite(ratio) and ratio > _WLOOM_ARM_AGREE_RATIO) else WHITE),
            (f"width = {wv:.1f} px" if wv else "width = HOLD (None)", GREEN if wv else GREY),
            (f"KF loom-rate {rate[k]:+.3f}", GREEN),
            (f"GT loom      {gt_loom[k]:+.3f}", RED),
        ]
        y = 15
        for line, col in hud:
            cv2.putText(fr, line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(fr, line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv2.LINE_AA)
            y += 14

        panel = _panel({
            "ln(width)": (lnW, WHITE, *r_lnW),
            "KF loom-rate  (-d/dt ln w)": (rate, GREEN, *r_rate),
            "GT loom": (gt_loom, RED, *r_rate),
        }, k, W, Hp)
        vw.write(np.vstack([fr, panel]))
    vw.release()
    print(f"[overlay] wrote {args.out}  ({nf} frames)")
    fin = np.isfinite(rate) & np.isfinite(gt_loom)
    if fin.sum() > 8:
        print(f"[overlay] corr(KF loom-rate, GT loom) over whole clip = "
              f"{np.corrcoef(rate[fin], gt_loom[fin])[0,1]:+.2f}")


if __name__ == "__main__":
    main()
