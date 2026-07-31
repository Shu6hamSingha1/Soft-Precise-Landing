#!/usr/bin/env python3
"""Certify the camera/marker lever arms against a set of recordings.

Standing tool (2026-07-28) - this check was rebuilt from scratch three times
in one session as a throwaway script and deleted after each use. Made
permanent so the isolated-translation recording (see
project_pi_gt_lever_arm_2026_07_27 / project_pi_cal_regime_mismatch) can be
checked with one command instead of hand-written each time.

Compares GT-predicted V-frame bearing against 'Virtual Feature Pts' (the
CORRECT comparand - V-frame, tilt already removed). An earlier pass used raw
'Image Feature Pts' by mistake, which still contains camera tilt and gave
misleading sign/magnitude results - do not go back to that array for this
check.

Usage:
    python3 check_lever_arm.py <run_dir> [<run_dir> ...]

Reports, using the CURRENT R_CAM_FRD/R_MARKER_FRD from derive_pi_cal.py:
  1. residual bias/scatter at the values currently in code
  2. a sign search over cam_x/cam_y/mrk_x/mrk_y (cam_z excluded - it is
     structurally degenerate with bearing scale, see derive_pi_cal.py's own
     comment on R_CAM_FRD; don't add it back here)
  3. a 1-D magnitude profile per component, to see whether the data's own
     minimum agrees with what is currently applied
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import derive_pi_cal as D

CACHE = []


def prep(run_dirs):
    for r in run_dirs:
        try:
            gt = np.load(os.path.join(r, "Ground_Truth.npy"), allow_pickle=True).item()
            img = np.load(os.path.join(r, "Img_Data.npy"), allow_pickle=True).item()
        except Exception as e:
            print(f"  SKIP {r}: {e}")
            continue
        tgv = np.asarray(gt["Time"], float)
        U, T = list(gt["UAV Pose"]), list(gt["Target Pose"])
        St = float(gt["Start Time"])
        n = min(len(tgv), len(U), len(T))
        up = np.full((n, 3), np.nan); Ru = np.full((n, 3, 3), np.nan)
        tp = np.full((n, 3), np.nan); Rt = np.full((n, 3, 3), np.nan)
        Vf = np.full((n, 3, 3), np.nan)
        for i in range(n):
            try:
                a, b = D._pose_to_frd(U[i]); c, e = D._pose_to_frd(T[i])
            except Exception:
                continue
            if not np.all(np.isfinite(b)):
                continue
            up[i], Ru[i], tp[i], Rt[i] = a, b, c, e
            try:
                Vf[i] = D._v_frame(b)
            except Exception:
                pass
        ok = (np.all(np.isfinite(up), 1) & np.all(np.isfinite(Vf.reshape(n, 9)), 1)
              & np.all(np.isfinite(Rt.reshape(n, 9)), 1))
        if ok.sum() < 200:
            print(f"  SKIP {r}: too few valid pose samples ({int(ok.sum())})")
            continue
        tags = np.array([str(x) for x in img["Opt Flow Estimator Tag"]])
        ti = np.asarray(img["Time"], float)
        # 'Virtual Feature Pts' - V-frame, tilt removed. NOT 'Image Feature Pts'.
        VIRT = np.asarray(img["Virtual Feature Pts"], float)[:, 0].mean(1)
        IP = np.asarray(img["Image Feature Pts"], float)[:, 0]
        side = np.linalg.norm(IP - np.roll(IP, -1, axis=1), axis=2).mean(1)
        m = (tags != "coast") & (side > 80) & np.all(np.isfinite(VIRT), 1)
        if m.sum() < 50:
            print(f"  SKIP {r}: too few tracked big-marker samples ({int(m.sum())})")
            continue
        CACHE.append(dict(t=tgv[:n][ok], up=up[ok], Ru=Ru[ok], tp=tp[ok], Rt=Rt[ok],
                          Vf=Vf[ok], St=St, ti=ti[m], v=VIRT[m]))
    print(f"  prepared {len(CACHE)}/{len(run_dirs)} run(s)")


def bias_scatter(cam, mrk):
    """Per-run median bias (mean over runs, magnitude not averaged pre-abs)
    and per-run scatter, both in V-frame normalised units."""
    bx, by, sc = [], [], []
    for c in CACHE:
        W = (c["tp"] + c["Rt"] @ mrk) - (c["up"] + c["Ru"] @ cam)
        B = np.einsum('nji,nj->ni', c["Ru"], W)
        V = np.einsum('nij,nj->ni', c["Vf"], B)
        den = V[:, 2]
        g = den > 0.5
        if g.sum() < 100:
            continue
        t = c["t"][g]
        sx = np.interp(c["ti"] - c["St"], t, V[g, 0] / den[g])
        sy = np.interp(c["ti"] - c["St"], t, V[g, 1] / den[g])
        ex = sx - c["v"][:, 0]; ey = sy - c["v"][:, 1]
        bx.append(np.median(ex)); by.append(np.median(ey))
        sc.append(np.median(np.hypot(ex - np.median(ex), ey - np.median(ey))))
    if not bx:
        return np.nan, np.nan
    return float(np.hypot(np.mean(bx), np.mean(by))), float(np.mean(sc))


def main():
    run_dirs = sys.argv[1:]
    if not run_dirs:
        print(__doc__)
        sys.exit(1)
    prep(run_dirs)
    if not CACHE:
        print("No usable runs."); sys.exit(1)

    cam0, mrk0 = D.R_CAM_FRD, D.R_MARKER_FRD
    b0, s0 = bias_scatter(cam0, mrk0)
    bz, sz = bias_scatter(np.zeros(3), np.zeros(3))
    print(f"\ncurrent code   : cam={cam0}  mrk={mrk0}")
    print(f"  bias={b0:.4f}  scatter={s0:.4f}   (no lever arm: bias={bz:.4f}  scatter={sz:.4f})\n")

    print("=" * 70)
    print("PER-COMPONENT PROFILE (cam_z NOT scanned - structurally degenerate,")
    print("see derive_pi_cal.py's R_CAM_FRD comment)")
    print("=" * 70)
    print("Each axis scanned independently through BOTH signs and a range of")
    print("magnitudes. This subsumes a separate sign check: if 'data min' has the")
    print("OPPOSITE sign from 'current', that axis's sign is in question, not just")
    print("its magnitude. (A whole-vector sign-flip table was tried and REMOVED -")
    print("it silently breaks when a vector has mixed-sign components, since a")
    print("single scalar flip can't test one axis independently of the others.)")
    print(f"{'component':<10} {'current':>9} {'data min':>10} {'bias@current':>13} {'bias@min':>10}")
    print("-" * 70)
    for label, idx, which in [("cam x", 0, "cam"), ("cam y", 1, "cam"),
                              ("mrk x", 0, "mrk"), ("mrk y", 1, "mrk")]:
        cur = (cam0 if which == "cam" else mrk0)[idx]
        grid = np.round(np.arange(cur - 0.20, cur + 0.2001, 0.02), 4)
        vals = []
        for g in grid:
            cam = cam0.copy(); mrk = mrk0.copy()
            (cam if which == "cam" else mrk)[idx] = g
            vals.append(bias_scatter(cam, mrk)[0])
        vals = np.array(vals)
        best_g = grid[np.nanargmin(vals)]
        at_cur = vals[np.nanargmin(np.abs(grid - cur))]
        railed = best_g <= grid[1] or best_g >= grid[-2]
        flag = " (RAILING)" if railed else ""
        print(f"{label:<10} {cur:>+9.3f} {best_g:>+10.3f}{flag:<11} {at_cur:>13.4f} "
              f"{np.nanmin(vals):>10.4f}")
    print("-" * 70)


if __name__ == "__main__":
    main()
