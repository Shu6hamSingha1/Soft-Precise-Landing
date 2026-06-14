#!/usr/bin/env python3
"""Score a CBF A/B bundle by TARGET VISIBILITY — the CBF's actual job — not by
landing xy/fly-away (a control-tuning outcome the CBF cannot fix).

Per rep, from Img_Data.npy: marker track-alive (N Flow Corners>0), marker-in-FoV
(all 4 corners on the sensor), and the same conditioned on body tilt (the regime
where the visibility barrier is supposed to act). Aggregates per arm.

Usage: analyze_cbf_visibility.py [bundle_dir]   (default: latest test_data/CBF_AB/*)
"""
import sys, os, glob
import numpy as np

CX, CY, W, H = 320.0, 240.0, 640.0, 480.0     # 640x480 down-cam (CLAUDE.md)
HI_TILT_DEG = 8.0                              # "tilt regime" threshold

def rep_metrics(d):
    im = np.load(os.path.join(d, "Img_Data.npy"), allow_pickle=True).item()
    fp = np.asarray(im["Image Feature Pts"])          # (N,2,4,2)
    if fp.ndim != 4 or len(fp) < 30:
        return None
    cur = fp[:, 1, :, :]                              # (N,4,2) current corners
    cen = cur.mean(1)                                 # (N,2) centroid px
    ncorn = np.asarray(im["N Flow Corners"])
    q = np.asarray(im["Quat"]); R33 = 1 - 2*(q[:,1]**2 + q[:,2]**2)
    tilt = np.degrees(np.arccos(np.clip(R33, -1, 1)))
    alive = ncorn > 0                                 # marker tracked/decoded this frame
    inframe = np.all((cur[:,:,0] >= 0) & (cur[:,:,0] <= W) &
                     (cur[:,:,1] >= 0) & (cur[:,:,1] <= H), axis=1)
    vis = alive & inframe                             # genuinely visible (tracked + on sensor)
    # post-acquisition window: from first alive frame to end
    if not alive.any():
        return None
    a0 = np.argmax(alive)
    sl = slice(a0, None)
    fps = float(np.nanmedian(np.asarray(im["FPS"]))) if "FPS" in im else 60.0
    fps = fps if fps > 1 else 60.0
    tl = tilt[sl]; vs = vis[sl]; al = alive[sl]
    hi = tl > HI_TILT_DEG
    # time-to-first-loss after acquisition (s)
    lost = np.argmax(~vs) if (~vs).any() else len(vs)
    return dict(
        n=len(vs),
        alive_frac=float(al.mean()),
        vis_frac=float(vs.mean()),
        t_vis=float(vs.sum()/fps),
        mean_tilt=float(tl.mean()), max_tilt=float(tl.max()),
        hitilt_vis=float(vs[hi].mean()) if hi.any() else np.nan,
        hitilt_n=int(hi.sum()),
        t_to_loss=float(lost/fps),
    )

def main():
    base = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(
        os.path.join(os.path.dirname(__file__), "..", "test_data", "CBF_AB", "*")))[-1]
    print(f"bundle: {base}\n")
    arms = {}
    for arm_dir in sorted(glob.glob(os.path.join(base, "*"))):
        if not os.path.isdir(arm_dir): continue
        arm = os.path.basename(arm_dir)
        rows = []
        for rep in sorted(glob.glob(os.path.join(arm_dir, "rep*"))):
            if not os.path.exists(os.path.join(rep, "Img_Data.npy")): continue
            try:
                m = rep_metrics(rep)
                if m: rows.append(m)
            except Exception as e:
                print(f"  ! {arm}/{os.path.basename(rep)}: {e}")
        if rows: arms[arm] = rows
    if not arms:
        print("no reps with Img_Data yet."); return
    cols = ["alive_frac","vis_frac","t_vis","mean_tilt","max_tilt","hitilt_vis","t_to_loss"]
    print(f"  {'arm':<10} {'n':>2}  " + " ".join(f"{c:>10}" for c in cols))
    print(f"  {'':<10} {'':>2}  " + " ".join(f"{'(mean)':>10}" for _ in cols))
    order = ["cbf_old","A_only","B_only","cbf_fixed"]
    for arm in [a for a in order if a in arms] + [a for a in arms if a not in order]:
        rows = arms[arm]
        vals = [np.nanmean([r[c] for r in rows]) for c in cols]
        print(f"  {arm:<10} {len(rows):>2}  " + " ".join(f"{v:>10.3f}" for v in vals))
    print("\n  alive_frac = marker tracked (N corners>0); vis_frac = tracked AND all 4 corners on sensor")
    print(f"  hitilt_vis = vis_frac among frames with tilt>{HI_TILT_DEG:.0f}° (the regime the CBF acts in)")
    print("  t_vis/t_to_loss in seconds. Higher alive/vis/hitilt/t_vis = better visibility (the CBF's job).")

if __name__ == "__main__":
    main()
