#!/usr/bin/env python3
"""Scan recorded reps for the visibility CBF's OWN guarantee: does the marker
centre stay inside the buffered safe set phi, and inside the PHYSICAL sensor?

This is the scan behind the 2026-09-17 audit block in
Memory/px4/project_20260909_visibility_projection_wire_in.md -- kept in-tree so
those per-arm numbers can be re-derived by anyone.

Reads `vis_c(t)` (marker-centre image tangent, module frame) plus the
`vis_active/slack/gz/drift` logs from each rep's Control_Data.npy. Frames where
`vis_c` is exactly (0,0) are "no marker this frame" and are excluded.

Judged per the CBF-behaviour criteria (feedback_dont_judge_cbf_by_sp), NOT by
landing outcome:
  exPHYS : frames with |c| past the physical half-extent (see AXIS ORDER) -- must be ~0
  exPHI  : frames past the buffered set phi = R/(2f)*(1-b)       -- buffer regime
  act%   : Tier-1 activation rate      slkmax : peak Tier-1 slack
  gzmin  : deepest Tier-2 descent scale              dmed/dmax : |d| fed to tau*d

NOTE the duration confound: a rep that dies at 2 s has few frames and few exits.
Compare arms at matched flight durations (e.g. --min-frames 400) before reading
any per-arm total as an effect.

Usage:
  scan_vis_safeset.py 'test_data/ICValidation/20260912-040029/*'
  scan_vis_safeset.py --min-frames 400 'test_data/RoverCBFSweep/*/*/off/*'
"""
import sys
import os
import glob
import numpy as np

# 320x240 sensor AFTER the cv2.ROTATE_90_CW detection rotation -> 240 wide x 320 tall,
# so center = (cx, cy) = (120, 160); f = 135 px (CLAUDE.md "Camera", verified 2026-09-02).
CENTER = np.array([120.0, 160.0])
FOCAL = np.array([135.0, 135.0])

# AXIS ORDER -- the trap this tool got wrong in v1 (2026-09-17), see
# Obsolete/tools/scan_vis_safeset_v1_transposed_phi.py.
# marker_tangent() applies _SWAP = [[0,1],[-1,0]] to (px - center)/focal, so
#     c[0] = +(y_px - cy)/f   -> spans +-cy/f = +-1.185   (the 320-tall axis)
#     c[1] = -(x_px - cx)/f   -> spans +-cx/f = +-0.889   (the 240-wide axis)
# The physical half-extent IN c's OWN AXIS ORDER is therefore (cy, cx)/f, i.e. CENTER
# REVERSED. Using CENTER/focal directly (as src/visibility_projection.fov_limit() does)
# transposes the box against the measurement: it over-tightens axis 0 by 36% and leaves
# axis 1's barrier (1.007) OUTSIDE the physical edge (0.889), i.e. inert.
PHI_PHYS = CENTER[::-1] / FOCAL        # physical half-extent in c's axis order
BUFFER_FRAC = 0.15                     # CBF_BUFFER_FRAC default
PHI = PHI_PHYS * (1.0 - BUFFER_FRAC)


def rep_metrics(d):
    f = os.path.join(d, "Control_Data.npy")
    if not os.path.exists(f):
        return None
    try:
        c = np.load(f, allow_pickle=True).item()
    except Exception:
        return None
    if "vis_c(t)" not in c:
        return None                                   # pre-28e4417b rep, no vis_c log
    vc = np.asarray(c["vis_c(t)"], float)
    if vc.ndim != 2 or len(vc) < 20:
        return None
    live = np.any(vc != 0.0, axis=1)
    if live.sum() < 20:
        return None
    v = vc[live]
    act = np.asarray(c.get("vis_active(t)", []), float)
    slk = np.asarray(c.get("vis_slack(t)", []), float)
    gz = np.asarray(c.get("vis_gz(t)", []), float)
    drift = np.asarray(c.get("vis_drift(t)", []), float)
    r_phi = np.max(np.abs(v) / PHI, axis=1)
    r_phys = np.max(np.abs(v) / PHI_PHYS, axis=1)
    return dict(
        n=int(live.sum()),
        exits_phys=int((r_phys > 1.0).sum()),
        exits_phi=int((r_phi > 1.0).sum()),
        max_rphi=float(r_phi.max()),
        act=float(np.mean(act)) if act.size else np.nan,
        slk_max=float(np.max(slk)) if slk.size else np.nan,
        gz_min=float(np.min(gz)) if gz.size else np.nan,
        d_med=float(np.median(drift)) if drift.size else np.nan,
        d_max=float(np.max(drift)) if drift.size else np.nan,
    )


def main(argv):
    min_frames = 0
    pats = []
    it = iter(argv)
    for a in it:
        if a == "--min-frames":
            min_frames = int(next(it))
        else:
            pats.append(a)
    if not pats:
        print(__doc__)
        return 1
    rows = []
    for p in pats:
        for d in sorted(glob.glob(p)):
            if not os.path.isdir(d):
                continue
            m = rep_metrics(d)
            if m and m["n"] >= min_frames:
                rows.append((d, m))
    if not rows:
        print("no reps with vis_c(t) logs matched")
        return 1
    print(f"{'rep':<62} {'n':>5} {'exPHYS':>6} {'exPHI':>6} {'maxC/phi':>8} "
          f"{'act%':>6} {'slkmax':>7} {'gzmin':>6} {'dmed':>6} {'dmax':>6}")
    tot = dict(n=0, exits_phys=0, exits_phi=0)
    for d, m in rows:
        print(f"{d[-62:]:<62} {m['n']:>5} {m['exits_phys']:>6} {m['exits_phi']:>6} "
              f"{m['max_rphi']:>8.2f} {100 * m['act']:>6.1f} {m['slk_max']:>7.3f} "
              f"{m['gz_min']:>6.2f} {m['d_med']:>6.3f} {m['d_max']:>6.3f}")
        for k in tot:
            tot[k] += m[k]
    print(f"\nTOTAL reps={len(rows)} frames={tot['n']} "
          f"sensor-exit={tot['exits_phys']} ({100 * tot['exits_phys'] / tot['n']:.2f}%) "
          f"buffered-set-exit={tot['exits_phi']} ({100 * tot['exits_phi'] / tot['n']:.2f}%)"
          + (f"   [--min-frames {min_frames}]" if min_frames else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
