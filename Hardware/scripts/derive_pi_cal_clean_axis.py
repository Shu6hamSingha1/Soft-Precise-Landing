#!/usr/bin/env python3
"""Extract genuinely clean single-axis excitation segments from ALREADY-
RECORDED calibration data (no new recording), using the CO-SAMPLED,
zero-interpolation Ground_Truth.npy arrays, and re-fit centroid slope
(sx/sy) and the corner flow matrix from only those segments, pooled across
however many runs are given.

WHY THIS EXISTS (2026-08-01): derive_one()'s phase_labels()-based fit found
2/6 Aug-1 runs producing a physically-impossible NEGATIVE sx. Root-caused
(see this session's own dig into "Sat Aug 01 21-51-24 2026"): phase_labels()
only requires the labeled axis to carry the PLURALITY of a window's
excitation energy (dom_min=0.45), not that other axes are near-zero - a
window with real simultaneous yaw during an "X-sweep" can still pass that
bar, and such a window's xc-vs-bearing relationship can genuinely INVERT
relative to a pure-X window. User's direction: don't record more data, mine
what's already recorded harder instead.

TWO CHANGES from derive_one()'s approach, both in the direction of more
rigor rather than more data:

1. STRICT PURITY, not just dominance: a window only counts as "clean X" if
   every OTHER axis's own (unnormalized) rate stays below an absolute floor
   for that window's whole duration - not merely smaller than X's own rate.
   This directly targets the yaw-during-translation contamination found.

2. ZERO-INTERPOLATION TIME SYNC: derive_one() re-loads Img_Data.npy and
   resamples it onto the GT clock via linear interpolation (g["align"]).
   output_calibration.py ALSO logs "Raw Img Feature Params"/"Raw Opt Flow
   Ang Vel" directly INTO Ground_Truth.npy, co-sampled on the EXACT SAME
   loop tick as "UAV Pose"/"Target Pose" (img_data.py's
   getRawImgFeatureParam()/getRawOptFlowAngVel(), called back-to-back with
   the mocap poll - see output_calibration.py's main loop). This script
   uses that pairing exclusively - no interpolation, no alignment error by
   construction. Estimator tags (needed to exclude coast/held samples,
   which aren't co-sampled into Ground_Truth.npy) are attached via
   NEAREST-NEIGHBOR match against Img_Data.npy's own Time/tag arrays -
   nearest-neighbor is safe for a discrete/categorical value in a way linear
   interpolation of a continuous signal across a real/coast boundary is not
   (see derive_pi_cal.py's own coast-frame-guard comment on that failure
   mode).

Usage:
    python3 derive_pi_cal_clean_axis.py <run_dir> [<run_dir> ...]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import derive_pi_cal as D

# Absolute purity floors (native units). A window's NON-dominant axes must
# all stay below these for the whole window to count as "clean" for the
# dominant axis - independent of how much bigger the dominant axis's own
# rate is. Env-overridable for experimentation.
PURITY_V_MAX = float(os.environ.get("CLEAN_PURITY_V_MAX", "0.06"))     # m/s, non-dominant translation
PURITY_YAW_MAX = float(os.environ.get("CLEAN_PURITY_YAW_MAX", "0.15"))  # rad/s, yaw during a translation window
WIN_S = float(os.environ.get("CLEAN_WIN_S", "0.5"))
TAG_MATCH_TOL_S = float(os.environ.get("CLEAN_TAG_TOL_S", "0.08"))


def load_cosampled(run_dir):
    """Returns per-sample arrays on the GT's own native (co-sampled) tick,
    plus a nearest-matched S/flow estimator tag for each tick."""
    gt = np.load(os.path.join(run_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt["Start Time"])
    tc = np.asarray(gt["Time"], float)
    u_all, tp_all = gt["UAV Pose"], gt["Target Pose"]
    raw_feat = np.asarray(gt["Raw Img Feature Params"], float)   # [xc,yc,1,alpha], pre-cal
    raw_flow = np.asarray(gt["Raw Opt Flow Ang Vel"], float)     # [h;w], pre-cal
    img_t_rel = gt.get("Img Time Stamp", None)

    n = min(len(tc), len(u_all), len(tp_all), len(raw_feat), len(raw_flow))
    tc = tc[:n]; u_all = u_all[:n]; tp_all = tp_all[:n]
    raw_feat = raw_feat[:n]; raw_flow = raw_flow[:n]
    img_t_rel = np.asarray(img_t_rel[:n], float) if img_t_rel is not None else None

    def _valid(p, t):
        if p is None or t is None:
            return False
        vals = [p.x, p.y, p.z, p.roll, p.pitch, p.yaw, t.x, t.y, t.z]
        return all(np.isfinite(v) for v in vals)
    keep = np.array([_valid(u_all[i], tp_all[i]) for i in range(n)])
    idx = np.where(keep)[0]
    tc = tc[idx]; u = [u_all[i] for i in idx]; tp = [tp_all[i] for i in idx]
    raw_feat = raw_feat[idx]; raw_flow = raw_flow[idx]
    img_t_rel = img_t_rel[idx] if img_t_rel is not None else None
    n = len(tc)
    if n < 20:
        raise ValueError(f"too few valid co-sampled ticks: {n}")

    order = np.hstack(([True], np.diff(tc) > 1e-6))
    tc = tc[order]; u = [u[i] for i in range(len(u)) if order[i]]
    tp = [tp[i] for i in range(len(tp)) if order[i]]
    raw_feat = raw_feat[order]; raw_flow = raw_flow[order]
    img_t_rel = img_t_rel[order] if img_t_rel is not None else None
    n = len(tc)

    # Geometry per sample - direct port of compute_gt_flow's own per-i loop
    # (lever arms, FLU->FRD, yaw), just operating on the co-sampled index
    # directly instead of a separately-resampled GT grid.
    W_x_tu = np.zeros((n, 3)); Ru = np.zeros((n, 3, 3)); yaw = np.zeros(n)
    roll = np.zeros(n); pitch = np.zeros(n)
    for i in range(n):
        p, t = u[i], tp[i]
        up, Ru[i] = D._pose_to_frd(p)
        tpp, Rt = D._pose_to_frd(t)
        W_x_tu[i] = (tpp + Rt @ D.R_MARKER_FRD) - (up + Ru[i] @ D.R_CAM_FRD)
        yaw[i] = -np.deg2rad(p.yaw - t.yaw)
        roll[i] = np.deg2rad(p.roll)
        # FIXED 2026-08-01 (matches derive_pi_cal.py's compute_gt_flow fix) -
        # empirically NOT flipped, unlike yaw: FC's own quaternion-derived
        # pitch matches unflipped mocap pitch (corr +0.99), the earlier
        # by-analogy-to-yaw flip gave corr -0.99 (anti-correlated - the sign
        # was wrong, not a real sensor disagreement).
        pitch[i] = np.deg2rad(p.pitch)

    W_v_tu = D._robust_vel(W_x_tu, tc)
    V_h_g = np.full((n, 3), np.nan); V_s_g = np.full((n, 2), np.nan)
    for i in range(n):
        zB = W_x_tu[i, 2]
        if abs(zB) < 0.1:
            continue
        B_x = Ru[i].T @ W_x_tu[i]
        V_x = D._v_frame(Ru[i]) @ B_x
        V_s_g[i] = [V_x[0] / zB, V_x[1] / zB]
        B_v = Ru[i].T @ W_v_tu[i]
        V_v = D._v_frame(Ru[i]) @ B_v
        V_h_g[i] = V_v / zB

    yaw_rate = D._robust_vel(np.unwrap(yaw)[:, None], tc)[:, 0]
    roll_rate = D._robust_vel(np.unwrap(roll)[:, None], tc)[:, 0]
    pitch_rate = D._robust_vel(np.unwrap(pitch)[:, None], tc)[:, 0]

    # Nearest-neighbor tag match (discrete value - safe unlike interpolation).
    s_tag = np.array([''] * n, dtype=object)
    flow_tag = np.array([''] * n, dtype=object)
    s_settled = np.zeros(n, dtype=bool)   # True = post-reacquisition transient, exclude
    img_path = os.path.join(run_dir, "Img_Data.npy")
    if img_t_rel is not None and os.path.exists(img_path):
        img = np.load(img_path, allow_pickle=True).item()
        t_img_abs = np.asarray(img["Time"], float)
        s_tag_full = np.array([str(x) for x in img.get("S Estimator Tag", [''] * len(t_img_abs))])
        flow_tag_full = np.array([str(x) for x in img.get("Opt Flow Estimator Tag", [''] * len(t_img_abs))])
        m = min(len(t_img_abs), len(s_tag_full), len(flow_tag_full))
        t_img_abs = t_img_abs[:m]; s_tag_full = s_tag_full[:m]; flow_tag_full = flow_tag_full[:m]

        # REACQUISITION SETTLE GUARD (2026-08-01) - same root cause as
        # derive_pi_cal.py's own settle guard: the first SETTLE_N real-tagged
        # samples right after a coast run are a transient (raw xc jumps
        # 0.15-0.29 while mocap GT stays smooth through the same instant -
        # confirmed on "Sat Aug 01 21-51-24 2026"'s wrong-signed sx), computed
        # HERE on Img_Data.npy's own native tag sequence (not the co-sampled
        # one, where a single native tag repeats across many co-sampled ticks
        # via nearest-match and would make an edge-count-based guard meaningless).
        s_is_real_native = np.isin(s_tag_full, list(D.CORNER_S_TAGS))
        _edge = np.diff(s_is_real_native.astype(int), prepend=0) == 1
        settled_native = np.zeros(m, dtype=bool)
        for _i in np.where(_edge)[0]:
            settled_native[_i:_i + D.SETTLE_N] = True

        img_t_abs = img_t_rel + St
        j = np.clip(np.searchsorted(t_img_abs, img_t_abs), 0, m - 1)
        j_lo = np.clip(j - 1, 0, m - 1)
        pick = np.where(np.abs(t_img_abs[j] - img_t_abs) <= np.abs(t_img_abs[j_lo] - img_t_abs), j, j_lo)
        dt = np.abs(t_img_abs[pick] - img_t_abs)
        ok = dt <= TAG_MATCH_TOL_S
        s_tag[ok] = s_tag_full[pick[ok]]
        flow_tag[ok] = flow_tag_full[pick[ok]]
        s_settled[ok] = settled_native[pick[ok]]

    return dict(t=tc, V_h_g=V_h_g, V_s_g=V_s_g, raw_feat=raw_feat, raw_flow=raw_flow,
                yaw_rate=yaw_rate, roll_rate=roll_rate, pitch_rate=pitch_rate, s_settled=s_settled,
                s_tag=s_tag, flow_tag=flow_tag)


def clean_axis_mask(d):
    """Per-sample dominant-axis label (0=X,1=Y,2=Z,3=yaw,-1=none) using a
    STRICT purity gate: the window's non-dominant axes must be below an
    ABSOLUTE floor (PURITY_V_MAX / PURITY_YAW_MAX), not just smaller than
    the dominant one. Returns int array, same length as d['t']."""
    t = d["t"]
    # V_h_g is h=v/z (not raw v) - fine for purity gating, which only needs
    # "is this axis near-zero", not absolute physical velocity units.
    vx, vy, vz = d["V_h_g"][:, 0], d["V_h_g"][:, 1], d["V_h_g"][:, 2]
    yaw_rate = d["yaw_rate"]

    lab = np.full(len(t), -1, dtype=int)
    if len(t) < 4:
        return lab
    edges = np.arange(t[0], t[-1], WIN_S)
    for a, b in zip(edges[:-1], edges[1:]):
        w = (t >= a) & (t < b)
        if w.sum() < 3:
            continue
        # translational purity uses V_h_g's own units (h=v/z, dimensionless
        # rate) - PURITY_V_MAX is interpreted in the same units here, not
        # literal m/s (V_h_g isn't m/s); still a meaningful absolute floor
        # since it's the SAME quantity phase_labels/derive_one ultimately fit.
        e = {0: np.nanmedian(np.abs(vx[w])), 1: np.nanmedian(np.abs(vy[w])),
             2: np.nanmedian(np.abs(vz[w])), 3: np.nanmedian(np.abs(yaw_rate[w])) * D.PHASE_YAW_SCALE}
        if not all(np.isfinite(list(e.values()))):
            continue
        dom = max(e, key=e.get)
        # Every OTHER axis must sit below ITS OWN absolute purity floor
        # (translational axes vs PURITY_V_MAX, yaw - already pre-scaled by
        # PHASE_YAW_SCALE in `e` above - vs the matching scaled yaw floor),
        # regardless of how much bigger the dominant axis's own rate is -
        # this is the actual purity requirement, not just relative dominance.
        floors = {0: PURITY_V_MAX, 1: PURITY_V_MAX, 2: PURITY_V_MAX,
                  3: PURITY_YAW_MAX * D.PHASE_YAW_SCALE}
        ok = all(e[k] < floors[k] for k in e if k != dom)
        # Require: dominant axis clearly excited (not just "least small"),
        # AND every other axis at/near its own noise floor.
        if e[dom] > 2 * PURITY_V_MAX and ok:
            lab[w] = dom
    return lab


def main():
    run_dirs = sys.argv[1:]
    if not run_dirs:
        print(__doc__)
        sys.exit(1)

    pooled = {0: {"x": [], "y": []}, 1: {"x": [], "y": []}}   # X, Y phases for sx, sy
    pooled_flow_R, pooled_flow_G = [], []

    for run_dir in run_dirs:
        name = os.path.basename(run_dir)
        try:
            d = load_cosampled(run_dir)
        except Exception as e:
            print(f"  skip {name}: {e}")
            continue
        lab = clean_axis_mask(d)
        s_real = np.isin(d["s_tag"], list(D.CORNER_S_TAGS)) & ~d["s_settled"]
        flow_real = np.isin(d["flow_tag"], list(D.CORNER_FLOW_TAGS))
        finite_s = np.all(np.isfinite(d["V_s_g"]), 1) & np.all(np.isfinite(d["raw_feat"][:, :2]), 1)
        finite_flow = np.all(np.isfinite(d["V_h_g"]), 1) & np.all(np.isfinite(d["raw_flow"]), 1)

        for axis, col, key in ((0, 0, "x"), (1, 1, "y")):
            sel = (lab == axis) & s_real & finite_s
            n = int(sel.sum())
            print(f"  {name}: clean axis-{axis} REAL samples = {n}")
            if n > 20:
                pooled[axis][key].append((d["raw_feat"][sel, col], d["V_s_g"][sel, col]))

        selF = (lab >= 0) & flow_real & finite_s & finite_flow
        if selF.sum() > 20:
            G = np.column_stack([d["V_h_g"], d["roll_rate"], d["pitch_rate"], d["yaw_rate"]])[selF]
            pooled_flow_R.append(d["raw_flow"][selF])
            pooled_flow_G.append(G)

    print("\n=== CENTROID SLOPE from pooled clean single-axis segments ===")
    for axis, key, lab_name in ((0, "x", "sx"), (1, "y", "sy")):
        chunks = pooled[axis][key]
        if not chunks:
            print(f"  {lab_name}: no usable clean segments across all runs")
            continue
        xs = np.concatenate([c[0] for c in chunks])
        ys = np.concatenate([c[1] for c in chunks])
        slope = float(np.polyfit(xs, ys, 1)[0])
        corr = float(np.corrcoef(xs, ys)[0, 1]) if np.std(xs) > 1e-9 else float("nan")
        print(f"  {lab_name} = {slope:+.4f}  (n={len(xs)} pooled samples, corr={corr:.3f})"
              f"{'  [STILL NEGATIVE]' if slope <= 0 else ''}")

    print("\n=== CORNER FLOW MATRIX from pooled clean segments ===")
    if pooled_flow_R:
        R = np.concatenate(pooled_flow_R); G = np.concatenate(pooled_flow_G)
        Msol, _, _, _ = np.linalg.lstsq(R, G, rcond=None)
        cal = Msol.T
        pred = R @ Msol
        r2 = 1 - np.sum((G - pred) ** 2, 0) / np.sum((G - G.mean(0)) ** 2, 0)
        print(f"  n={len(R)} pooled samples")
        print("  per-axis R^2: " + "  ".join(f"{D.LAB[k]}={r2[k]:.2f}" for k in range(6)))
        print(cal)
    else:
        print("  no usable clean segments across all runs")


if __name__ == "__main__":
    main()
