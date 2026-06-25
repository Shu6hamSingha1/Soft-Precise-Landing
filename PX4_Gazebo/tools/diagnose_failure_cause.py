#!/usr/bin/env python3
"""Per-rep ROOT-CAUSE diagnosis for a PLASMC landing failure.

Beyond the explosion-chain first-mover, this classifies WHY the divergence
started, by contrasting the image error against Gazebo truth at the onset:

  PERCEPTION  : image s_e_n large while GT lateral offset small  -> centroid is
                wrong (marker drift/jump/partial-loss), OR corners dropped.
  CONTROL     : GT lateral offset large too -> the drone is physically off and
                the loop can't correct it (lag-limited lateral servo).
  CONE-CLAMP  : theta_cone collapsed to theta_current near onset -> the FoV cone
                strangled the lateral correction (d_min collapse).
  TERMINAL    : marker went stale/lost in the final <0.5 s at high extent
                (touchdown perception loss) -> last-glimpse / open-loop drift.

Usage: diagnose_failure_cause.py <rep_dir> [<rep_dir> ...]
"""
import sys, os, numpy as np

def _arr(x):
    try: return np.asarray(x, dtype=float)
    except Exception: return np.asarray(x)

def load(d):
    g = lambda f: np.load(os.path.join(d, f), allow_pickle=True).item()
    return g("Control_Data.npy"), g("Img_Data.npy"), g("Ground_Truth.npy")

def diagnose(d):
    cd, img, gt = load(d)
    t = _arr(cd["t"]); t = t - t[0]
    sen = _arr(cd["s_e_n(t)"])            # (N,2) normalized image error x,y
    dsd = _arr(cd["ds_d(t)"])
    kap = _arr(cd["kappa(t)"])
    auv = _arr(cd["a_u(t)"])
    thc = _arr(cd["theta_cone(t)"]); thcur = _arr(cd["theta_current(t)"])
    sp  = gt.get("SoftPrecise", {}) or {}
    xy_end = sp.get("xy_err", float('nan')); v_end = sp.get("rel_vel", float('nan'))
    tl = bool(sp.get("target_lost", False)); tpl = bool(sp.get("terminal_perception_loss", False))

    # GT lateral offset (drone - target), ENU xy, over time
    uav = gt.get("UAV Pose", []); tgt = gt.get("Target Pose", gt.get("target", []))
    def xy_of(p):  # robust to dict/obj
        try: return np.array([p.position.x, p.position.y])
        except Exception:
            try: return np.array([p["position"]["x"], p["position"]["y"]])
            except Exception: return np.array([np.nan, np.nan])
    gt_lat = None
    try:
        U = np.array([xy_of(p) for p in uav])
        if len(tgt) == len(uav):
            T = np.array([xy_of(p) for p in tgt]); gt_lat = np.linalg.norm(U - T, axis=1)
        else:
            gt_lat = np.linalg.norm(U - U[-1], axis=1)  # vs final (target≈landing pt)
    except Exception:
        gt_lat = None

    # perception health
    ncorn = _arr(img.get("N Flow Corners", []))
    fps   = _arr(img.get("FPS", []))

    m = len(t)
    # divergence onset = first time |s_e_n| (either axis) > 0.5
    sen_mag = np.abs(sen[:m]).max(axis=1) if sen.ndim == 2 else np.abs(sen[:m])
    onset_idx = next((i for i in range(m) if sen_mag[i] > 0.5), None)
    if onset_idx is None:  # no s_e_n breach; use peak ds_d time
        onset_idx = int(np.nanargmax(np.abs(dsd[:m]).max(axis=1))) if dsd.ndim==2 else int(np.nanargmax(np.abs(dsd[:m])))
    t_on = t[onset_idx]; t_end = t[m-1]
    axis = int(np.argmax(np.abs(sen[onset_idx]))) if sen.ndim==2 else 0
    sen_on = float(sen_mag[onset_idx])

    # GT lateral at onset (truth)
    gt_lat_on = float('nan')
    if gt_lat is not None and len(gt_lat):
        gi = min(int(onset_idx * len(gt_lat) / m), len(gt_lat)-1)
        gt_lat_on = float(gt_lat[gi])

    # cone clamp: theta_cone within ~2deg of theta_current near onset?
    cone_strangle = False
    try:
        w = slice(max(0,onset_idx-10), min(m,onset_idx+5))
        cone_strangle = bool(np.nanmedian(thc[w] - thcur[w]) < np.deg2rad(3.0))
    except Exception: pass

    # perception drop near onset
    corn_drop = False
    if len(ncorn):
        ci = min(int(onset_idx*len(ncorn)/m), len(ncorn)-1)
        base = np.nanmedian(ncorn[max(0,ci-30):ci]) if ci>5 else np.nanmedian(ncorn)
        corn_drop = bool(ncorn[ci] < 0.6*base) if base>0 else False

    # ---- classify ----
    reasons = []
    if tl: reasons.append("TARGET_LOST (marker gone > grace)")
    if tpl or (t_end - t_on) < 0.6:
        reasons.append("TERMINAL: divergence in final %.2fs" % (t_end - t_on))
    if corn_drop:
        reasons.append("PERCEPTION: corner count dropped at onset")
    # perception vs control discriminant
    if not np.isnan(gt_lat_on):
        if sen_on > 0.4 and gt_lat_on < 0.5:
            reasons.append("PERCEPTION-led: image err %.2f but GT lateral only %.2fm (centroid wrong)" % (sen_on, gt_lat_on))
        elif gt_lat_on >= 0.5:
            reasons.append("CONTROL-led: GT lateral %.2fm at onset (real physical drift, lag-limited)" % gt_lat_on)
    if cone_strangle:
        reasons.append("CONE-CLAMP: theta_cone~theta_current at onset (FoV cone strangling lateral)")

    print(f"--- {os.path.basename(os.path.dirname(os.path.dirname(d)))}/{os.path.basename(d)} ---")
    print(f"  result: xy_end={xy_end:.2f}m vel_end={v_end:.2f}m/s  T_flight={t_end:.1f}s")
    print(f"  onset: t={t_on:.2f}s (axis={'x' if axis==0 else 'y'})  |s_e_n|={sen_on:.2f}  GT_lat={gt_lat_on:.2f}m"
          f"  kappa_end={np.round(kap[m-1],2) if kap.ndim==1 else np.round(kap[m-1],2)}  a_u_pk={np.round(np.nanmax(np.abs(auv[:m]),axis=0),0) if auv.ndim==2 else round(float(np.nanmax(np.abs(auv[:m]))),0)}")
    print("  ROOT CAUSE: " + (" | ".join(reasons) if reasons else "inconclusive (no clear onset signal)"))
    return reasons

if __name__ == "__main__":
    for d in sys.argv[1:]:
        try: diagnose(d)
        except Exception as e:
            print(f"--- {d} ---\n  [error] {type(e).__name__}: {e}")
