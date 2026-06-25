#!/usr/bin/env python3
"""Validate a baked-config landing bundle: per-rep validity gate, landing
outcome (xy/vel/TL), and OUTER-FUNNEL RESIDENCY (r=s_e_n/p_s).

Designed for the n=5 validation-debt runs on the IC=2 lateral gain-chain
stack (commit c5f9989). Consumes a `run_knob_sweep.sh` bundle layout:
    <bundle>/<timestamp>/<group>/rep*/{Ground_Truth,Control_Data,Control_Params}.npy

VALIDITY RULE (a rep is REJECTED — excluded from stats, needs a re-run — if):
    * empty GT trajectory
    * flight_s < MIN_FLIGHT_S (truncated / buggy-run artifact, e.g. the f194989
      NameError that froze flights at the first zero-corner frame)
    * GT ends > MAX_FINAL_ALT_M above the deck (hover / balloon / fly-away that
      never touched down — the "fake SOFT hovering at 5 m" artifact)

FUNNEL RESIDENCY: r_axis(t) = |s_e_n_axis(t)| / p_s_axis(t). r<1 = inside the
outer position funnel; r->1 = ratio saturation -> G_s^-1 collapse -> demand
starvation (the IC=2 gain-chain mechanism). Reports per-axis max-r and the
fraction of the flight spent inside (r<1) and near-breach (r>0.9).

Usage:
    analyze_baked_validation.py <bundle_dir> [<bundle_dir> ...]
        (bundle_dir = either the .../BakedStack_IC2 root or a specific timestamp)
"""
import sys, os, glob
import numpy as np

MIN_FLIGHT_S    = 2.0    # below this -> truncated artifact
MAX_FINAL_ALT_M = 0.25   # above this -> never reached the deck
GT_RATE_HZ      = 60.0   # UAV Pose sample rate (matches run_ic_validation/run_knob_sweep)


def _final_alt(uav_pose):
    """ENU z (up) of the last GT pose = altitude above deck."""
    try:
        return float(uav_pose[-1].position.z)
    except Exception:
        # array-of-array fallback
        return float(np.asarray(uav_pose[-1]).ravel()[2])


def _reconstruct_p_s(t, params):
    """Fallback for pre-2026-06-12 logs that lack p_s(t): rebuild the analytic
    funnel envelope from Control_Params. p_s(t)=exp(-gamma_s*(t-t0))*(p0-pinf)+pinf."""
    gs = np.asarray(params["gamma_s"])
    g = np.diag(gs) if gs.ndim == 2 else np.asarray(gs)
    p0  = np.asarray(params["p_s_0"]).ravel()
    pin = np.asarray(params["p_s_inf"]).ravel()
    t = np.asarray(t, float)
    trel = t - t[0]
    return np.exp(-np.outer(trel, g)) * (p0 - pin) + pin   # (T,2)


def _residency(rep_dir):
    """Return dict of per-axis funnel-residency stats, or None if unavailable."""
    cd_p = os.path.join(rep_dir, "Control_Data.npy")
    if not os.path.exists(cd_p):
        return None
    cd = np.load(cd_p, allow_pickle=True).item()
    s_e_n = np.asarray(cd.get("s_e_n(t)", []), float)
    if s_e_n.size == 0:
        return None
    s_e_n = s_e_n.reshape(len(s_e_n), -1)[:, :2]

    p_s = cd.get("p_s(t)", [])
    p_s = np.asarray(p_s, float) if len(p_s) else np.empty((0,))
    if p_s.size == 0:                                   # fallback: reconstruct
        cp = np.load(os.path.join(rep_dir, "Control_Params.npy"), allow_pickle=True).item()
        t = cd.get("t", [])
        if not len(t):
            return None
        p_s = _reconstruct_p_s(t, cp)
    p_s = p_s.reshape(len(p_s), -1)[:, :2]

    n = min(len(s_e_n), len(p_s))
    if n == 0:
        return None
    r = np.abs(s_e_n[:n]) / np.clip(p_s[:n], 1e-9, None)   # (n,2)
    out = {}
    for i, ax in enumerate("xy"):
        ri = r[:, i]
        out[ax] = dict(rmax=float(np.nanmax(ri)),
                       inside=float(np.mean(ri < 1.0) * 100),
                       near=float(np.mean(ri > 0.9) * 100))
    out["p_s_final"] = [float(p_s[n-1, 0]), float(p_s[n-1, 1])]
    return out


def analyze_rep(rep_dir):
    gt = np.load(os.path.join(rep_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    sp = gt.get("SoftPrecise", {}) or {}
    uav = gt.get("UAV Pose", [])
    rec = dict(name=os.path.basename(rep_dir), valid=True, reason="",
               flight_s=0.0, final_alt=np.nan,
               xy=float(sp.get("xy_err", np.nan)), vel=float(sp.get("rel_vel", np.nan)),
               # Honest precision @ lowest altitude (before any terminal balloon).
               xy_minz=float(sp.get("min_alt_xy", np.nan)),
               vel_minz=float(sp.get("min_alt_rel_vel", np.nan)),
               minz=float(sp.get("min_alt", np.nan)),
               tl=bool(sp.get("target_lost", False)),
               tpl=bool(sp.get("terminal_perception_loss", False)),
               soft=bool(sp.get("soft", False)), prec=bool(sp.get("precise", False)),
               res=None)
    if uav is None or len(uav) == 0:
        rec.update(valid=False, reason="empty GT")
        return rec
    rec["flight_s"] = len(uav) / GT_RATE_HZ
    rec["final_alt"] = _final_alt(uav)
    if rec["flight_s"] < MIN_FLIGHT_S:
        rec.update(valid=False, reason=f"flight_s {rec['flight_s']:.1f}<{MIN_FLIGHT_S}")
    elif rec["final_alt"] > MAX_FINAL_ALT_M:
        rec.update(valid=False, reason=f"final_alt {rec['final_alt']:.2f}>{MAX_FINAL_ALT_M}")
    rec["res"] = _residency(rep_dir)
    return rec


def find_reps(bundle):
    reps = sorted(glob.glob(os.path.join(bundle, "**", "*rep*"), recursive=True))
    return [r for r in reps if os.path.isdir(r) and os.path.exists(os.path.join(r, "Ground_Truth.npy"))]


def main(argv):
    if len(argv) < 2:
        print(__doc__); return 2
    for bundle in argv[1:]:
        reps = find_reps(bundle)
        print(f"\n===== {bundle}  ({len(reps)} reps) =====")
        if not reps:
            print("  (no rep*/Ground_Truth.npy found)"); continue
        hdr = (f"  {'rep':<22} {'valid':<6} {'flt_s':>6} {'alt':>6} "
               f"{'xy':>7} {'vel':>6} {'xy@mnZ':>7} {'v@mnZ':>6} {'mnZ':>5} {'TL':>3}  "
               f"{'r_x':>5} {'in%':>4} {'r_y':>5} {'in%':>4}  reason")
        print(hdr); print("  " + "-" * (len(hdr) - 2))
        recs = [analyze_rep(r) for r in reps]
        for r in recs:
            res = r["res"]
            rx = f"{res['x']['rmax']:>5.2f}" if res else "   - "
            ry = f"{res['y']['rmax']:>5.2f}" if res else "   - "
            ix = f"{res['x']['inside']:>4.0f}" if res else "   -"
            iy = f"{res['y']['inside']:>4.0f}" if res else "   -"
            print(f"  {r['name']:<22} {'Y' if r['valid'] else 'N':<6} "
                  f"{r['flight_s']:>6.1f} {r['final_alt']:>6.2f} "
                  f"{r['xy']:>7.3f} {r['vel']:>6.3f} "
                  f"{r['xy_minz']:>7.3f} {r['vel_minz']:>6.3f} {r['minz']:>5.2f} "
                  f"{('Y' if r['tl'] else ''):>3}  "
                  f"{rx} {ix} {ry} {iy}  {r['reason']}")

        valid = [r for r in recs if r["valid"]]
        landed = [r for r in valid if not r["tl"]]
        tl = [r for r in valid if r["tl"]]
        print(f"\n  VALID {len(valid)}/{len(recs)}  (rejected {len(recs)-len(valid)}: "
              + ", ".join(f"{r['name']}={r['reason']}" for r in recs if not r["valid"]) + ")")
        if valid:
            xy = np.array([r["xy"] for r in landed]) if landed else np.array([np.nan])
            ve = np.array([r["vel"] for r in landed]) if landed else np.array([np.nan])
            nsp = sum(r["soft"] and r["prec"] for r in landed)
            print(f"  landed {len(landed)}/{len(valid)} | TL {len(tl)} | SP {nsp}")
            print(f"  xy(landed,endpoint): mean {np.nanmean(xy):.3f}  min {np.nanmin(xy):.3f}  "
                  f"max {np.nanmax(xy):.3f}   vel: mean {np.nanmean(ve):.3f}")
            # Honest precision: lateral error at the lowest altitude reached
            # (over ALL valid reps incl. the "flys" — the kick balloons the
            # endpoint but they touch down dead-centered; this is the real metric).
            xymz = np.array([r["xy_minz"] for r in valid])
            vemz = np.array([r["vel_minz"] for r in valid])
            if np.isfinite(xymz).any():
                print(f"  xy@min-alt (HONEST precision, all valid): "
                      f"mean {np.nanmean(xymz):.3f}  min {np.nanmin(xymz):.3f}  "
                      f"max {np.nanmax(xymz):.3f}   v@min-alt: mean {np.nanmean(vemz):.3f}  "
                      f"max {np.nanmax(vemz):.3f}")
                print(f"  dead-centered @ touchdown (xy@min-alt ≤ 0.10 m): "
                      f"{int(np.nansum(xymz <= 0.10))}/{int(np.isfinite(xymz).sum())}")
            rr = [r["res"] for r in valid if r["res"]]
            if rr:
                for ax in "xy":
                    rmax = np.array([d[ax]["rmax"] for d in rr])
                    ins  = np.array([d[ax]["inside"] for d in rr])
                    print(f"  funnel r_{ax}: max {np.nanmean(rmax):.2f} (worst {np.nanmax(rmax):.2f}) | "
                          f"inside funnel {np.nanmean(ins):.0f}% of flight"
                          + ("  [BREACH]" if np.nanmax(rmax) >= 1.0 else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
