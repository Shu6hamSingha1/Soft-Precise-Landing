#!/usr/bin/env python3
"""CBF-behaviour analysis for the rover CBF sweep (scripts/run_rover_cbf_sweep.sh).

Scores the visibility CBF on TRIGGER CORRECTNESS + moving-target-lead EFFECT, not
landing precision. Per (trajectory, arm):

  in_frame_%    fraction of control frames with a live marker detection (vis_c != 0)
  last_seen_%   last marker-present frame, as % of flight  (higher = kept in frame longer)
  drift_off     did img flag _last_drifted_off at any point (marker left FoV off-centre)
  maxC/phi      worst |vis_c|/phi over the flight  (>1 = left the buffered box;
                >1/(1-buf) = left the physical sensor)
  act_%         fraction of frames the Tier-1 QP modified a_xy (vis_active)
  t_first_act   first vis_active frame as % of flight  (lead should fire EARLIER)
  |d|_mean/max  vis_drift magnitude (tangent/s) -- is the h_xy-sourced lead sane / non-zero
  slack_n       frames with vis_slack>1e-6 (FoV vs thrust-ball conflict), + max
  gz_min        Tier-2 descent-ease floor reached
  flight_s      recording length / 60

The lead arm (tau>0) is judged BETTER when, vs its tau=0 twin: last_seen_% higher,
maxC/phi lower, drift_off less, t_first_act earlier, with |d| non-trivial and no
new slack/instability.
"""
import os
import sys
import glob

import numpy as np

BUF = float(os.environ.get("CBF_BUFFER_FRAC", "0.15"))


def _load(d):
    cd = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
    try:
        idd = np.load(os.path.join(d, "Img_Data.npy"), allow_pickle=True).item()
    except Exception:
        idd = {}
    return cd, idd


def _arr(cd, k):
    v = cd.get(k)
    return None if v is None else np.asarray(v, float)


def score(d):
    cd, idd = _load(d)
    c = _arr(cd, "vis_c(t)")
    if c is None or len(c) == 0:
        return None
    n = len(c)
    fs = n / 60.0
    act = _arr(cd, "vis_active(t)")
    sl = _arr(cd, "vis_slack(t)")
    dr = _arr(cd, "vis_drift(t)")
    gz = _arr(cd, "vis_gz(t)")

    present = np.linalg.norm(c, axis=1) > 1e-9
    in_frame = float(np.mean(present))
    last_seen = (float(np.max(np.where(present)[0]) / n) if present.any() else 0.0)

    # phi in tangent units. Hardcoded for the current camera: img_data rotates the
    # frame CW so the detection frame is 240w x 320h, center = _resolution/2 =
    # (120, 160), focal = (135, 135). (Img_Data["Center Px"] logs all-nan -- do
    # not use it.) Update these two lines if the camera changes.
    ctr = np.array([120.0, 160.0])
    foc = np.array([135.0, 135.0])
    phi = (ctr / foc) * (1.0 - BUF)
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.abs(c[present]) / phi if present.any() else np.zeros((1, 2))
    maxC = float(np.nanmax(ratio)) if ratio.size else 0.0

    drift_off = bool(np.any(idd.get("_drift_off_hist", idd.get("drift_off_hist", [False]))))

    act_pct = float(np.mean(act > 0.5)) if act is not None and len(act) else 0.0
    t_first = (float(np.where(act > 0.5)[0][0] / n)
               if act is not None and np.any(act > 0.5) else np.nan)
    dmean = float(np.mean(dr[dr > 0])) if dr is not None and np.any(dr > 0) else 0.0
    dmax = float(np.max(dr)) if dr is not None and len(dr) else 0.0
    slack_n = int(np.sum(sl > 1e-6)) if sl is not None else 0
    slack_max = float(np.max(sl)) if sl is not None and len(sl) else 0.0
    gz_min = float(np.min(gz)) if gz is not None and len(gz) else 1.0

    return dict(flight_s=fs, in_frame=in_frame, last_seen=last_seen, drift_off=drift_off,
                maxC=maxC, act_pct=act_pct, t_first=t_first, dmean=dmean, dmax=dmax,
                slack_n=slack_n, slack_max=slack_max, gz_min=gz_min)


def main():
    bundle = sys.argv[1]
    rows = {}
    for tsv_dir in sorted(glob.glob(os.path.join(bundle, "*", "*", "rep*"))):
        parts = tsv_dir.split(os.sep)
        traj, arm = parts[-3], parts[-2]
        s = score(tsv_dir)
        if s is None:
            continue
        rows.setdefault((traj, arm), []).append(s)

    hdr = (f"{'traj':12} {'arm':5} {'n':>2} {'flt_s':>6} {'inFrm%':>7} {'lastSn%':>8} "
           f"{'drift_off':>9} {'maxC/phi':>9} {'act%':>6} {'t1stAct%':>9} "
           f"{'|d|mean':>8} {'|d|max':>7} {'slk_n':>6} {'slk_max':>8} {'gz_min':>7}")
    print(hdr)
    print("-" * len(hdr))
    for traj in ("Static", "Linear", "Circular", "EightShape", "Sinusoidal", "Lissajous", "CircularYaw"):
        for arm in ("off", "lead"):
            r = rows.get((traj, arm))
            if not r:
                continue
            g = lambda k: np.nanmean([x[k] for x in r])
            print(f"{traj:12} {arm:5} {len(r):>2} {g('flight_s'):6.1f} "
                  f"{100*g('in_frame'):7.1f} {100*g('last_seen'):8.1f} "
                  f"{sum(x['drift_off'] for x in r):>4}/{len(r):<4} "
                  f"{g('maxC'):9.2f} {100*g('act_pct'):6.1f} "
                  f"{100*g('t_first'):9.1f} {g('dmean'):8.4f} {g('dmax'):7.4f} "
                  f"{np.mean([x['slack_n'] for x in r]):6.1f} {g('slack_max'):8.4f} {g('gz_min'):7.3f}")
        print()

    print("READ: lead (tau>0) is doing its job vs its off twin when -> lastSn% higher, "
          "maxC/phi lower, drift_off fewer, t1stAct% earlier, with |d| non-trivial and "
          "no new slack. If |d|mean~0 on a moving profile the h_xy source or the flow-"
          "validity gate is the problem, not the QP.")


if __name__ == "__main__":
    main()
