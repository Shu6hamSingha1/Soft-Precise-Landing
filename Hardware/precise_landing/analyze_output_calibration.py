"""Diagnostic/sanity-check analyzer for output_calibration.py recordings on
the Pi hardware pipeline (Test_Data/Calibration/<timestamp>/).

*** This does NOT derive the sensor-calibration matrix (_sensor_cal_hw). ***
The Gazebo SITL tools/derive_board_cal.py does that via compute_gt_signals(),
which is tightly coupled to Gazebo's simulated Pose objects (ENU frame,
quaternion orientation) and phased (x/y/z/yaw-tagged) recordings - neither
of which apply directly here:
  1. output_calibration.py records CONTINUOUS hand-move data with no phase
     tagging (no 'Phase' key in Ground_Truth), unlike the SITL phased maneuver.
  2. Qualisys (QTM) mocap's coordinate-frame convention relative to NED has
     NOT been confirmed (flagged during this session's mocaptools.py audit -
     needs on-site verification against the QTM lab's calibrated axes).
Porting derive_board_cal.py's GT-derivation math without that confirmation
risks a silently-wrong sensor calibration - a real derivation should be built
carefully in a followup session once the frame convention is confirmed.

What THIS script does instead: per-run and aggregate diagnostics -
excitation coverage per axis, raw flow/feature sample counts, and basic
plots - so you can judge run quality and prep before that derivation work.

Usage: python3 analyze_output_calibration.py [cal_dir]
"""
import os
import sys

import numpy as np

CAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Test_Data", "Calibration")


def analyze_run(run_dir):
    try:
        gt = np.load(f"{run_dir}/Ground_Truth.npy", allow_pickle=True).item()
        img = np.load(f"{run_dir}/Img_Data.npy", allow_pickle=True).item()
    except FileNotFoundError:
        return None

    flow = gt.get("Opt Flow Ang Vel", [])
    feat = gt.get("Img Feature Params", [])
    up = [p for p in gt.get("UAV Pose", []) if p is not None]
    tp = [p for p in gt.get("Target Pose", []) if p is not None]
    t = gt.get("Time", [0.0])
    dur = float(t[-1]) if len(t) else 0.0

    xs = np.array([p.x for p in up]) if up else np.array([])
    ys = np.array([p.y for p in up]) if up else np.array([])
    zs = np.array([p.z for p in up]) if up else np.array([])
    yaws = np.array([])
    if up:
        try:
            # Drop NaN (QTM tracking dropouts) BEFORE unwrap: np.unwrap is a
            # cumulative running correction over consecutive-sample deltas -
            # a single NaN corrupts everything after it, silently collapsing
            # the reported span to near-zero from a handful of leftover
            # finite values (confirmed 2026-07-10: 157.7deg true span
            # reported as 1.2deg with NaN left in).
            _raw_yaws = np.array([p.yaw for p in up])
            yaws = np.unwrap(np.deg2rad(_raw_yaws[~np.isnan(_raw_yaws)]))
        except AttributeError:
            pass

    def span(a):
        return float(np.nanmax(a) - np.nanmin(a)) if len(a) else 0.0

    return dict(
        dir=os.path.basename(run_dir),
        duration=dur,
        flow_n=len(flow),
        feat_n=len(feat),
        x_span=span(xs), y_span=span(ys), z_span=span(zs),
        yaw_span_deg=float(np.rad2deg(span(yaws))) if len(yaws) else 0.0,
        n_uav_samples=len(up), n_target_samples=len(tp),
    )


def main():
    cal_dir = sys.argv[1] if len(sys.argv) > 1 else CAL_DIR
    if not os.path.isdir(cal_dir):
        print(f"No such directory: {cal_dir}")
        sys.exit(1)
    runs = sorted(d for d in os.listdir(cal_dir)
                  if os.path.isdir(os.path.join(cal_dir, d)) and d != "Test_Videos"
                  and d != "Input")
    if not runs:
        print(f"No recordings under {cal_dir}")
        sys.exit(1)

    print(f"[output-cal] {len(runs)} run(s) under {cal_dir}\n")
    results = []
    for d in runs:
        r = analyze_run(os.path.join(cal_dir, d))
        if r is None:
            print(f"  SKIP {d} (missing Ground_Truth.npy / Img_Data.npy)")
            continue
        results.append(r)

    if not results:
        print("No usable runs found.")
        return

    hdr = f'  {"run":32s} {"dur(s)":>7s} {"flow_n":>7s} {"X":>6s} {"Y":>6s} {"Z":>6s} {"yaw":>6s}  verdict'
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    weak_runs = 0
    for r in results:
        checks = [("flow", r["flow_n"], 300), ("X", r["x_span"], 0.40),
                  ("Y", r["y_span"], 0.40), ("Z", r["z_span"], 0.40),
                  ("yaw", r["yaw_span_deg"], 20.0)]
        weak = [n for n, v, lo in checks if v < lo]
        verdict = "WEAK: " + ",".join(weak) if weak else "GOOD"
        if weak:
            weak_runs += 1
        print(f'  {r["dir"]:32s} {r["duration"]:7.1f} {r["flow_n"]:7d} '
              f'{r["x_span"]:6.2f} {r["y_span"]:6.2f} {r["z_span"]:6.2f} '
              f'{r["yaw_span_deg"]:6.1f}  {verdict}')

    print(f"\n  {len(results)} run(s) analyzed, {weak_runs} flagged WEAK "
          f"(excitation below threshold on >=1 axis).")
    print("  Thresholds: flow>=300 samples, X/Y/Z span>=0.40m, yaw span>=20deg")
    print(f"\n  Recommendation: need >=5 GOOD runs with well-separated axis sweeps")
    print("  before attempting a sensor-cal derivation (per project convention).")
    if weak_runs > 0:
        print(f"  Consider re-recording the {weak_runs} WEAK run(s) with more vigorous,")
        print("  axis-separated hand-moves (move ONE axis at a time, not simultaneously).")


if __name__ == "__main__":
    main()
