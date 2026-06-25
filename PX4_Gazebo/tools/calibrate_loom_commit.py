#!/usr/bin/env python3
"""Calibrate the loom-accumulation commit threshold (LANDING_COMMIT_LOOM).

Reconstructs the harness accumulator  loom_accum = ∫|h_z| dt  (the SAME quantity
landing_test.py integrates from EC_node.LOOM_Z over fresh frames, from Control_Data
h(t)) and maps it to the actual altitude-above-target + lateral error, so we can
pick a threshold that fires just ABOVE the deck (before the terminal 1/Z kick)
while still centered.

NOTE: Control_Data "t" (perf_counter) and Ground_Truth "Time" (elapsed) are on
DIFFERENT clock origins — align by ELAPSED time (subtract each t0), not by value.
The first-descent bottom is found by freezing once altitude climbs >0.3 m above the
running min (a later off-target second descent can reach a lower GLOBAL min).

Usage:  calibrate_loom_commit.py <rep_dir> [<rep_dir> ...]
"""
import sys, os
import numpy as np

THRS = (2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2)


def analyze(rep_dir):
    cd = np.load(os.path.join(rep_dir, "Control_Data.npy"), allow_pickle=True).item()
    gt = np.load(os.path.join(rep_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    t = np.asarray(cd.get("t", []), float)
    h = np.asarray(cd.get("h(t)", []), float)
    if t.size == 0 or h.size == 0:
        print(f"  {os.path.basename(rep_dir)}: no Control_Data h(t)/t"); return
    t = t - t[0]
    h = h.reshape(len(h), -1); hz = h[:, 2]
    n = min(len(t), len(hz)); t, hz = t[:n], hz[:n]
    dt = np.clip(np.diff(t, prepend=t[0]), 0, 0.2)
    accum = np.cumsum(np.abs(hz) * dt)                 # = harness loom_accum

    gtt = np.asarray(gt.get("Time", []), float)
    uav, tgt = gt["UAV Pose"], gt["Target Pose"]
    m = min(len(uav), len(tgt), len(gtt)) if len(gtt) else min(len(uav), len(tgt))
    alt = np.array([uav[i].position.z - tgt[i].position.z for i in range(m)])
    lat = np.array([((uav[i].position.x - tgt[i].position.x)**2 +
                     (uav[i].position.y - tgt[i].position.y)**2) ** 0.5 for i in range(m)])
    gtt = (gtt[:m] - gtt[0]) if len(gtt) else np.arange(m) / 60.0
    alt_i = np.interp(t, gtt, alt); lat_i = np.interp(t, gtt, lat)

    # First-descent bottom (freeze on >0.3 m climb above running min).
    rmin, bottom = np.inf, 0
    for i in range(len(alt_i)):
        if alt_i[i] < rmin:
            rmin, bottom = alt_i[i], i
        elif alt_i[i] > rmin + 0.3:
            break

    print(f"\n  === {os.path.basename(rep_dir)} ===")
    print(f"  start alt {alt_i[0]:.2f} m | 1st-descent bottom alt {alt_i[bottom]:.3f} m "
          f"lat {lat_i[bottom]:.3f} m @ accum {accum[bottom]:.2f} | "
          f"loom h_z median {np.median(hz[:bottom]):.3f} | total accum {accum[-1]:.2f}")
    print(f"  {'accum_thr':>9} {'fires@alt':>10} {'lat@fire':>9} {'t@fire':>7}")
    for thr in THRS:
        idx = np.argmax(accum >= thr) if (accum >= thr).any() else -1
        if idx > 0:
            print(f"  {thr:>9.2f} {alt_i[idx]:>10.3f} {lat_i[idx]:>9.3f} {t[idx]:>6.1f}s")
        else:
            print(f"  {thr:>9.2f} {'(never)':>10}")


def main(argv):
    if len(argv) < 2:
        print(__doc__); return 2
    for d in argv[1:]:
        if os.path.isdir(d) and os.path.exists(os.path.join(d, "Control_Data.npy")):
            analyze(d)
        else:
            print(f"  skip {d} (no Control_Data.npy)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
