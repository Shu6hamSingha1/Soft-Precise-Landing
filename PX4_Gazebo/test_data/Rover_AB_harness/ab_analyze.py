#!/usr/bin/env python3
"""Extract terminal-fly-away metrics per rep for the world A/B."""
import sys, os, glob, numpy as np
sys.path.insert(0, '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/src')

def metrics(d):
    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    up = gt['UAV Pose']; tp = gt['Target Pose']
    z = np.array([p.position.z for p in up], float)            # ENU up (alt)
    x = np.array([p.position.x for p in up], float)
    y = np.array([p.position.y for p in up], float)
    tx = np.array([p.position.x for p in tp], float)
    ty = np.array([p.position.y for p in tp], float)
    n = min(len(z), len(tx))
    z, x, y, tx, ty = z[:n], x[:n], y[:n], tx[:n], ty[:n]
    lat = np.hypot(x - tx, y - ty)                              # lateral err to target
    imin = int(np.argmin(z))
    # fly-away peak AFTER the first-descent bottom
    max_after = float(z[imin:].max()) if imin < n - 1 else float(z[imin])
    return dict(min_alt=float(z[imin]), lat_at_min=float(lat[imin]),
                max_alt=float(z.max()), max_after_min=max_after,
                final_lat=float(lat[-1]), flew=(z.max() > 15.0))

def arm(name, base):
    dirs = sorted(glob.glob(os.path.join(base, '*/')))
    print(f"\n=== ARM {name}  ({len(dirs)} reps) ===")
    rows = []
    for d in dirs:
        try:
            m = metrics(d)
            rows.append(m)
            print(f"  min_alt={m['min_alt']:.2f}m  lat@min={m['lat_at_min']:.2f}m  "
                  f"PEAK_alt={m['max_alt']:.1f}m  final_lat={m['final_lat']:.1f}m  "
                  f"{'FLY-AWAY' if m['flew'] else 'bounded'}")
        except Exception as e:
            print(f"  [skip {os.path.basename(d.rstrip('/'))}: {e}]")
    if rows:
        pk = np.array([r['max_alt'] for r in rows])
        ma = np.array([r['min_alt'] for r in rows])
        nf = sum(r['flew'] for r in rows)
        print(f"  SUMMARY: fly-aways {nf}/{len(rows)}  peak_alt med={np.median(pk):.1f}m "
              f"[{pk.min():.1f},{pk.max():.1f}]  min_alt med={np.median(ma):.2f}m")
    return rows

base = '/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad'
arm('ARUCO (stationary)', os.path.join(base, 'ab_aruco'))
arm('ROVER (stationary)',  os.path.join(base, 'ab_rover'))
