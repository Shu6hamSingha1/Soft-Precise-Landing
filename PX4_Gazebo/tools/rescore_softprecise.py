#!/usr/bin/env python3
"""Re-score archived landings against the SoftPrecise TERMINAL-STATE GATE (2026-09-02).

WHY. `apps/landing_test.py` used to read `xy_err` from the pose at whatever instant
the control loop exited -- its "PX4 reports LANDED here" comment was never enforced.
A touchdown-detect latch, a timeout or an abort all exit that loop too, so the
drone's MID-AIR pose got scored as a landing. Archive audit 2026-09-02: 159 of 1192
`precise` verdicts (13.4%) were computed mid-air. `landing_test.py` now gates on a
verified terminal state, but archived runs cannot be re-flown -- this re-scores them.

NON-DESTRUCTIVE: reads `Ground_Truth.npy`, never writes to it. Emits a CSV plus a
summary; `--json` also dumps the raw per-run rows.

THE GATE (both thresholds validated on all 4446 archived runs -- 159 unsupported vs
1033 legitimate `precise`):
  (a) LOWEST altitude above the LANDING SURFACE <= --touchdown-alt (0.20 m)
      Catches all 159, 0 false positives.
  (b) surface-agnostic backstop: min_alt <= --descent-frac (0.15) * start_alt
      Catches 45/159, 0/1033 false positives; still fires if (a) is misconfigured.
REJECTED after measuring (do not re-add): a |dz/dt| terminal-rest check catches
152/159 but rejects 58% of LEGITIMATE landings; an IMU contact-spike gate does not
separate at all (27.0% vs 26.7%).

⚠ SURFACE HEIGHT. The floor is relative to the LANDING SURFACE, not the target-pose
origin. On `rover_cross` the 0.4 m platform means a real landing sits ~0.48 m above
the rover's own pose origin -- scoring those against a 0.20 m flat floor wrongly
condemns 34 genuine landings. Live runs get this from PLASMC_GT_MARKER_DZ (LANDING_SURFACE_DZ overrides), but the
world is NOT recorded in Ground_Truth.npy, so for archived data this tool applies a
HEURISTIC: median target-pose z > --rover-tz => --rover-surface-dz. Override with
--surface-dz to force one value for a directory you know the world of.

USAGE
  python3 tools/rescore_softprecise.py test_data
  python3 tools/rescore_softprecise.py test_data/ICValidation --csv /tmp/out.csv
  python3 tools/rescore_softprecise.py test_data/Final --surface-dz 0.0 -v
"""
import argparse, csv, glob, json, os, sys, warnings
import numpy as np
warnings.filterwarnings("ignore")


def score_run(run_dir, surface_dz=None, rover_tz=0.01, rover_dz=0.50,
              touchdown_alt=0.20, descent_frac=0.15):
    """Re-score one run dir. Returns a row dict, or None if unreadable."""
    try:
        gt = np.load(os.path.join(run_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    except Exception:
        return None
    up, tp, T = gt.get("UAV Pose"), gt.get("Target Pose"), gt.get("Time")
    if up is None or tp is None or T is None:
        return None
    T = np.asarray(T, float)
    n = min(len(up), len(tp), len(T))
    if n < 10:
        return None
    rel = np.array([up[i].position.z - tp[i].position.z for i in range(n)], float)
    tz = float(np.median([tp[i].position.z for i in range(n)]))
    dz = rover_dz if (surface_dz is None and tz > rover_tz) else (surface_dz or 0.0)
    alt_end, alt_min, alt_0 = rel[-1] - dz, rel.min() - dz, rel[0] - dz

    # Gate on the LOWEST altitude reached, not the endpoint: a run that touches at
    # 0.05 m and then balloons to 0.5 m DID land (its endpoint xy is post-kick --
    # a separate, pre-existing issue that `min_alt_xy` already exists to expose).
    # alt_min is also the criterion the 0-false-positive thresholds were validated on.
    reason = None
    if alt_min > touchdown_alt:
        reason = f"never reached surface: min {alt_min:.2f} m above it"
    elif alt_0 > 0 and alt_min > descent_frac * alt_0:
        reason = f"never descended: min {alt_min:.2f} m of {alt_0:.2f} m start"

    sp = gt.get("SoftPrecise") or {}
    was_p, was_s = bool(sp.get("precise", False)), bool(sp.get("soft", False))
    return dict(path=run_dir, world="rover" if tz > rover_tz else "flat", surface_dz=dz,
                stored_precise=was_p, stored_soft=was_s,
                rescored_precise=(was_p and reason is None),
                rescored_soft=(was_s and reason is None),
                terminal_state_ok=(reason is None), not_landed_reason=reason or "",
                xy_err=float(sp.get("xy_err", np.nan)),
                alt_end=float(alt_end), alt_min=float(alt_min), alt_start=float(alt_0),
                dur=float(T[-1] - T[0]))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", help="directory to walk for */Ground_Truth.npy")
    ap.add_argument("--csv", default=None, help="write per-run rows here")
    ap.add_argument("--json", default=None, help="also dump raw rows as JSON")
    ap.add_argument("--surface-dz", type=float, default=None,
                    help="force the landing-surface height above the target pose origin "
                         "(m). Omit to use the rover heuristic.")
    ap.add_argument("--rover-tz", type=float, default=0.01,
                    help="median target-pose z above this => treat as rover world (0.01)")
    ap.add_argument("--rover-surface-dz", type=float, default=0.50,
                    help="surface height for rover-world runs (0.50 = PLASMC_GT_MARKER_DZ)")
    ap.add_argument("--touchdown-alt", type=float, default=0.20,
                    help="max altitude above surface that still counts as landed (0.20)")
    ap.add_argument("--descent-frac", type=float, default=0.15,
                    help="min_alt must be <= this fraction of start alt (0.15)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="list every run whose verdict changes")
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.root, "**", "Ground_Truth.npy"), recursive=True))
    if not files:
        sys.exit(f"no Ground_Truth.npy under {a.root}")
    rows = [r for r in (score_run(os.path.dirname(f), a.surface_dz, a.rover_tz,
                                  a.rover_surface_dz, a.touchdown_alt, a.descent_frac)
                        for f in files) if r]

    stored = [r for r in rows if r["stored_precise"]]
    lost = [r for r in stored if not r["terminal_state_ok"]]
    print(f"\nre-scored {len(rows)} runs under {a.root}"
          f"   (rover-heuristic: {sum(1 for r in rows if r['world']=='rover')} rover / "
          f"{sum(1 for r in rows if r['world']=='flat')} flat)")
    print(f"  stored   precise : {len(stored)}")
    print(f"  rescored precise : {len(stored)-len(lost)}"
          f"   ({len(lost)} lose the flag = "
          f"{100*len(lost)/max(len(stored),1):.1f}%)")
    if lost:
        nd = sum(1 for r in lost if "never descended" in r["not_landed_reason"])
        print(f"     never reached the surface (gate a) : {len(lost)-nd}")
        print(f"     never descended    (gate b backstop): {nd}")
        print(f"     LOWEST altitude above surface: median "
              f"{np.median([r['alt_min'] for r in lost]):.3f} m, "
              f"max {max(r['alt_min'] for r in lost):.3f} m")
    if a.verbose:
        for r in sorted(lost, key=lambda r: -r["alt_end"]):
            print(f"    {r['alt_end']:7.3f} m  xy={r['xy_err']:.3f}  {r['path']}"
                  f"   [{r['not_landed_reason']}]")
    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)
        print(f"  wrote {a.csv}")
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=1)
        print(f"  wrote {a.json}")


if __name__ == "__main__":
    main()
