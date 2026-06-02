#!/usr/bin/env python3
"""Scan every SITL landing bundle under ~/Soft-Precise-Landing/PX4_Gazebo/test_data/ and classify every
rep. Identifies SP landings, PRECISE-only, SOFT-only, and the distribution
of failures. Then for each SP/near-SP we extract diagnostic signals so we
can answer "what was different about this rep?"
"""
from __future__ import annotations
import glob
import json
import os
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np


def _to_array(seq):
    return np.asarray([np.asarray(x) for x in seq])


def classify_rep(rep_dir: Path) -> dict | None:
    gt_path = rep_dir / "Ground_Truth.npy"
    if not gt_path.exists():
        return None
    try:
        gt = np.load(gt_path, allow_pickle=True).item()
    except Exception:
        return None
    sp = gt.get("SoftPrecise", {})
    if not sp:
        return None
    xy = float(sp.get("xy_err", np.nan))
    vel = float(sp.get("rel_vel", np.nan))
    tl = bool(sp.get("target_lost", False))
    prec = bool(sp.get("precise", False))
    soft = bool(sp.get("soft", False))
    # Classification taxonomy
    sp_full = prec and soft and not tl
    if np.isnan(xy):
        cat = "NO_DATA"
    elif sp_full:
        cat = "SP"
    elif tl:
        cat = "TARGET_LOST"
    elif prec and not soft:
        cat = "PRECISE_only"
    elif soft and not prec:
        cat = "SOFT_only"
    elif xy < 0.15 and vel < 0.4:
        cat = "near_SP"
    elif xy < 0.3:
        cat = "MOD_PRECISE"
    elif xy >= 1.0:
        cat = "CATASTROPHIC"
    else:
        cat = "POOR"
    out = {
        "rep_dir": str(rep_dir),
        "rep_name": rep_dir.name,
        "bundle": rep_dir.parent.name,
        "xy_err": xy,
        "rel_vel": vel,
        "target_lost": tl,
        "precise": prec,
        "soft": soft,
        "sp_full": sp_full,
        "category": cat,
    }
    # Try to extract IC + diagnostic signals for interesting cases
    if cat in {"SP", "PRECISE_only", "SOFT_only", "near_SP"}:
        try:
            cd = np.load(rep_dir / "Control_Data.npy", allow_pickle=True).item()
            td = np.load(rep_dir / "Telemetry_Data.npy", allow_pickle=True).item()
            sigma = _to_array(cd["sigma(t)"])
            kappa = _to_array(cd["kappa(t)"])
            t = np.asarray(cd["t"], dtype=float)
            out["flight_s"] = float(t[-1] - t[0])
            out["sigma_xy0"] = float(np.linalg.norm(sigma[0, :2]))
            out["sigma_xy_tail"] = float(np.linalg.norm(
                sigma[-min(20, len(sigma)):, :2], axis=1).mean())
            out["kappa_tail"] = float(np.linalg.norm(kappa[-1]))
            v = td["Velocity Body"][0]
            out["vh0"] = float(np.hypot(v.x_m_s, v.y_m_s))
            out["vz0"] = float(v.z_m_s)
        except Exception as e:
            out["extract_err"] = str(e)
    return out


def main():
    roots = [
        os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo/test_data"),
    ]
    all_reps = []
    bundles_scanned = []
    for root in roots:
        if not os.path.isdir(root):
            continue
        # Each entry under root is a sweep type (e.g. DefaultN10/, BigSensitivity/)
        for sweep_dir in sorted(Path(root).iterdir()):
            if not sweep_dir.is_dir():
                continue
            # Each entry is a timestamped bundle
            for bundle_dir in sorted(sweep_dir.iterdir()):
                if not bundle_dir.is_dir():
                    continue
                # Each entry under bundle is a rep (or cell × rep)
                rep_dirs_in_bundle = []
                for entry in bundle_dir.iterdir():
                    if not entry.is_dir():
                        continue
                    if (entry / "Ground_Truth.npy").exists():
                        rep_dirs_in_bundle.append(entry)
                    else:
                        # Maybe one level deeper (BigSensitivity has cell/rep layout)
                        for sub in entry.iterdir():
                            if sub.is_dir() and (sub / "Ground_Truth.npy").exists():
                                rep_dirs_in_bundle.append(sub)
                if rep_dirs_in_bundle:
                    bundles_scanned.append({
                        "path": str(bundle_dir),
                        "n_reps": len(rep_dirs_in_bundle),
                        "sweep": sweep_dir.name,
                    })
                    for rd in rep_dirs_in_bundle:
                        r = classify_rep(rd)
                        if r:
                            r["sweep"] = sweep_dir.name
                            all_reps.append(r)
    print(f"Scanned {len(bundles_scanned)} bundles across {len(roots)} roots.")
    print(f"Found {len(all_reps)} reps total.")
    print()

    # Per-sweep category breakdown
    print("=" * 92)
    print("PER-SWEEP CATEGORY COUNTS")
    print("=" * 92)
    by_sweep = defaultdict(Counter)
    for r in all_reps:
        by_sweep[r["sweep"]][r["category"]] += 1
    sweep_keys = sorted(by_sweep.keys())
    categories = ["SP", "PRECISE_only", "SOFT_only", "near_SP",
                  "MOD_PRECISE", "POOR", "CATASTROPHIC", "TARGET_LOST",
                  "NO_DATA"]
    print(f"{'sweep':<30}  " + "  ".join(f"{c[:7]:>7}" for c in categories) + "  total")
    for sw in sweep_keys:
        counts = by_sweep[sw]
        total = sum(counts.values())
        print(f"{sw[:30]:<30}  " + "  ".join(f"{counts[c]:>7}" for c in categories) + f"  {total:>5}")

    # Overall counts
    overall = Counter(r["category"] for r in all_reps)
    print()
    print("=" * 92)
    print("OVERALL")
    print("=" * 92)
    total = sum(overall.values())
    for c in categories:
        n = overall[c]
        print(f"  {c:<18} {n:>5} ({100*n/max(total,1):>5.1f}%)")
    print(f"  {'TOTAL':<18} {total:>5}")

    # Detail every SP and near-SP rep
    print()
    print("=" * 92)
    print("ALL SOFT+PRECISE LANDINGS (the rarest signal)")
    print("=" * 92)
    sps = [r for r in all_reps if r["category"] == "SP"]
    if not sps:
        print("  (none found)")
    else:
        for r in sps:
            print(f"  {r['sweep']}/{r['bundle']}/{r['rep_name']}")
            print(f"    xy={r['xy_err']:.4f}  vel={r['rel_vel']:.4f}")
            for k in ("vh0","vz0","sigma_xy0","sigma_xy_tail","kappa_tail","flight_s"):
                if k in r:
                    print(f"    {k:<15}={r[k]:.4f}")

    print()
    print("=" * 92)
    print("NEAR-SP REPS  (xy<0.15, vel<0.4) — what's close but not crossing both thresholds?")
    print("=" * 92)
    near_sps = [r for r in all_reps if r["category"] == "near_SP"]
    near_sps.sort(key=lambda r: (r["xy_err"], r["rel_vel"]))
    print(f"  ({len(near_sps)} reps total — showing top 25 by xy)")
    for r in near_sps[:25]:
        missing = []
        if not r["precise"]: missing.append(f"xy={r['xy_err']:.3f}>0.08")
        if not r["soft"]:    missing.append(f"vel={r['rel_vel']:.3f}>0.2")
        if r["target_lost"]: missing.append("TL")
        print(f"  {r['sweep']:>22}/{r['bundle'][:14]:<14} {r['rep_name']:<10}  "
              f"xy={r['xy_err']:.4f} vel={r['rel_vel']:.4f}  miss: {','.join(missing)}")

    # Save full table for later cross-analysis
    out_path = Path("/tmp/all_landings.json")
    with out_path.open("w") as f:
        json.dump(all_reps, f, indent=2, default=str)
    print(f"\nFull table saved → {out_path}  ({len(all_reps)} reps)")


if __name__ == "__main__":
    main()
