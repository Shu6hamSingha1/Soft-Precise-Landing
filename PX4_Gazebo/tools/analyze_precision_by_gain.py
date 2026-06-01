#!/usr/bin/env python3
"""Look at every rep across all sweeps and ask: which gain perturbations
produce reps that get under PRECISE threshold (xy < 0.08)?

Uses the cross-bundle scan from scan_all_landings.py.  The bundle names
in BigSensitivity/, SensitivitySweep/, PitchAxisSweep/, etc. encode the
parameter being varied (e.g. KP_X_1.5_rep3 → K_rp boosted on X-axis).
"""
from __future__ import annotations
import json, re
from collections import defaultdict, Counter
import numpy as np


def parse_cell(rep_name: str, bundle: str = ""):
    """Try to extract (param_family, scale, axis) from rep_name or bundle.
    Examples:
      'KP_X_1.5_rep3' → ('KP', 1.5, 'X')
      'P_Z_0.5_rep1'  → ('P', 0.5, 'Z')
      'RHOFOVINF_1.5_rep3' → ('RHOFOVINF', 1.5, '')
      'YAW_E_1.5_rep2' → ('YAW_E', 1.5, '')
      'ref_-0.70_rep4' → ('REF_RAD', -0.70, '')
      'kr_0.4_rep1'   → ('KR', 0.4, '')
      'rep2', 'rep10' → ('BASELINE', 1.0, '')
    """
    m = re.match(r"^([A-Z][A-Z_0-9]*)_([XYZ])_(-?\d+(?:\.\d+)?)(?:_rep(\d+))?$", rep_name)
    if m:
        return (m.group(1), float(m.group(3)), m.group(2))
    m = re.match(r"^([A-Z][A-Z_0-9]*)_(-?\d+(?:\.\d+)?)(?:_rep(\d+))?$", rep_name)
    if m:
        return (m.group(1), float(m.group(2)), "")
    m = re.match(r"^(ref|kr|kp|kd|ki|tau|p20|p2inf|rhofov0|rhofovinf|lfov|thetacap)_(-?\d+\.?\d*)", rep_name, re.I)
    if m:
        return (m.group(1).upper(), float(m.group(2)), "")
    if re.match(r"^rep\d+$", rep_name):
        return ("BASELINE", 1.0, "")
    return (None, None, None)


def main():
    with open("/tmp/all_landings.json") as f:
        reps = json.load(f)

    # Parse params for every rep
    enriched = []
    for r in reps:
        fam, scale, axis = parse_cell(r["rep_name"], r.get("bundle", ""))
        r["param_family"] = fam
        r["param_scale"]  = scale
        r["param_axis"]   = axis
        enriched.append(r)

    # Filter to reps that landed (have xy)
    landed = [r for r in enriched if r["xy_err"] is not None and not np.isnan(r["xy_err"])]
    print(f"Reps with known param config + landed: "
          f"{len([r for r in landed if r['param_family'] is not None])}")
    print(f"BASELINE reps (default config):       "
          f"{len([r for r in landed if r['param_family'] == 'BASELINE'])}")
    print(f"Unknown-config reps:                   "
          f"{len([r for r in landed if r['param_family'] is None])}")
    print()

    # All PRECISE reps (xy < 0.08)
    precise = [r for r in landed if r["xy_err"] < 0.08]
    print(f"All PRECISE reps (xy<0.08): {len(precise)}")
    print(f"  Of which SP:        {sum(1 for r in precise if r.get('sp_full'))}")
    print(f"  Of which TL:        {sum(1 for r in precise if r['target_lost'])}")
    print(f"  Of which had vel>0.5: {sum(1 for r in precise if r['rel_vel'] > 0.5)}")
    print()

    # Which (param_family, scale) combos produced ≥1 PRECISE rep?
    family_precise = defaultdict(list)
    for r in precise:
        if r["param_family"] is None: continue
        key = (r["param_family"], r["param_axis"], r["param_scale"])
        family_precise[key].append(r)

    # Rank by xy_min within the family
    print("=" * 110)
    print("PARAM SETTINGS THAT EVER PRODUCED A PRECISE LANDING")
    print("=" * 110)
    print(f"{'family':<14} {'axis':<5} {'scale':>7}  {'n_precise':>9} {'xy_min':>7} {'xy_mean':>8} {'vel_mean':>9}  {'note'}")
    rows = []
    for (fam, ax, sc), reps_in in family_precise.items():
        xys = [r["xy_err"] for r in reps_in]
        vels = [r["rel_vel"] for r in reps_in]
        rows.append((fam, ax, sc, len(reps_in), min(xys), np.mean(xys), np.mean(vels)))
    rows.sort(key=lambda r: r[4])  # by xy_min
    for fam, ax, sc, n, xmn, xa, va in rows:
        print(f"  {fam:<12} {ax:<5} {sc:>7.3f}   {n:>9} {xmn:>7.4f} {xa:>8.4f} {va:>9.4f}")

    # For each parameter family, what's the best xy_min observed?
    print()
    print("=" * 110)
    print("BEST xy_min PER PARAMETER FAMILY (across all scales/axes)")
    print("=" * 110)
    family_all = defaultdict(list)
    for r in landed:
        if r["param_family"] is None: continue
        family_all[r["param_family"]].append(r)
    family_summary = []
    for fam, reps_in in family_all.items():
        xys = [r["xy_err"] for r in reps_in]
        n_pp = sum(1 for r in reps_in if r["xy_err"] < 0.08)
        family_summary.append({
            "family": fam, "n": len(reps_in),
            "xy_min": min(xys), "xy_mean": np.mean(xys),
            "xy_median": np.median(xys),
            "n_precise": n_pp,
            "precise_rate": n_pp / len(reps_in),
        })
    family_summary.sort(key=lambda r: r["xy_min"])
    print(f"{'family':<14} {'n':>4} {'xy_min':>8} {'xy_median':>10} {'xy_mean':>8} {'n_precise':>10} {'precise_rate':>13}")
    for f in family_summary:
        print(f"  {f['family']:<12} {f['n']:>4} {f['xy_min']:>8.4f} {f['xy_median']:>10.4f} "
              f"{f['xy_mean']:>8.4f} {f['n_precise']:>10} {100*f['precise_rate']:>12.1f}%")

    # Now ask the controller-tuning question: for the families that ever
    # produced PRECISE, what's the WORST xy observed at the same scale?
    # (Robustness — if a config produces PRECISE 1/5 times but xy=1.2 the
    # other 4 times, that's a bad config.)
    print()
    print("=" * 110)
    print("PRECISE-PRODUCING CELLS — what's the spread / worst-case at same config?")
    print("=" * 110)
    print(f"{'family':<14} {'axis':<5} {'scale':>7}  {'best_xy':>8} {'worst_xy':>9} {'mean_xy':>8} {'mean_vel':>9} {'n_total':>8} {'n_precise':>10}")
    for (fam, ax, sc), reps_in in family_precise.items():
        # Find all reps at this same config across all bundles
        same_cfg = [r for r in landed
                    if r["param_family"] == fam and r["param_axis"] == ax
                    and r["param_scale"] == sc]
        xys  = [r["xy_err"] for r in same_cfg]
        vels = [r["rel_vel"] for r in same_cfg]
        n_p = sum(1 for r in same_cfg if r["xy_err"] < 0.08)
        print(f"  {fam:<12} {ax:<5} {sc:>7.3f}   {min(xys):>8.4f} {max(xys):>9.4f} "
              f"{np.mean(xys):>8.4f} {np.mean(vels):>9.4f} {len(same_cfg):>8} {n_p:>10}")

    # FINALLY: for the reps that came closest to SP (xy<0.15 AND vel<0.5),
    # what gain configs produced them?
    print()
    print("=" * 110)
    print("REPS NEAREST SP (xy<0.15 AND vel<0.5) — what configs got us closest?")
    print("=" * 110)
    near_sp = [r for r in landed if r["xy_err"] < 0.15 and r["rel_vel"] < 0.5
               and not r["target_lost"]]
    near_sp.sort(key=lambda r: r["xy_err"] + 0.5 * r["rel_vel"])
    print(f"{'xy':>7} {'vel':>6}  {'family':<14} {'axis':<5} {'scale':>7}  {'bundle':<24} {'rep':<24}")
    for r in near_sp[:25]:
        fam = r["param_family"] or "?"; ax = r["param_axis"] or ""
        sc = r["param_scale"] if r["param_scale"] is not None else float("nan")
        print(f"  {r['xy_err']:>6.4f} {r['rel_vel']:>6.4f}  {fam:<12} {ax:<5} {sc:>7.3f}  "
              f"{r['bundle'][:22]:<22} {r['rep_name']:<24}")


if __name__ == "__main__":
    main()
