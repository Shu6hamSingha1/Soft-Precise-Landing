#!/usr/bin/env python3
"""Scan every SITL landing rep under PX4_Gazebo/test_data/, aggregate per
config bundle, and emit three artifacts from one source of truth:

  * test_data/test_record_runs.json   — full per-config aggregate (machine)
  * test_data/test_record.tsv         — one row per config (grep/awk friendly)
  * docs/TEST_RECORD.md               — reading digest (Claude/human reference)

The canonical per-rep metric source is Ground_Truth.npy['SoftPrecise']
(xy_err, rel_vel, precise, soft, target_lost); classification taxonomy
mirrors tools/scan_all_landings.py::classify_rep. Re-run to regenerate:
  ~/ws/scripts/env2025/bin/python3 tools/build_test_record.py
"""
from __future__ import annotations
import json
import os
import re
import statistics as st
from collections import Counter
from pathlib import Path
import numpy as np

ROOT = Path(os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo"))
TEST_DATA = ROOT / "test_data"

_TS_DASH = re.compile(r"(20\d{6})-(\d{6})")
_TS_WORDY = re.compile(r"[A-Z][a-z]{2} ([A-Z][a-z]{2}) +(\d{1,2}) [\d-]+ (20\d{2})")
_MONTHS = {m: i for i, m in enumerate(
    "Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec".split(), 1)}


def parse_date(path_str: str) -> str:
    """Best-effort YYYY-MM-DD from a path containing a timestamp dir."""
    m = _TS_DASH.search(path_str)
    if m:
        d = m.group(1)
        return f"{d[:4]}-{d[4:6]}-{d[6:8]}"
    m = _TS_WORDY.search(path_str)
    if m:
        mon, day, yr = m.group(1), int(m.group(2)), m.group(3)
        return f"{yr}-{_MONTHS.get(mon, 0):02d}-{day:02d}"
    return ""


def classify(xy, vel, tl, prec, soft):
    if np.isnan(xy):
        return "NO_DATA"
    if prec and soft and not tl:
        return "SP"
    if tl:
        return "TARGET_LOST"
    if prec and not soft:
        return "PRECISE_only"
    if soft and not prec:
        return "SOFT_only"
    if xy < 0.15 and vel < 0.4:
        return "near_SP"
    if xy < 0.3:
        return "MOD_PRECISE"
    if xy >= 1.0:
        return "CATASTROPHIC"
    return "POOR"


def read_rep(gt_path: Path):
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
    return {"xy": xy, "vel": vel, "tl": tl, "prec": prec, "soft": soft,
            "cat": classify(xy, vel, tl, prec, soft)}


def med(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and np.isnan(x))]
    return round(st.median(xs), 3) if xs else None


def main():
    # group every leaf run by its top-level config dir
    configs: dict[str, list] = {}
    config_dates: dict[str, list] = {}
    for gt in TEST_DATA.rglob("Ground_Truth.npy"):
        rel = gt.relative_to(TEST_DATA)
        config = rel.parts[0]
        rep = read_rep(gt)
        if rep is None:
            continue
        rep["path"] = str(gt.parent.relative_to(TEST_DATA))
        configs.setdefault(config, []).append(rep)
        d = parse_date(str(gt.parent))
        if d:
            config_dates.setdefault(config, []).append(d)

    rows = []
    for config in sorted(configs):
        reps = configs[config]
        n = len(reps)
        cats = Counter(r["cat"] for r in reps)
        xy_all = [r["xy"] for r in reps]
        vel_all = [r["vel"] for r in reps]
        dates = sorted(config_dates.get(config, []))
        rows.append({
            "config": config,
            "date": dates[0] if dates else "",
            "n": n,
            "SP": cats.get("SP", 0),
            "TL": cats.get("TARGET_LOST", 0),
            "near_SP": cats.get("near_SP", 0),
            "catastrophic": cats.get("CATASTROPHIC", 0),
            "xy_med": med(xy_all),
            "xy_min": round(min(x for x in xy_all if not np.isnan(x)), 3)
                      if any(not np.isnan(x) for x in xy_all) else None,
            "vel_med": med(vel_all),
            "cats": dict(cats),
        })

    # sort rows by date then config
    rows.sort(key=lambda r: (r["date"], r["config"]))

    total_reps = sum(r["n"] for r in rows)
    total_sp = sum(r["SP"] for r in rows)

    # ---- JSON ----
    out_json = TEST_DATA / "test_record_runs.json"
    out_json.write_text(json.dumps(
        {"n_configs": len(rows), "n_reps": total_reps, "n_SP": total_sp,
         "configs": rows}, indent=1))

    # ---- TSV ----
    cols = ["config", "date", "n", "SP", "TL", "near_SP", "catastrophic",
            "xy_med", "xy_min", "vel_med"]
    tsv = TEST_DATA / "test_record.tsv"
    with tsv.open("w") as f:
        f.write("\t".join(cols) + "\n")
        for r in rows:
            f.write("\t".join("" if r[c] is None else str(r[c]) for c in cols) + "\n")

    # ---- MD digest ----
    md = ROOT / "docs" / "TEST_RECORD.md"
    lines = [
        "# PX4/Gazebo SITL Test Record (auto-generated)",
        "",
        f"Generated by `tools/build_test_record.py` from `test_data/`. "
        f"**{len(rows)} config bundles, {total_reps} reps, {total_sp} full SP.**",
        "Per-rep source = `Ground_Truth.npy['SoftPrecise']`; verdict taxonomy "
        "from `scan_all_landings.py`.",
        "SP = precise (xy_err<=0.10m) AND soft (rel_vel<=0.2m/s) AND not target_lost.",
        "",
        "> ⚠ **Frozen-GT false-SP caveat:** rows with `xy_min` ≈ 0 (flagged ⚠) can "
        "contain a frozen-GT artifact (ground truth resets to origin → xy_err≈1e-21 → "
        "spurious SP). Verify such SPs against the trajectory before trusting "
        "(see `Memory/feedback_false_sp_frozen_gt.md`).",
        "",
        "For full per-axis gains per trial, see `parameter_record.ods` "
        "(`PX4_Gain_Record` / `PX4_NewCal_Record` sheets).",
        "",
        "| Config (test_data/) | Date | n | SP | TL | catastr | xy_med | xy_min | vel_med | |",
        "|---|---|--:|--:|--:|--:|--:|--:|--:|:--|",
    ]
    for r in rows:
        flag = "⚠" if (r["xy_min"] is not None and r["xy_min"] < 0.005 and r["SP"]) else ""
        d = {k: ("" if r[k] is None else r[k]) for k in r}
        lines.append("| {config} | {date} | {n} | {SP} | {TL} | {catastrophic} | "
                     "{xy_med} | {xy_min} | {vel_med} | ".format(**d) + flag + " |")
    md.write_text("\n".join(lines) + "\n")

    print(f"configs={len(rows)} reps={total_reps} SP={total_sp}")
    print("wrote:", out_json, tsv, md, sep="\n  ")


if __name__ == "__main__":
    main()
