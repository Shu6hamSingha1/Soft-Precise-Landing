#!/usr/bin/env python3
"""Idempotently (re)build the two AUTO-SCANNED sheets in parameter_record.ods
from test_data/test_record_runs.json:

  * All_Test_Runs    — one row per scanned config bundle (aggregate metrics)
  * Genuine_SP_Reps  — per-rep drill-down of full-SP candidates (frozen-GT last)

Both sheets are removed-then-readded, so this is safe to re-run after every
build_test_record.py scan. It does NOT touch the manually-maintained sheets
(PX4_Gain_Record, PX4_NewCal_Record, Removed_Parameters, MATLAB_Test_Record,
NewCal_Notes). Supersedes the one-shot restructure_parameter_record.py (which
performed the original NewCal_Notes rescue) and the old add_sp_drilldown.py.

  ~/ws/scripts/env2025/bin/python3 tools/build_test_record.py       # scan
  ~/ws/scripts/env2025/bin/python3 tools/refresh_scan_sheets.py     # then this
"""
import json
import os
from pathlib import Path
from odf.opendocument import load
from odf.table import Table, TableRow, TableCell
from odf.text import P

ROOT = Path(os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo"))
ODS = ROOT / "test_data" / "Landing_Test" / "parameter_record.ods"
JSON = ROOT / "test_data" / "test_record_runs.json"
AUTO_SHEETS = ("All_Test_Runs", "Genuine_SP_Reps")


def row(values):
    r = TableRow()
    for v in values:
        c = TableCell(valuetype="string")
        for line in str(v).split("\n"):
            c.addElement(P(text=line))
        r.addElement(c)
    return r


def build_all_test_runs(data):
    t = Table(name="All_Test_Runs")
    t.addElement(row([
        f"Auto-scanned from test_data/ by tools/build_test_record.py — "
        f"{data['n_configs']} configs, {data['n_reps']} reps, {data['n_SP']} full SP. "
        f"SP=precise(xy<=0.10m)&soft(vel<=0.2m/s)&!target_lost. "
        f"xy_min~0 + SP => verify vs frozen-GT false-SP (feedback_false_sp_frozen_gt)."]))
    cols = ["Config (test_data/)", "Date", "n", "SP", "TL", "near_SP",
            "catastrophic", "xy_med", "xy_min", "vel_med", "flag"]
    t.addElement(row(cols))
    for r in data["configs"]:
        flag = "frozen-GT?" if (r["xy_min"] is not None
                                and r["xy_min"] < 0.005 and r["SP"]) else ""
        t.addElement(row([
            r["config"], r["date"], r["n"], r["SP"], r["TL"], r["near_SP"],
            r["catastrophic"],
            "" if r["xy_med"] is None else r["xy_med"],
            "" if r["xy_min"] is None else r["xy_min"],
            "" if r["vel_med"] is None else r["vel_med"], flag]))
    return t, len(data["configs"])


def build_genuine_sp(data):
    sp = data.get("sp_reps", [])
    genuine = [r for r in sp if not r["frozen_GT"]]
    t = Table(name="Genuine_SP_Reps")
    t.addElement(row([
        f"Per-rep full-SP drill-down: {len(genuine)} genuine candidates "
        f"(+{len(sp) - len(genuine)} frozen-GT, listed last & flagged). "
        f"SP=precise(xy<=0.10m)&soft(vel<=0.2m/s)&!target_lost. "
        f"CAVEAT: some SPCampaign b9*/b10B reps are open-loop-HANDOFF INVALID "
        f"(last ~25cm not closed-loop) per PX4_Gain_Record remarks; verify vs "
        f"trajectory before citing. frozen-GT = xy_err<0.005m (GT reset to origin)."]))
    t.addElement(row(["Config", "Rep path (test_data/)", "Date",
                      "xy_err (m)", "rel_vel (m/s)", "flag"]))
    for r in sorted(sp, key=lambda x: (x["frozen_GT"], x["xy_err"])):
        t.addElement(row([r["config"], r["path"], r["date"], r["xy_err"],
                          r["rel_vel"], "frozen-GT" if r["frozen_GT"] else ""]))
    return t, len(genuine), len(sp)


def main():
    data = json.loads(JSON.read_text())
    doc = load(str(ODS))
    for t in list(doc.spreadsheet.getElementsByType(Table)):
        if t.getAttribute("name") in AUTO_SHEETS:
            doc.spreadsheet.removeChild(t)
    atr, n_cfg = build_all_test_runs(data)
    gsp, n_gen, n_sp = build_genuine_sp(data)
    doc.spreadsheet.addElement(atr)
    doc.spreadsheet.addElement(gsp)
    doc.save(str(ODS))
    print(f"All_Test_Runs: {n_cfg} configs | "
          f"Genuine_SP_Reps: {n_gen} genuine + {n_sp - n_gen} frozen = {n_sp}")
    print("sheets:", sorted(t.getAttribute("name")
          for t in doc.spreadsheet.getElementsByType(Table)))


if __name__ == "__main__":
    main()
