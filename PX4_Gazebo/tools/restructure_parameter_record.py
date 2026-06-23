#!/usr/bin/env python3
"""One-shot restructure of test_data/Landing_Test/parameter_record.ods:

  1. Rescue the ~14 KB free-text log dump crammed into PX4_NewCal_Record!R0
     into a dedicated chronological `NewCal_Notes` sheet, and remove that row
     so the NewCal sheet starts at its real header.
  2. Add an `All_Test_Runs` sheet built from test_data/test_record_runs.json
     (one row per scanned config bundle, with extracted SP/TL/xy/vel metrics).

The three good sheets (PX4_Gain_Record, MATLAB_Test_Record, Removed_Parameters)
and the NewCal body are left intact. Back up the .ods before running.
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


def cell(text, bold=False):
    c = TableCell(valuetype="string")
    for line in str(text).split("\n"):
        c.addElement(P(text=line))
    return c


def row(values):
    r = TableRow()
    for v in values:
        r.addElement(cell(v))
    return r


def get_cell_text(c):
    return "\n".join(str(p) for p in c.getElementsByType(P))


def main():
    doc = load(str(ODS))
    tables = {t.getAttribute("name"): t for t in
              doc.spreadsheet.getElementsByType(Table)}

    # ---- 1. rescue the NewCal R0 log blob ----
    nc = tables["PX4_NewCal_Record"]
    rows = nc.getElementsByType(TableRow)
    r0 = rows[0]
    blob = max((get_cell_text(c) for c in r0.getElementsByType(TableCell)),
               key=len, default="")
    nc.removeChild(r0)  # drop the giant header row; real header is now row 0

    notes = Table(name="NewCal_Notes")
    notes.addElement(row(["#", "NewCal chronological log entry (rescued from the "
                          "old PX4_NewCal_Record header cell)"]))
    # the log is a running journal delimited by ' || '
    entries = [e.strip() for e in blob.split("||") if e.strip()]
    for i, e in enumerate(entries, 1):
        notes.addElement(row([i, e]))
    doc.spreadsheet.addElement(notes)
    print(f"NewCal_Notes: {len(entries)} entries rescued ({len(blob)} chars)")

    # ---- 2. All_Test_Runs from the scan ----
    data = json.loads(JSON.read_text())
    runs = Table(name="All_Test_Runs")
    runs.addElement(row([
        f"Auto-scanned from test_data/ by tools/build_test_record.py — "
        f"{data['n_configs']} configs, {data['n_reps']} reps, {data['n_SP']} full SP. "
        f"SP=precise(xy<=0.10m)&soft(vel<=0.2m/s)&!target_lost. "
        f"xy_min~0 + SP => verify vs frozen-GT false-SP (feedback_false_sp_frozen_gt)."]))
    cols = ["Config (test_data/)", "Date", "n", "SP", "TL", "near_SP",
            "catastrophic", "xy_med", "xy_min", "vel_med", "flag"]
    runs.addElement(row(cols))
    for r in data["configs"]:
        flag = "frozen-GT?" if (r["xy_min"] is not None
                                and r["xy_min"] < 0.005 and r["SP"]) else ""
        runs.addElement(row([
            r["config"], r["date"], r["n"], r["SP"], r["TL"], r["near_SP"],
            r["catastrophic"],
            "" if r["xy_med"] is None else r["xy_med"],
            "" if r["xy_min"] is None else r["xy_min"],
            "" if r["vel_med"] is None else r["vel_med"], flag]))
    doc.spreadsheet.addElement(runs)
    print(f"All_Test_Runs: {len(data['configs'])} config rows")

    doc.save(str(ODS))
    final = {t.getAttribute("name") for t in doc.spreadsheet.getElementsByType(Table)}
    print("sheets now:", sorted(final))


if __name__ == "__main__":
    main()
