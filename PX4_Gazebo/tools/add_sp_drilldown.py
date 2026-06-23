#!/usr/bin/env python3
"""Add (or refresh) a `Genuine_SP_Reps` per-rep drill-down sheet in
parameter_record.ods from test_data/test_record_runs.json['sp_reps'].

Idempotent: removes any existing Genuine_SP_Reps sheet first, so it can be
re-run after build_test_record.py without duplicating. Does NOT touch the
other sheets (unlike the one-shot restructure_parameter_record.py).
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


def row(values):
    r = TableRow()
    for v in values:
        c = TableCell(valuetype="string")
        for line in str(v).split("\n"):
            c.addElement(P(text=line))
        r.addElement(c)
    return r


def main():
    doc = load(str(ODS))
    for t in list(doc.spreadsheet.getElementsByType(Table)):
        if t.getAttribute("name") == "Genuine_SP_Reps":
            doc.spreadsheet.removeChild(t)

    sp = json.loads(JSON.read_text()).get("sp_reps", [])
    genuine = [r for r in sp if not r["frozen_GT"]]

    tbl = Table(name="Genuine_SP_Reps")
    tbl.addElement(row([
        f"Per-rep full-SP drill-down: {len(genuine)} genuine candidates "
        f"(+{len(sp) - len(genuine)} frozen-GT, listed last & flagged). "
        f"SP=precise(xy<=0.10m)&soft(vel<=0.2m/s)&!target_lost. "
        f"CAVEAT: some SPCampaign b9*/b10B reps are open-loop-HANDOFF INVALID "
        f"(last ~25cm not closed-loop) per PX4_Gain_Record remarks; verify vs "
        f"trajectory before citing. frozen-GT = xy_err<0.005m (GT reset to origin)."]))
    tbl.addElement(row(["Config", "Rep path (test_data/)", "Date",
                        "xy_err (m)", "rel_vel (m/s)", "flag"]))
    for r in sorted(sp, key=lambda x: (x["frozen_GT"], x["xy_err"])):
        tbl.addElement(row([r["config"], r["path"], r["date"],
                            r["xy_err"], r["rel_vel"],
                            "frozen-GT" if r["frozen_GT"] else ""]))
    doc.spreadsheet.addElement(tbl)
    doc.save(str(ODS))
    print(f"Genuine_SP_Reps: {len(genuine)} genuine + "
          f"{len(sp) - len(genuine)} frozen = {len(sp)} rows")
    print("sheets:", sorted(t.getAttribute("name")
          for t in doc.spreadsheet.getElementsByType(Table)))


if __name__ == "__main__":
    main()
