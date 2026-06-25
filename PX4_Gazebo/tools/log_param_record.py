#!/usr/bin/env python3
"""Append a row to parameter_record.ods :: PX4_NewCal_Record (with backup).

Columns (36): No,Date,n,Tested(delta),Purpose,P_x,P_y,P_z,E_x,E_y,E_z,KP,DH_D_MAX,
KI,GAM_Y,N_z,N_xy,FLOOR,W_U_MAX,PSID_RATE,YAW_GAM,gamma_s,TAU_UA,TAU_DS,xy_med,
xy_min,xy_max,vel_med,TL,SP,e_a_max(deg),sen_max,Verdict,Finding,Bundle,Timestamp

Usage (programmatic): import and call append_row(dict) keyed by column name.
CLI: python3 tools/log_param_record.py 'No=110' 'Date=06-23' 'Tested(delta)=...' ...
Unspecified columns -> ''. Always backs up the ods to .bak_<epoch> first.
"""
import sys, os, time, shutil
from odf.opendocument import load
from odf.table import Table, TableRow, TableCell
from odf.text import P

ODS = os.path.join(os.path.dirname(__file__), "..", "test_data", "Landing_Test", "parameter_record.ods")
SHEET = "PX4_NewCal_Record"
COLS = ['No','Date','n','Tested(delta)','Purpose','P_x','P_y','P_z','E_x','E_y','E_z','KP',
        'DH_D_MAX','KI','GAM_Y','N_z','N_xy','FLOOR','W_U_MAX','PSID_RATE','YAW_GAM','gamma_s',
        'TAU_UA','TAU_DS','xy_med','xy_min','xy_max','vel_med','TL','SP','e_a_max(deg)','sen_max',
        'Verdict','Finding','Bundle (test_data/)','Timestamp']


def append_row(values: dict, ods=ODS):
    bad = [k for k in values if k not in COLS]
    if bad:
        raise KeyError(f"unknown columns {bad}; valid: {COLS}")
    shutil.copy2(ods, ods + f".bak_{int(time.time())}")
    doc = load(ods)
    table = next(t for t in doc.spreadsheet.getElementsByType(Table)
                 if t.getAttribute("name") == SHEET)
    tr = TableRow()
    for c in COLS:
        cell = TableCell(valuetype="string")
        cell.addElement(P(text=str(values.get(c, ""))))
        tr.addElement(cell)
    table.addElement(tr)
    doc.save(ods)
    print(f"[log] appended NC{values.get('No','?')} to {SHEET}")


if __name__ == "__main__":
    d = {}
    for a in sys.argv[1:]:
        k, _, v = a.partition("=")
        d[k] = v
    append_row(d)
