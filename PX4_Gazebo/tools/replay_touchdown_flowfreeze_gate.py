#!/usr/bin/env python3
"""Offline check: would the flow-freeze live-visibility gate (controller.py,
_touchdownDetectV2, 2026-09-21) suppress a recorded false touchdown, and does it leave
genuine touchdowns untouched?

Does NOT re-run the full V2 state machine (armed timing, the 3-in-6 windows, etc.) --
that needs live SITL. What it DOES check, from recorded Img_Data.npy /
<rep>.log pairs, is the one condition the gate adds: was the marker's own
FEATURE_IS_VISIBLE flag True at the moment flow-freeze actually fired. If it was, the
new `not marker_visible` precondition would have blocked that fire; if it wasn't
(a genuine off-marker settle), the fire is unaffected.

Also reports, for every rep in the glob, whether flow-freeze fired at all -- so a rep
with NO flow-freeze line is unaffected by this gate by construction (it only touches
that one path; overfill/backstop/IMU-spike are untouched).

Usage:
  replay_touchdown_flowfreeze_gate.py 'test_data/ICValidation/20260918-*/IC*_rep*'
"""
import sys
import os
import re
import glob
import numpy as np


def check_rep(d):
    """-> None if no flow-freeze firing in this rep's log, else a result dict."""
    logf = d + ".log"
    if not os.path.exists(logf):
        return None
    txt = open(logf, errors="ignore").read()
    m = re.search(r"TOUCHDOWN-DETECT v2 \[flow-freeze\]: extent=(\S+)/(\S+)px", txt)
    if not m:
        return None
    trig_ext = float(m.group(1))
    imf = os.path.join(d, "Img_Data.npy")
    if not os.path.exists(imf):
        return dict(fired=True, verdict="no Img_Data.npy -- can't check")
    im = np.load(imf, allow_pickle=True).item()
    vis = np.asarray(im.get("FEATURE_IS_VISIBLE", []), bool)
    ext_log = np.asarray(im.get("MARKER_EXTENT_PX", []), float)
    if not len(vis) or not len(ext_log):
        return dict(fired=True, verdict="missing FEATURE_IS_VISIBLE/MARKER_EXTENT_PX")
    # match the trigger extent within the last 60 frames (flow-freeze only fires
    # near the end of a recording, since it's a one-way terminal latch)
    tail = slice(max(0, len(ext_log) - 60), len(ext_log))
    idx = np.where(np.isclose(ext_log[tail], trig_ext, atol=1.0))[0]
    if not len(idx):
        return dict(fired=True, verdict="couldn't match trigger frame")
    i = tail.start + idx[-1]
    was_visible = bool(vis[i]) if i < len(vis) else None
    if was_visible is None:
        verdict = "unknown (index out of range)"
    elif was_visible:
        verdict = "SUPPRESSED by new gate (marker was visible -- not a genuine off-marker settle)"
    else:
        verdict = "still fires (marker genuinely not visible -- correct behavior)"
    return dict(fired=True, verdict=verdict, trig_ext=trig_ext)


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    rows = []
    for pat in argv:
        for d in sorted(glob.glob(pat)):
            if not os.path.isdir(d):
                continue
            r = check_rep(d)
            if r is not None:
                rows.append((d, r))
    if not rows:
        print("no flow-freeze firings found in the matched reps "
              "(this gate is a no-op for all of them by construction)")
        return 0
    n_suppress = 0
    n_keep = 0
    for d, r in rows:
        print(f"{d:<70} {r['verdict']}")
        if "SUPPRESSED" in r["verdict"]:
            n_suppress += 1
        elif "correct behavior" in r["verdict"]:
            n_keep += 1
    print(f"\n{len(rows)} flow-freeze firings found: {n_suppress} would be suppressed "
          f"(marker was visible), {n_keep} would still fire (marker genuinely absent), "
          f"{len(rows) - n_suppress - n_keep} unresolved")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
