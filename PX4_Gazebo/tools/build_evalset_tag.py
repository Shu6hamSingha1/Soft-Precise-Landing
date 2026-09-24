#!/usr/bin/env python3
"""Assemble one perception eval-set TAG from a recorded landing rep + its IMG_RECORD frames
(2026-09-24, for the locked-design detector rewrite -- see
Memory/px4/feedback_cross_detector_robustness_requirement.md).

Output layout (what tools/validate_detector_gt.py --set expects):
    <out>/<tag>/Ground_Truth.npy Img_Data.npy Control_Data.npy Img_Params.txt
                meta.json                      {"world", "marker_dz", "case", "source_rep", "source_raw"}
                frames/f%05d.png frames/frames.tsv

The raw dir is found by STAMP MATCH, not name/mtime (both have lied before --
feedback_recurring_analysis_mistakes §2/§13): the candidate *_raw dir whose frames.tsv
capture stamps all appear in this rep's Img_Data['Stamp'] wins. Raw dirs without a
frames.tsv (recorded before 2026-09-24) are refused rather than guessed.

Usage:
  python3 tools/build_evalset_tag.py <rep_dir> <out_dir> <tag> --world rover_cross --marker-dz 0.5 [--case Sinusoidal]
"""
import sys, os, glob, json, shutil, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VIDEOS = os.path.join(HERE, "..", "test_data", "Test_Videos")


def find_raw(rep_dir):
    """The *_raw dir whose frames.tsv stamps form a CONTIGUOUS, IN-ORDER block of this rep's
    Img_Data['Stamp'] rows (frames are saved once per processed frame; Img_Data logs once
    per processed frame -> the correct pairing is positional). Set MEMBERSHIP is NOT enough:
    sim time restarts at 0 every launch and frames land on a near-identical ~16 ms cadence,
    so an UNRELATED run of the same campaign shares 59-70% of stamps by coincidence
    (measured 2026-09-24) vs 91-100% for the true one -- too thin a margin to trust.
    Score = longest run of positional equality tsv[i] == img[j0 + i] over the frames inside
    the rep's logged window (Img_Data can stop at touchdown while frames keep coming)."""
    arr = np.asarray(np.load(os.path.join(rep_dir, "Img_Data.npy"), allow_pickle=True).item()["Stamp"], float)
    lo, hi = float(np.nanmin(arr)), float(np.nanmax(arr))
    pos = {}
    for j, v in enumerate(arr.tolist()):
        pos.setdefault(v, j)
    best = None
    for tsv in glob.glob(os.path.join(VIDEOS, "*_raw", "frames.tsv")):
        rows = open(tsv).read().splitlines()[1:]
        if not rows:
            continue
        stamps = np.array([float(r.split("\t")[1]) for r in rows])
        inw = np.nonzero((stamps >= lo) & (stamps <= hi))[0]
        if len(inw) < 0.5 * len(stamps):
            continue
        run = cur = 0
        for i in inw:
            j0 = pos.get(stamps[i])
            if j0 is not None and cur and j0 == prev_j + 1:
                cur += 1
            elif j0 is not None:
                cur = 1
            else:
                cur = 0
            prev_j = j0 if j0 is not None else -10
            run = max(run, cur)
        frac = run / len(inw)
        # measured: true pairing 0.76-1.00, best unrelated run <=0.068 -> 0.5 is >7x margin
        if frac > 0.5 and (best is None or frac > best[1]):
            best = (os.path.dirname(tsv), frac, int(len(inw)))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rep_dir"); ap.add_argument("out_dir"); ap.add_argument("tag")
    ap.add_argument("--world", required=True)
    ap.add_argument("--marker-dz", type=float, required=True)
    ap.add_argument("--case", default="")
    a = ap.parse_args()
    rep = a.rep_dir.rstrip("/")
    best = find_raw(rep)
    if best is None:
        sys.exit(f"no *_raw dir with a stamp-matching frames.tsv for {rep} (recorded before the sidecar existed?)")
    raw, hit, n = best
    dst = os.path.join(a.out_dir, a.tag)
    os.makedirs(os.path.join(dst, "frames"), exist_ok=True)
    for f in ("Ground_Truth.npy", "Img_Data.npy", "Control_Data.npy", "Img_Params.txt"):
        if os.path.isfile(os.path.join(rep, f)):
            shutil.copy2(os.path.join(rep, f), dst)
    for f in glob.glob(os.path.join(raw, "f*.png")) + [os.path.join(raw, "frames.tsv")]:
        shutil.copy2(f, os.path.join(dst, "frames"))
    json.dump(dict(world=a.world, marker_dz=a.marker_dz, case=a.case,
                   source_rep=os.path.relpath(rep, os.path.join(HERE, "..")),
                   source_raw=os.path.relpath(raw, os.path.join(HERE, ".."))),
              open(os.path.join(dst, "meta.json"), "w"), indent=1)
    print(f"{a.tag}: {n} frames (contiguous stamp run {100*hit:.1f}% of in-window) <- {os.path.basename(raw)} + {os.path.basename(rep)}")


if __name__ == "__main__":
    main()
