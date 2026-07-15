#!/usr/bin/env python3
"""Offline validation of PlanarFeatureMap against recorded IMG_RECORD_RAW frames.
Runs the map builder across a real sequence, decoding ArUco only sparsely (simulating
the intended live behavior), and compares the map's inferred marker position against
what a FRESH DECODE would have said on frames where decode actually succeeds (held out,
not fed to the map) -- a genuine held-out accuracy check, not just a self-consistency one.

MULTI-SLOT (2026-07-15): PlanarFeatureMap now tracks BOTH nested markers (small+big) as
persistent slots and routes each decode to the right one by pixel geometry alone (see
src/planar_map.py module docstring) -- so this script no longer needs to maintain its own
persistent marker-id lock at all; it just decodes "whichever marker is best-conditioned
this frame" every time and lets the module's own slot routing sort out identity. The
held-out check compares against WHICHEVER slot the fresh decode best matches (minimum
error across all current slots), not a specific assumed identity.

Usage: python3 tools/validate_planar_map.py <raw_frame_dir>
"""
import sys, os, glob
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from planar_map import PlanarFeatureMap

ADICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMS = cv2.aruco.DetectorParameters()
DET = cv2.aruco.ArucoDetector(ADICT, PARAMS)


def decode(gray):
    """Decode the best-conditioned (largest-spread) currently-visible marker, no
    persistent lock needed -- PlanarFeatureMap's own slot routing (geometry, not id)
    handles identity continuity across whichever marker happens to decode each frame."""
    corners, ids, _ = DET.detectMarkers(gray)
    if ids is None:
        return None, None
    ids_flat = ids.flatten()
    spreads = [float(np.std(c.reshape(-1, 2))) for c in corners]
    k = int(np.argmax(spreads))
    return corners[k].reshape(-1, 2), int(ids_flat[k])


def best_match_error(pm, corners):
    """Held-out error vs whichever slot this decode geometrically belongs to, via the
    module's own public identify_slot() -- the SAME routing decision
    loop_closure_correct uses internally, so this harness can never disagree with the
    module about which slot a decode is (that mismatch was a real, separate bug found in
    img_data.py's live shadow wiring 2026-07-15 -- see its comment). Returns None if the
    decode doesn't match any slot yet (a marker not yet discovered/mapped -- correctly
    excluded, not a real accuracy failure)."""
    slot = pm.identify_slot(corners)
    if slot is None:
        return None
    pred = pm.get_marker_frame_pts(slot=slot)
    if pred is None or len(pred) != len(corners):
        return None
    return float(np.mean(np.linalg.norm(pred - corners, axis=1)))


def main(raw_dir):
    frames = sorted(glob.glob(os.path.join(raw_dir, "f*.png")))
    print(f"{len(frames)} frames in {raw_dir}")

    pm = PlanarFeatureMap()
    held_out_errors = []
    n_decode_calls = 0
    n_novel_excluded = 0   # held-out decodes that didn't match any mapped slot yet (see best_match_error)
    DECODE_EVERY_MIN = 3  # simulate the adaptive scheduler's floor
    last_mid = None

    for i, fpath in enumerate(frames):
        gray = cv2.imread(fpath, cv2.IMREAD_GRAYSCALE)

        if not pm.initialized:
            corners, mid = decode(gray)
            last_mid = mid
            pm.bootstrap(gray, marker_px_corners=corners, marker_id=mid)
            n_decode_calls += 1
            continue

        pm.update(gray)

        trigger = pm.should_trigger_decode(min_interval_frames=DECODE_EVERY_MIN)
        if trigger:
            corners, mid = decode(gray)
            n_decode_calls += 1
            if corners is not None:
                if mid != last_mid:
                    print(f"  [ID CHANGE] frame {i}: decoded marker id {last_mid} -> {mid} "
                          f"(informational only -- slot routing is geometric, not id-based)")
                    last_mid = mid
                pm.loop_closure_correct(corners, marker_id=mid)
        else:
            # HELD-OUT CHECK: on frames we did NOT feed to the map, if decode happens to
            # succeed anyway, compare it to whichever slot the map itself would say is the
            # best match -- a genuine accuracy check, not circular.
            corners, mid = decode(gray)
            if corners is not None:
                err = best_match_error(pm, corners)
                if err is not None:
                    held_out_errors.append((i, err))
                else:
                    n_novel_excluded += 1

        if i % 40 == 0:
            print(f"  frame {i}: n_tracked={pm.n_tracked} resid_px={pm.resid_px:.2f} "
                  f"confidence={pm.confidence:.2f} frames_since_decode={pm.frames_since_decode} "
                  f"n_slots={len(pm.marker_slots)} "
                  f"marker_rigid_ok={pm.marker_rigid_ok} marker_shape_change={pm.marker_shape_change:.3f}")

    print(f"\ndecode calls: {n_decode_calls}/{len(frames)} frames ({100*n_decode_calls/len(frames):.0f}%)")
    print(f"marker slots discovered: {len(pm.marker_slots)}")
    print(f"held-out decodes excluded as 'not yet a mapped slot' (not a routing error, "
          f"see best_match_error docstring): {n_novel_excluded}")
    if held_out_errors:
        idxs = np.array([h[0] for h in held_out_errors])
        he = np.array([h[1] for h in held_out_errors])
        print(f"held-out marker-position error (map-only, not decode-corrected this frame):")
        print(f"  n={len(he)}  mean={he.mean():.2f}px  median={np.median(he):.2f}px  "
              f"p90={np.percentile(he,90):.2f}px  max={he.max():.2f}px")
        worst = np.argsort(he)[::-1][:5]
        print("  worst 5 frames:")
        for w in worst:
            print(f"    frame={idxs[w]}  err={he[w]:.2f}px")
    else:
        print("no held-out comparison frames available (decode never succeeded on a non-triggered frame)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    main(sys.argv[1])
