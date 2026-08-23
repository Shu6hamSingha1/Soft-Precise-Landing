"""Standalone Phase-0 feasibility benchmark for PlanarFeatureMap on real Pi
hardware/CPU, using LIVE captured frames (not synthetic) so goodFeaturesToTrack/
KLT see realistic scene texture. Does NOT touch img_data.py or the capture loop -
throwaway diagnostic, run once and discarded.

Single imgstream instance (avoid re-initializing the camera twice) at the
project's normal 640x480 raw config; pulls BOTH the 320x240 'main' ISP-scaled
stream (already the size ArUco detection uses) and the 640x480 raw stream from
the SAME captured frames, times bootstrap()+update() at each size, reports
mean/p95 ms/frame to compare against the project's known ArUco-detection cost
(76% of frame budget at whatever resolution it runs).
"""
import time
import numpy as np
from imgstreamer import imgstream
from planar_map import PlanarFeatureMap

N_FRAMES = 40
WARMUP = 5


def bench(frames, label):
    print(f"\n=== {label} (n={len(frames)} frames, shape={frames[0].shape}) ===")
    if len(frames) < WARMUP + 5:
        print("  not enough frames, skipping")
        return
    pm = PlanarFeatureMap()
    pm.bootstrap(frames[0])
    times = []
    for f in frames[1:]:
        t0 = time.perf_counter()
        pm.update(f)
        times.append((time.perf_counter() - t0) * 1000.0)
    times = np.array(times[WARMUP:])
    print(f"  update() cost: mean={times.mean():.1f}ms  p95={np.percentile(times,95):.1f}ms  "
          f"max={times.max():.1f}ms  n_tracked_final={pm.n_tracked}  map_conf={pm.map_confidence:.2f}")
    print(f"  implied max sustainable fps from update() alone: {1000.0/times.mean():.1f}")


if __name__ == "__main__":
    strm = imgstream(resolution=(640, 480), capRate=30)
    time.sleep(2.0)  # let AEC/AGC + capture settle
    main_frames, raw_frames = [], []
    last_main = None
    t0 = time.perf_counter()
    while len(main_frames) < N_FRAMES + WARMUP and (time.perf_counter() - t0) < 40:
        m = list(strm.getImages())[-1]
        r = list(strm.getImages())[-1]
        if m is not None and r is not None and (last_main is None or not np.array_equal(m, last_main)):
            main_frames.append(m.copy())
            raw_frames.append(r.copy())
            last_main = m
        time.sleep(0.02)
    strm.close()
    print(f"captured {len(main_frames)} distinct frame pairs")

    bench(main_frames, "320x240 main/ISP-scaled stream (ArUco-detection size)")
    bench(raw_frames, "640x480 raw stream")
