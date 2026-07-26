"""Find and extract the pure-yaw (low-translation, high-yaw-rate) sub-window
of an existing output-cal mocap recording, so check_mount_rotation() can be
re-run against a clean isolated segment without needing a fresh recording.

Slices Ground_Truth.npy / Img_Data.npy / Telemetry_Data.npy by ABSOLUTE
perf_counter() time (all three files share the same process clock -
confirmed: output_calibration.py/flight_controller.py/img_data.py all stamp
via self._time.perf_counter() / time.perf_counter() in the SAME process, no
epoch offset between files) - a straight time-window boolean mask on each
file's own native time array keeps everything correctly synced, no
resampling/interpolation needed for the trim itself (only the mount-rotation
check's OWN internal alignment, unchanged, does that later).

Usage: python trim_pure_yaw.py <run_dir> [--speed-max 0.12] [--yaw-min 0.35] [--min-dur 3.0]
"""
import os
import sys
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from derive_pi_cal import _pose_to_frd, _robust_vel, T_QTM_TO_FRD  # noqa: E402


def gt_speed_and_yawrate(run_dir):
    gt = np.load(os.path.join(run_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt["Start Time"])
    tg = np.asarray(gt["Time"], float)
    u_all = gt["UAV Pose"]
    n = min(len(tg), len(u_all))
    tg, u_all = tg[:n], u_all[:n]

    def _valid(p):
        if p is None:
            return False
        return all(np.isfinite(v) for v in (p.x, p.y, p.z, p.roll, p.pitch, p.yaw))

    keep = np.array([_valid(p) for p in u_all])
    tg = tg[keep]; u = [u_all[i] for i in range(n) if keep[i]]
    order = np.hstack(([True], np.diff(tg) > 1e-6))
    tg = tg[order]; u = [u[i] for i in range(len(u)) if order[i]]
    n = len(tg)
    if n < 20:
        raise ValueError(f"too few valid GT samples: {n}")

    pos_frd = np.zeros((n, 3)); yaw = np.zeros(n)
    for i, p in enumerate(u):
        pos_frd[i], _ = _pose_to_frd(p)
        yaw[i] = -np.deg2rad(p.yaw)
    vel_frd = _robust_vel(pos_frd, tg)
    speed = np.linalg.norm(vel_frd[:, :2], axis=1)   # horizontal translation speed
    yaw_unwrapped = np.unwrap(yaw)
    yaw_rate = _robust_vel(yaw_unwrapped[:, None], tg)[:, 0]
    return tg, St, speed, yaw_rate


def best_window(tg, speed, yaw_rate, speed_max, yaw_min, min_dur):
    ok = (speed <= speed_max) & (np.abs(yaw_rate) >= yaw_min)
    if not ok.any():
        return None
    # longest contiguous run of ok==True
    idx = np.where(ok)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.hstack(([idx[0]], idx[breaks + 1] if len(breaks) else []))
    ends = np.hstack((idx[breaks] if len(breaks) else [], idx[-1]))
    best = None
    for s, e in zip(starts, ends):
        dur = tg[e] - tg[s]
        if dur >= min_dur and (best is None or dur > best[2]):
            best = (s, e, dur)
    return best


def _slice_dict(d, t0_abs, t1_abs):
    """Group keys by list length, find each group's time key (name containing
    'Time' or 'Timestamp' or 'Stamp'), build a mask from it, apply to every
    same-length key. Scalars and empty/mismatched-length lists pass through
    unchanged (mismatched lengths are independent/low-rate logs not safely
    sliceable by this same mask; left as-is rather than guessed at)."""
    by_len = {}
    for k, v in d.items():
        if isinstance(v, list):
            by_len.setdefault(len(v), []).append(k)

    out = dict(d)
    for length, keys in by_len.items():
        if length == 0:
            continue
        time_keys = [k for k in keys if ("Time" in k or "Stamp" in k)]
        if not time_keys:
            continue   # no time reference for this length-group; leave untouched
        tk = time_keys[0]
        t = np.asarray(d[tk], float)
        mask = (t >= t0_abs) & (t <= t1_abs)
        if mask.sum() < 2:
            continue
        for k in keys:
            v = d[k]
            out[k] = [v[i] for i in range(len(v)) if mask[i]]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--speed-max", type=float, default=0.12, help="m/s, horizontal translation speed ceiling")
    ap.add_argument("--yaw-min", type=float, default=0.35, help="rad/s, yaw rate floor")
    ap.add_argument("--min-dur", type=float, default=3.0, help="seconds, minimum contiguous window duration")
    args = ap.parse_args()

    run_dir = args.run_dir
    tg, St, speed, yaw_rate = gt_speed_and_yawrate(run_dir)
    print(f"GT speed:   min={speed.min():.3f} median={np.median(speed):.3f} max={speed.max():.3f} m/s")
    print(f"GT yawrate: min={np.degrees(yaw_rate.min()):.1f} median={np.degrees(np.median(np.abs(yaw_rate))):.1f} "
          f"max={np.degrees(np.abs(yaw_rate)).max():.1f} deg/s")

    win = best_window(tg, speed, yaw_rate, args.speed_max, args.yaw_min, args.min_dur)
    if win is None:
        print(f"\nNo contiguous window >= {args.min_dur}s found meeting "
              f"speed<={args.speed_max} m/s AND |yaw_rate|>={args.yaw_min} rad/s.")
        print("Try relaxing --speed-max / --yaw-min / --min-dur.")
        return

    s, e, dur = win
    t0_rel, t1_rel = tg[s], tg[e]
    t0_abs, t1_abs = t0_rel + St, t1_rel + St
    print(f"\nBest window: t=[{t0_rel:.2f}, {t1_rel:.2f}]s (dur={dur:.2f}s), "
          f"speed<= {speed[s:e+1].max():.3f} m/s, yaw_rate in "
          f"[{np.degrees(yaw_rate[s:e+1].min()):.1f}, {np.degrees(yaw_rate[s:e+1].max()):.1f}] deg/s")

    out_dir = run_dir.rstrip("/\\") + "_PureYaw"
    os.makedirs(out_dir, exist_ok=True)
    # Ground_Truth.npy's OWN "Time"/"GT Pose Stamp"/"Img Time Stamp" are all
    # logged as (perf_counter() - start_time) -- RELATIVE to its own "Start
    # Time" (see output_calibration.py:162,185-187) -- while Img_Data.npy and
    # Telemetry_Data.npy stamp raw ABSOLUTE perf_counter() with no such
    # offset. Using the same window bounds for all three would silently mask
    # nothing (or the wrong slice) out of Ground_Truth.npy.
    windows = {"Ground_Truth.npy": (t0_rel, t1_rel),
               "Img_Data.npy": (t0_abs, t1_abs),
               "Telemetry_Data.npy": (t0_abs, t1_abs)}
    for fn, (w0, w1) in windows.items():
        path = os.path.join(run_dir, fn)
        if not os.path.exists(path):
            continue
        d = np.load(path, allow_pickle=True).item()
        trimmed = _slice_dict(d, w0, w1)
        np.save(os.path.join(out_dir, fn), trimmed, allow_pickle=True)
        _tk = "Time" if "Time" in d else next((k for k in d if "Timestamp" in k), None)
        n_before = len(d.get(_tk, [])) if _tk else "?"
        n_after = len(trimmed.get(_tk, [])) if _tk else "?"
        print(f"  {fn}: {n_before} -> {n_after} samples")
    print(f"\nSaved trimmed run to: {out_dir}")


if __name__ == "__main__":
    main()
