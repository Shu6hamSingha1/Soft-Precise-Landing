#!/usr/bin/env python3
"""Cross-correlation lag scan between GT (mocap) and raw image-pipeline
signals. Confirms/refutes a CONSTANT PROCESSING LAG between the two data
streams - distinct from the one-time startup-offset check already done
(img Time[0] - GT Start Time ~= 0.35s, which only anchors t=0, and says
nothing about whether the two streams drift or lag relative to each other
DURING the recording, e.g. from image-pipeline processing latency, QTM
network latency, or a clock-rate mismatch).

Method: resample both GT and raw signals onto a common UNIFORM time grid,
then for a range of candidate lags (seconds), shift the raw signal and
compute normalized cross-correlation against GT. If the correlation peaks
at a lag far from 0, that's a real, fixable synchronization offset - not
a data-quality problem. If it peaks at (or near) 0, timing sync is NOT the
explanation for the poor calibration fits, and the issue is elsewhere
(excitation mixing, mounting rotation, genuine noise floor).

Usage: python3 check_time_lag.py <run_dir> [<run_dir> ...]
"""
import sys
import numpy as np
from derive_pi_cal import compute_gt_flow, kf_filter_causal, reject_outliers, FLOW_KF_Q, FLOW_KF_R, GAP_EXCLUDE_S

LAG_RANGE_S = 2.0     # search +/- this many seconds
LAG_STEP_S = 0.02     # resolution
GRID_HZ = 30.0        # common uniform grid rate for the cross-correlation


def _bracket_width(t_query, t_src):
    """For each t_query[j], the width of the t_src interval np.interp would
    draw it from (t_src[idx-1], t_src[idx]) - large where t_src has a gap."""
    t_src = np.asarray(t_src, float)
    idx = np.clip(np.searchsorted(t_src, t_query), 1, len(t_src) - 1)
    return t_src[idx] - t_src[idx - 1]


def xcorr_lag_scan(t_gt, gt, t_raw, raw, lags):
    """Returns (lags, correlations) - correlation of raw shifted by each lag
    against gt, both resampled onto a common uniform grid first.

    Confirmed 2026-07-11 (check_loop_staleness.py + derive_pi_cal.py's
    gap_mask): raw's own time axis (t_raw) has frequent 1-18s marker-loss
    gaps. np.interp fabricates a straight-line fill across any such gap -
    at EVERY lag tried, not just lag=0 - so without masking, this scan was
    comparing GT against long runs of fabricated raw data, which is almost
    certainly why the best-fit lag bounced around inconsistently across
    runs and channels rather than converging on a fixed true offset (the
    thing this scan was actually trying to measure got swamped by gap
    fabrication noise). Grid points whose shifted query time (tu + lag)
    would draw raw_u from a gap >= GAP_EXCLUDE_S are now excluded from the
    correlation at THAT lag - the valid set shrinks/shifts per lag since
    the query time itself shifts by lag each iteration."""
    t0 = max(t_gt[0], t_raw[0]) + LAG_RANGE_S
    t1 = min(t_gt[-1], t_raw[-1]) - LAG_RANGE_S
    if t1 <= t0:
        return None, None
    tu = np.arange(t0, t1, 1.0 / GRID_HZ)
    gt_u = np.interp(tu, t_gt, gt)
    corrs = []
    for lag in lags:
        raw_u = np.interp(tu + lag, t_raw, raw)
        valid = _bracket_width(tu + lag, t_raw) < GAP_EXCLUDE_S
        if valid.sum() < 20 or np.std(gt_u[valid]) < 1e-9 or np.std(raw_u[valid]) < 1e-9:
            corrs.append(0.0)
            continue
        corrs.append(float(np.corrcoef(gt_u[valid], raw_u[valid])[0, 1]))
    return tu, np.array(corrs)


def main(run_dirs):
    lags = np.arange(-LAG_RANGE_S, LAG_RANGE_S + LAG_STEP_S, LAG_STEP_S)
    LAB = ["Hx", "Hy", "Hz"]

    for run_dir in run_dirs:
        print(f"\n=== {run_dir} ===")
        img = np.load(f"{run_dir}/Img_Data.npy", allow_pickle=True).item()
        g = compute_gt_flow(run_dir)

        t_img_abs = np.asarray(img["Time"], float)
        raw_flow = np.asarray(img["Opt Flow Ang Vel"], float)
        n = min(len(t_img_abs), len(raw_flow))
        t_img_abs, raw_flow = t_img_abs[:n], raw_flow[:n]
        raw_flow = reject_outliers(raw_flow)
        raw_flow = kf_filter_causal(raw_flow, t_img_abs, FLOW_KF_Q, FLOW_KF_R)

        # raw's own time axis, in the SAME origin as GT's t_g (seconds since
        # Start Time) - NOT g["align"]'s pre-resampled version, since we need
        # to shift raw's OWN timestamps to scan lags.
        t_raw_rel = t_img_abs - g["start_time"]

        all_corrs = []
        for k in range(3):
            tu, corrs = xcorr_lag_scan(g["t_g"], g["V_h_g"][:, k], t_raw_rel, raw_flow[:, k], lags)
            if tu is None:
                print(f"  {LAB[k]}: recording too short for a +/-{LAG_RANGE_S}s lag scan")
                all_corrs.append(None)
                continue
            finite = np.isfinite(corrs)
            if not np.any(finite):
                print(f"  {LAB[k]}: no finite correlations")
                all_corrs.append(None)
                continue
            best_i = np.nanargmax(np.abs(corrs))
            zero_i = int(np.argmin(np.abs(lags)))
            print(f"  {LAB[k]}: best |corr|={abs(corrs[best_i]):.3f} at lag={lags[best_i]:+.2f}s"
                  f"   (zero-lag corr={corrs[zero_i]:+.3f})")
            all_corrs.append(corrs)

        # COMBINED scan: the correct test for "one true lag, shared across
        # channels since they come from the same image pair" - sum each
        # channel's squared correlation at each lag (so channels with weak/
        # noisy signal don't get to veto a real shared lag, but also can't
        # spuriously dominate) and find the SINGLE lag maximizing the total.
        # This is the test that actually answers the hypothesis, not three
        # independent per-channel peaks (which is prone to each channel
        # finding its own spurious best-fit lag on noisy data).
        valid = [c for c in all_corrs if c is not None]
        if len(valid) == 3:
            combined = np.sum([c ** 2 for c in valid], axis=0)
            best_i = np.nanargmax(combined)
            zero_i = int(np.argmin(np.abs(lags)))
            print(f"  COMBINED (sum of corr^2, all 3 channels): "
                  f"best lag={lags[best_i]:+.2f}s (score={combined[best_i]:.3f}, "
                  f"zero-lag score={combined[zero_i]:.3f})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: check_time_lag.py <run_dir> [<run_dir> ...]")
        sys.exit(1)
    main(sys.argv[1:])
