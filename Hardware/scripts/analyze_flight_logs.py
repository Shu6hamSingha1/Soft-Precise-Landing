"""Quick summary of today's bench-test .ulg flight logs -- arming state,
failsafe flags, and estimator health, to correlate against the terminal
output pasted during the 2026-07-30 CBF_CORNERS_STALE bench-test session.

Not a general-purpose tool: scoped to whatever topics are present, prints a
human-readable summary rather than full CSV export (pyulog installed here is
an older version without the ulog2csv CLI script, only the core ULog class).

Usage: python analyze_flight_logs.py <dir_with_ulg_files>
"""
import sys
import os
import glob
from pyulog import ULog

TOPICS = ["vehicle_status", "failsafe_flags", "estimator_status",
          "vehicle_local_position", "battery_status"]


def summarize(path):
    print(f"\n{'='*70}\n{os.path.basename(path)}\n{'='*70}")
    try:
        log = ULog(path, message_name_filter_list=TOPICS)
    except Exception as e:
        print(f"  [FAILED to parse] {e}")
        return

    if not log.data_list:
        print("  No matching topics found.")
        return

    t0 = log.start_timestamp
    duration_s = (log.last_timestamp - log.start_timestamp) / 1e6
    print(f"  Duration: {duration_s:.1f}s")

    for d in log.data_list:
        name = d.name
        fields = d.data
        n = len(fields.get("timestamp", []))
        if name == "vehicle_status" and n:
            arming = fields.get("arming_state")
            nav = fields.get("nav_state")
            if arming is not None:
                changes = [(i, arming[i]) for i in range(1, n) if arming[i] != arming[i-1]]
                print(f"  vehicle_status: {n} samples, arming_state changes: "
                      f"{[(round((fields['timestamp'][i]-t0)/1e6,1), v) for i, v in changes][:10]}")
        elif name == "failsafe_flags" and n:
            # Print any sample where a flag is nonzero, first few only
            keys = [k for k in fields.keys() if k != "timestamp"]
            flagged = []
            for i in range(n):
                active = [k for k in keys if fields[k][i]]
                if active:
                    flagged.append((round((fields["timestamp"][i]-t0)/1e6, 1), active))
            print(f"  failsafe_flags: {n} samples, {len(flagged)} with any flag set")
            for t_s, active in flagged[:5]:
                print(f"    t={t_s}s: {active}")
        elif name == "estimator_status" and n:
            gps_fail = fields.get("gps_check_fail_flags")
            print(f"  estimator_status: {n} samples"
                  + (f", nonzero gps_check_fail_flags at "
                     f"{sum(1 for v in gps_fail if v)} samples" if gps_fail is not None else ""))
        elif name == "battery_status" and n:
            volt = fields.get("voltage_v")
            if volt is not None and len(volt):
                print(f"  battery_status: {n} samples, voltage range "
                      f"{min(volt):.2f}V - {max(volt):.2f}V")
        else:
            print(f"  {name}: {n} samples")


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    files = sorted(glob.glob(os.path.join(d, "*.ulg")))
    if not files:
        print(f"No .ulg files in {d}")
        return
    for f in files:
        summarize(f)


if __name__ == "__main__":
    main()
