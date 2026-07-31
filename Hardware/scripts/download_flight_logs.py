#!/usr/bin/env python3
"""
Download PX4 .ulg flight logs from the flight controller's own SD card over
the existing MAVSDK connection (no arming, read-only).

2026-07-11 rewrite: the log-specific plugin (mavsdk.log_files.get_entries())
times out on this link ("TIMEOUT: 'Timeout'; origin: get_entries()") --
a documented MAVSDK issue (chunk retransmission/timeout bugs, see MAVSDK
PR #2654 / PX4 issue #14795), same underlying LOG_REQUEST_LIST protocol QGC
uses but apparently more fragile in MAVSDK's implementation on this link.
Switched to mavsdk.ftp (MAVLink FTP), which lists/downloads by path directly
(list_directory/download) -- a different, simpler protocol, same one behind
the "/fs/microsd/log/..." paths already seen in PX4 console messages.

2026-07-11: added a lock-file guard (LOCK_FILE) -- a stuck/running MAVSDK log
download can interfere with a new flight starting (documented MAVSDK/PX4
behavior). This script now writes the lock file right after connecting and
ALWAYS removes it in a finally block (covers normal completion, exceptions,
and Ctrl-C/CancelledError), so flight_controller.py's arm_and_takeoff() can
check for it and refuse to arm while a download is in progress. Don't rely on
manually remembering "don't fly during a download" -- this makes it a hard
stop instead. If a download crashes hard enough to skip even the finally
block (e.g. SIGKILL), delete the lock file manually before flying:
  rm Test_Data/.log_download_active

Usage:
  /home/doctor/denv/bin/python3 -u download_flight_logs.py
  LOG_REMOTE_DATE_DIR=/fs/microsd/log/2026-07-10 /home/doctor/denv/bin/python3 -u download_flight_logs.py
  LOG_OUT_DIR=Test_Data/FlightLogs /home/doctor/denv/bin/python3 -u download_flight_logs.py
"""
import asyncio
import os

from mavsdk import System

LOG_OUT_DIR = os.environ.get("LOG_OUT_DIR", "Test_Data/FlightLogs")
LOG_REMOTE_DATE_DIR = os.environ.get("LOG_REMOTE_DATE_DIR", "")
LOG_ROOT = "/fs/microsd/log"
LOCK_FILE = os.environ.get("LOG_DOWNLOAD_LOCK_FILE", "Test_Data/.log_download_active")


async def _run():
    vehicle = System()
    print("Connecting to Flight Controller...")
    await vehicle.connect(system_address="serial:///dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00:57600")

    async for state in vehicle.core.connection_state():
        if state.is_connected:
            print("-- Connected!")
            break

    date_dirs = []
    if LOG_REMOTE_DATE_DIR:
        date_dirs = [LOG_REMOTE_DATE_DIR]
    else:
        print(f"Listing date directories under {LOG_ROOT} ...")
        try:
            listing = await vehicle.ftp.list_directory(LOG_ROOT)
        except Exception as ex:
            print(f"[FAILED] list_directory({LOG_ROOT}): {ex}")
            return
        print(f"  raw dirs: {listing.dirs}")
        date_dirs = [f"{LOG_ROOT}/{d}" for d in listing.dirs if d[0].isdigit()]
        print(f"Found {len(date_dirs)} date directories: {date_dirs}")

    os.makedirs(LOG_OUT_DIR, exist_ok=True)
    for date_dir in date_dirs:
        print(f"\nListing files in {date_dir} ...")
        try:
            listing = await vehicle.ftp.list_directory(date_dir)
        except Exception as ex:
            print(f"  [FAILED] list_directory({date_dir}): {ex}")
            continue
        ulg_files = [f for f in listing.files if f.endswith(".ulg")]
        print(f"  {len(ulg_files)} .ulg files: {ulg_files}")

        # BUG FIX (2026-07-30): logs used to be saved flat into LOG_OUT_DIR keyed only by
        # HH_MM_SS.ulg, the PX4-assigned filename WITHIN a date directory -- two different
        # dates producing a log at the same clock time (confirmed live: a 2026-07-30 recording
        # collided with a pre-existing 2026-07-20 file of the same name) silently overwrote
        # one date's data with another's, with no warning. Each date now gets its own
        # subdirectory (LOG_OUT_DIR/<date>/HH_MM_SS.ulg) so collisions are structurally
        # impossible across dates; a same-date re-download overwriting itself is still fine
        # (that's the same flight's log, not a different one).
        date_name = os.path.basename(date_dir.rstrip("/"))
        out_subdir = os.path.join(LOG_OUT_DIR, date_name)
        os.makedirs(out_subdir, exist_ok=True)

        for fname in ulg_files:
            remote_path = f"{date_dir}/{fname}"
            print(f"  Downloading {remote_path} -> {out_subdir}/ ...")
            try:
                async for progress in vehicle.ftp.download(remote_path, out_subdir, True):
                    pass
                print(f"    done: {fname}")
            except Exception as ex:
                print(f"    [FAILED] {fname}: {ex}")

    print("\nAll downloads attempted. Use pyulog / ulog2csv to inspect EKF status,"
          " e.g.: ulog2csv <file>.ulg -o out_dir (look for estimator_status,"
          " vehicle_local_position, and failsafe_flags topics).")


async def main():
    os.makedirs(os.path.dirname(LOCK_FILE) or ".", exist_ok=True)
    with open(LOCK_FILE, "w") as f:
        f.write(f"log download started, pid={os.getpid()}\n")
    print(f"[LOCK] {LOCK_FILE} created -- flight scripts will refuse to arm until this is removed.")
    try:
        await _run()
    finally:
        try:
            os.remove(LOCK_FILE)
            print(f"[LOCK] {LOCK_FILE} removed.")
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    asyncio.run(main())
