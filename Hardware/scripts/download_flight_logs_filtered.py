#!/usr/bin/env python3
"""
Download PX4 .ulg flight logs with optional time filtering.

Usage:
  LOG_REMOTE_DATE_DIR=/fs/microsd/log/2026-08-03 LOG_MIN_TIME_HMS=16:34:00 python3 download_flight_logs_filtered.py
"""
import asyncio
import os
from mavsdk import System

LOG_OUT_DIR = os.environ.get("LOG_OUT_DIR", "Test_Data/FlightLogs")
LOG_REMOTE_DATE_DIR = os.environ.get("LOG_REMOTE_DATE_DIR", "")
LOG_MIN_TIME_HMS = os.environ.get("LOG_MIN_TIME_HMS", "")
LOG_ROOT = "/fs/microsd/log"
LOCK_FILE = os.environ.get("LOG_DOWNLOAD_LOCK_FILE", "Test_Data/.log_download_active")

def time_to_seconds(hms_str):
    try:
        h, m, s = map(int, hms_str.split(":"))
        return h * 3600 + m * 60 + s
    except:
        return 0

def filename_to_seconds(fname):
    try:
        base = fname.replace(".ulg", "")
        h, m, s = map(int, base.split("_"))
        return h * 3600 + m * 60 + s
    except:
        return 0

async def _run():
    vehicle = System()
    print("Connecting to Flight Controller...")
    await vehicle.connect(system_address="serial:///dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00:57600")

    async for state in vehicle.core.connection_state():
        if state.is_connected:
            print("-- Connected!")
            break

    min_time_seconds = time_to_seconds(LOG_MIN_TIME_HMS) if LOG_MIN_TIME_HMS else 0
    if LOG_MIN_TIME_HMS:
        print(f"Filtering: only files >= {LOG_MIN_TIME_HMS}")

    date_dirs = []
    if LOG_REMOTE_DATE_DIR:
        date_dirs = [LOG_REMOTE_DATE_DIR]
    else:
        print(f"Listing date directories under {LOG_ROOT}...")
        try:
            listing = await vehicle.ftp.list_directory(LOG_ROOT)
        except Exception as ex:
            print(f"[FAILED] list_directory: {ex}")
            return
        date_dirs = [f"{LOG_ROOT}/{d}" for d in listing.dirs if d[0].isdigit()]

    os.makedirs(LOG_OUT_DIR, exist_ok=True)
    for date_dir in date_dirs:
        print(f"\nListing {date_dir}...")
        try:
            listing = await vehicle.ftp.list_directory(date_dir)
        except Exception as ex:
            print(f"  [FAILED] {ex}")
            continue
        ulg_files = [f for f in listing.files if f.endswith(".ulg")]
        print(f"  Found {len(ulg_files)} .ulg files")

        if min_time_seconds > 0:
            ulg_files = [f for f in ulg_files if filename_to_seconds(f) >= min_time_seconds]
            print(f"  Filtered to {len(ulg_files)} files >= {LOG_MIN_TIME_HMS}")

        date_name = os.path.basename(date_dir.rstrip("/"))
        out_subdir = os.path.join(LOG_OUT_DIR, date_name)
        os.makedirs(out_subdir, exist_ok=True)

        for fname in ulg_files:
            remote_path = f"{date_dir}/{fname}"
            print(f"  Downloading {fname}...")
            try:
                async for progress in vehicle.ftp.download(remote_path, out_subdir, True):
                    pass
                print(f"    done")
            except Exception as ex:
                print(f"    [FAILED] {ex}")

async def main():
    os.makedirs(os.path.dirname(LOCK_FILE) or ".", exist_ok=True)
    with open(LOCK_FILE, "w") as f:
        f.write(f"log download started\n")
    print(f"[LOCK] created")
    try:
        await _run()
    finally:
        try:
            os.remove(LOCK_FILE)
            print(f"[LOCK] removed")
        except FileNotFoundError:
            pass

if __name__ == "__main__":
    asyncio.run(main())
