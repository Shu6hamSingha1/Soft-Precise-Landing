#!/usr/bin/env python3
"""
Record camera feed while running tests
Captures video + metadata for diagnostics
"""

import sys
import os
import time
import cv2
import subprocess
import threading
from datetime import datetime

sys.path.insert(0, ".")

from imgstreamer import imgstream


def record_camera_feed(duration=60, output_dir="Test_Data/Calibration/Test_Videos"):
    """Record camera feed to MP4 with timestamp"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(output_dir, f"test_feed_{timestamp}.mp4")
    
    print(f"Recording camera feed to: {output_file}")
    print(f"Duration: {duration}s")
    print(f"Resolution: 960×720")
    print(f"FPS: 30")
    print()
    
    # Initialize stream
    stream = imgstream(resolution=(960, 720), capRate=60)
    
    # Setup video writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_file, fourcc, 30.0, (960, 720))
    
    # Record
    start_time = time.time()
    frame_count = 0
    
    while time.time() - start_time < duration:
        imgs = stream.getImages()
        if imgs and len(imgs) > 0:
            frame = imgs[-1]
            if frame is not None:
                out.write(frame)
                frame_count += 1
                
                # Progress
                elapsed = time.time() - start_time
                if frame_count % 30 == 0:
                    print(f"  {elapsed:.1f}s / {duration}s — {frame_count} frames")
        
        time.sleep(0.001)
    
    out.release()
    stream.close()
    
    elapsed = time.time() - start_time
    fps = frame_count / elapsed
    file_size = os.path.getsize(output_file) / (1024 * 1024)
    
    print()
    print("✓ Recording complete")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Frames: {frame_count}")
    print(f"  FPS: {fps:.1f}")
    print(f"  File size: {file_size:.1f} MB")
    print(f"  Path: {output_file}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Record camera feed for testing")
    parser.add_argument("--duration", type=int, default=60, help="Recording duration in seconds")
    parser.add_argument("--output", default="Test_Data/Calibration/Test_Videos", help="Output directory")
    
    args = parser.parse_args()
    
    record_camera_feed(duration=args.duration, output_dir=args.output)

