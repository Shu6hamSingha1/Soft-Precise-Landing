# Use 6d_marks and 6d_euler_markers.
# Send telemetry data.
# This thread is created in ardupilot thread.

import asyncio
import os
from threading import Thread
import qtm
import time
import xml.etree.ElementTree as ET

# Found 2026-07-23 (GIL-contention diagnosis, output_calibration.py dry run):
# stream_frames() was called with its default frames='allframes' - QTM's own
# NATIVE capture rate (often 100-300+Hz), uncapped. Every packet fires
# _on_packet on QTMWrapper's own thread, competing for the single Python GIL
# with IMG_PROCESSOR's thread - confirmed live: ring_flow/aruco_detect (pure
# CPU-bound cv2 calls, unrelated to QTM) cost 3-4x MORE wall-clock time with
# QTM connected than without (7-8ms->19-28ms, 5ms->13-24ms), same computation,
# just starved of GIL time between calls. Nothing else consumes GT poses
# faster than the main poll loop's own rate (200Hz, output_calibration.py),
# so streaming faster than that is pure wasted GIL-contention with no benefit.
# qtm's own stream_frames() supports 'frequency:n' for exactly this - see its
# docstring. Env-overridable in case a future need wants a different cap.
QTM_STREAM_FREQUENCY = int(os.environ.get("QTM_STREAM_FREQUENCY_HZ", "100"))

# body_keys = ["UAV"]
body_keys = ["UAV", "target"]

def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index

class PoseData:
    """Simple class to hold UAV and target pose data"""
    def __init__(self):
        self.data = {}
    
    def __getitem__(self, key):
        return self.data.get(key)
    
    def __setitem__(self, key, value):
        self.data[key] = value
    
    def __len__(self):
        return len(self.data)

    def __bool__(self):
        return bool(self.data)
    
    def __repr__(self):
        return str(self.data)

class Pose:
    """Holds pose data with euler angles and/or rotation matrix"""
    def __init__(self, x, y, z, roll=None, pitch=None, yaw=None):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    @classmethod
    def from_qtm_6deuler(cls, qtm_6deuler):
        """Build pose from rigid body data in QTM 6deuler component"""
        return cls(qtm_6deuler[0].x / 1000,
                   qtm_6deuler[0].y / 1000,
                   qtm_6deuler[0].z / 1000,
                   roll  = qtm_6deuler[1].a3,
                   pitch = qtm_6deuler[1].a2,
                   yaw   = qtm_6deuler[1].a1)

class  QTMWrapper(Thread):
    """Run QTM Wrapper on its own thread."""
    def __init__(self, qtm_ip):
        Thread.__init__(self)
        self._connection = None
        self.connected = False
        self._stay_open = True
        self.qtm_ip = qtm_ip

        self._start_time = None
        self._pose = PoseData()
        # Real per-packet capture stamp, distinct from self._time (which is
        # relative to _start_time and, per the comment in _on_packet, had no
        # active callers anyway). A caller polling getPose() at its own rate
        # (e.g. output_calibration.py's 30 Hz loop) gets whatever pose was
        # LAST written here - which may be stale by up to one QTM inter-
        # packet gap relative to the poll instant. This stamp records when
        # THIS packet actually arrived (time.perf_counter(), same clock as
        # the rest of the pipeline) so a caller can tell how stale its poll
        # was, instead of conflating "when I polled" with "when QTM produced
        # this pose" - same class of fix as imgstreamer.py's Capture Stamp.
        self._pose_stamp = None
        self.start()
    
    def close(self):
        self._stay_open = False
    
    def run(self):
        asyncio.run(self._life_cycle())
    
    async def _life_cycle(self):
        await self._connect()
        # Found 2026-07-23 (GIL-contention pinpointing): this idle loop's only
        # job is noticing self._stay_open flip to False for shutdown - it was
        # waking 100x/sec UNCONDITIONALLY, for the entire connection lifetime,
        # completely independent of QTM_STREAM_FREQUENCY_HZ (that only caps
        # _on_packet's own call rate, not this). Every wake requires the
        # asyncio loop to run its own scheduling bookkeeping (GIL-acquiring
        # Python bytecode), competing with IMG_PROCESSOR's thread 100x/sec
        # regardless of actual packet traffic - capping the packet rate alone
        # never touched this. A recording session runs for many seconds; a
        # ~200ms shutdown-check latency here is imperceptible in practice.
        while self._stay_open:
            await asyncio.sleep(0.2)
        if self.connected:
            await self._close()

    async def _connect(self):
        host = self.qtm_ip
        print('Connecting to QTM on ' + host)
        self._connection = await qtm.connect(host)
        if not isinstance(self._connection, qtm.qrt.QRTConnection):
            self._stay_open = False
            return
        else: 
            self.connected = True

        # Get 6dof settings from qtm
        xml_string = await self._connection.get_parameters(parameters=["6d"])
        self._body_index = create_body_index(xml_string)

        # await self._connection.stream_frames(components=['6d'], on_packet=self._on_packet)
        # frames=f'frequency:{...}' caps QTM's stream rate - see QTM_STREAM_FREQUENCY
        # comment above (was the default 'allframes' = QTM's uncapped native rate).
        await self._connection.stream_frames(
            frames=f'frequency:{QTM_STREAM_FREQUENCY}',
            components=['6deuler'], on_packet=self._on_packet)
    
    def _on_packet(self, packet):
        if self._start_time is None:
            self._start_time = packet.timestamp
            self._time = 0
        else:
            # QTM packet.timestamp is microseconds; convert to seconds for
            # consistency with the rest of the pipeline (time.perf_counter()-based).
            # Was \ - Python XOR, not exponentiation - evaluated to *12, a
            # near-no-op bug. Not currently read by any active script (getTime()
            # has no callers), fixed while noticed.
            self._time = (packet.timestamp - self._start_time) / 1e6
        # _, markers = packet.get_6d()
        _, bodies = packet.get_6d_euler()

        for key in body_keys:
            if key is not None and key in self._body_index:
                # Extract one specific body
                key_idx = self._body_index[key]
                self._pose[key] = Pose.from_qtm_6deuler(bodies[key_idx])
            else:
                print(f"{key} not found in MoCap...")
        self._pose_stamp = time.perf_counter()
        # REMOVED 2026-07-23: a synchronous time.sleep(0.01) used to sit here.
        # _on_packet is invoked from the asyncio event loop (via qtm's
        # stream_frames callback) - a blocking sleep here doesn't rate-limit
        # QTM cleanly, it stalls the ENTIRE event loop for 10ms on every
        # single mocap packet, including output_calibration.py's main poll
        # loop (now 200 Hz, see its SLEEP_TIME comment) and anything else
        # running concurrently (FC telemetry, image processing callbacks).
        # No comment ever justified it as intentional, unlike every other
        # deliberate sleep in this codebase - looks like an accidental
        # throttle, not a design choice.

    def getPose(self):
        return self._pose

    def getPoseStamp(self):
        """time.perf_counter() when the CURRENT self._pose was actually
        written by _on_packet - i.e. the real capture instant of the pose a
        caller gets from getPose(), not the caller's own poll time."""
        return self._pose_stamp

    def getTime(self):
        return self._time

    def help(self):
        print("The required modules can be imported with: \n")
        print("1. import asyncio \n2. from threading import Thread")
        print("3. import numpy as np \n4. import qtm")
        self.close()

    async def _close(self):
        await self._connection.stream_frames_stop()
        self._connection.disconnect()