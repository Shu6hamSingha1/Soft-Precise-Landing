# Use 6d_marks and 6d_euler_markers.
# Send telemetry data.
# This thread is created in ardupilot thread.

import asyncio
from threading import Thread
import qtm
import time
import xml.etree.ElementTree as ET

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
        self.start()
    
    def close(self):
        self._stay_open = False
    
    def run(self):
        asyncio.run(self._life_cycle())
    
    async def _life_cycle(self):
        await self._connect()
        while self._stay_open:
            await asyncio.sleep(0.01)
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
        await self._connection.stream_frames(components=['6deuler'], on_packet=self._on_packet)
    
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
        time.sleep(0.01)
    
    def getPose(self):
        return self._pose

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