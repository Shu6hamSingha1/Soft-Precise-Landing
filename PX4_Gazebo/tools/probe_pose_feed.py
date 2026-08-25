#!/usr/bin/env python3
"""Raw probe on the bridged /pose PoseArray — diagnoses the 2026-08-20 GT freeze.

Background: 23/56 GT-FB reps showed Ground_Truth.npy's UAV Pose going
BIT-IDENTICAL for the final ~22 s at exactly alt=0.4860 m (std 0.0000 across
reps), while /clock kept advancing and PX4 telemetry showed the drone still
airborne and creeping BELOW that altitude. Two hypotheses remained:

  (A) the feed stops delivering new messages (bridge/Gazebo publish stall) and
      Pose_Node silently holds its last value -- pose_callback assigns
      self._pose.UAV unconditionally and getPose() has NO staleness tracking,
      so a dead feed is indistinguishable from a still drone downstream; or
  (B) the drone is genuinely, exactly motionless and the identical values are
      real.

This probe discriminates them by recording, per RAW message: wall time, message
count, len(poses), and poses[UAV_IDX] -- so we can see whether messages KEEP
ARRIVING while their CONTENT stops changing (=> B, real stillness) or stop
arriving altogether (=> A, dead feed). It also watches len(poses), since
Pose_Node indexes a FIXED index (POSE_IDX_UAV=2) into the array: if the array
membership/length ever changes, index 2 silently starts denoting a DIFFERENT
entity (an IndexError would at least print, but a shorter-but-still-valid array
would not).

Run it alongside a landing:  python3 tools/probe_pose_feed.py --out probe.csv
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


class Probe(Node):
    def __init__(self, out, uav_idx):
        super().__init__('pose_feed_probe')
        self._out = open(out, 'w', buffering=1)
        self._out.write('wall_t,msg_n,n_poses,uav_x,uav_y,uav_z,all_z...\n')
        self._uav_idx = uav_idx
        self._n = 0
        self._t0 = time.perf_counter()
        # Same QoS depth as Pose_Node so we observe what it would observe.
        self.create_subscription(PoseArray, '/pose', self._cb, 10)

    def _cb(self, msg):
        self._n += 1
        n = len(msg.poses)
        if n > self._uav_idx:
            p = msg.poses[self._uav_idx].position
            x, y, z = p.x, p.y, p.z
        else:
            x = y = z = float('nan')
        # ALL entity z's (2026-08-21): Pose_Node reads a FIXED index, so a frozen
        # index-2 is ambiguous between "the drone stopped" and "the drone moved to
        # a different index". Logging every z disambiguates: if ANY index is still
        # moving while index 2 is frozen, the array re-ordered under us.
        allz = ','.join(f'{q.position.z:.6f}' for q in msg.poses)
        self._out.write(f'{time.perf_counter()-self._t0:.4f},{self._n},{n},'
                        f'{x:.6f},{y:.6f},{z:.6f},{allz}\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='pose_probe.csv')
    ap.add_argument('--uav-idx', type=int, default=2)
    a = ap.parse_args()
    rclpy.init()
    node = Probe(a.out, a.uav_idx)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._out.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
