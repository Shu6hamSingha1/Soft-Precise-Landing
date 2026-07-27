#!/usr/bin/env python3
"""
Live-guided ISOLATED-TRANSLATION recording for the camera/marker lever-arm
certification (see check_lever_arm.py / project_pi_gt_lever_arm_2026_07_27).

Unlike output_calibration.py's free hand-sweep (excitation phases are
reconstructed AFTER the fact from mocap via derive_pi_cal.phase_labels()),
this script ENFORCES the phase structure LIVE, the same way a compass/IMU
calibration wizard does: it tells you which phase to fly right now, only
counts progress while your motion actually satisfies that phase's criteria,
and will not advance past a phase until it has accumulated enough clean
coverage. If you drift off-axis, drift yaw, lose the marker, or just stand
still during a sweep phase, progress PAUSES (does not reset) and the status
line tells you exactly which criterion is unmet - it resumes accumulating
the moment you fix it.

Phases, in order:
  1. settle1  - hold still, marker tracked (establishes a clean reference)
  2. sweep_x  - move in X only (Y/yaw/altitude held), marker tracked
  3. settle2  - hold still again
  4. sweep_y  - move in Y only (X/yaw/altitude held), marker tracked
  5. done

cam_z is deliberately NOT a phase here - it is structurally degenerate with
the bearing's own scale (see derive_pi_cal.py's R_CAM_FRD comment) and no
recording, however clean, can resolve it; only a physical measurement can.

Saves to Test_Data/Calibration/LeverArm/<timestamp>/ (NOT Output/ - this is
a distinct, narrower-purpose recording and mixing it into the ordinary
phased-excitation pool would dilute both). Point check_lever_arm.py /
derive_pi_cal.py at this folder explicitly.

Usage: python3 record_lever_arm.py
"""
import os
import sys
import time
import asyncio

import numpy as np

sys.path.insert(0, ".")

# Reuse output_calibration.py's env defaults (raw-flow checkposts off, manual
# exposure on, map ON) and its run-validity gate - this recording must be made
# under the SAME regime as the ordinary output-cal recording it complements,
# or its coast/margin numbers aren't comparable. Importing (not running) only
# executes output_calibration.py's top-level env-setdefault + import-time
# print; main()/save logic there are guarded by `if __name__ == "__main__"`,
# so nothing hardware-touching runs just from this import.
import output_calibration as OC

from flight_controller import FC
import img_data as ID
from mocaptools import QTMWrapper

QTM_IP = OC.QTM_IP

# ---- phase-gating thresholds (env-overridable) ----
SETTLE_S = float(os.environ.get("LEVER_SETTLE_S", "2.0"))
SWEEP_GOOD_S = float(os.environ.get("LEVER_SWEEP_GOOD_S", "6.0"))
MIN_SPEED_MS = float(os.environ.get("LEVER_MIN_SPEED_MS", "0.05"))
MAX_SETTLE_SPEED_MS = float(os.environ.get("LEVER_MAX_SETTLE_SPEED_MS", "0.03"))
ISOLATION_RATIO = float(os.environ.get("LEVER_ISOLATION_RATIO", "2.5"))
MAX_YAW_DRIFT_DEG = float(os.environ.get("LEVER_MAX_YAW_DRIFT_DEG", "8.0"))
MAX_ALT_DRIFT_M = float(os.environ.get("LEVER_MAX_ALT_DRIFT_M", "0.15"))

PHASES = ["settle1", "sweep_x", "settle2", "sweep_y", "done"]
INSTRUCTIONS = {
    "settle1": "SETTLE - hold the drone still, keep the marker in view.",
    "sweep_x": "SWEEP X - move the drone smoothly back and forth along X ONLY "
               "(no Y, no yaw, no altitude change). Keep the marker tracked.",
    "settle2": "SETTLE - hold still again before the Y sweep.",
    "sweep_y": "SWEEP Y - move the drone smoothly back and forth along Y ONLY "
               "(no X, no yaw, no altitude change). Keep the marker tracked.",
    "done": "DONE - all phases complete.",
}


class PhaseGate:
    """Tracks live pose + decode status and decides, per tick, whether the
    CURRENT phase's criteria are met - and if not, WHY, so the operator gets
    the same actionable feedback a compass-cal wizard gives ("hold still",
    "keep rotating") instead of finding out after the recording is over."""

    def __init__(self):
        self.phase_i = 0
        self.good_s = 0.0
        self.last_t = None
        self.ref_yaw = None
        self.ref_alt = None
        self.prev_pose = None
        self.prev_raw_flow = None

    @property
    def phase(self):
        return PHASES[self.phase_i]

    def _reset_phase(self):
        self.good_s = 0.0
        self.ref_yaw = None
        self.ref_alt = None

    def tick(self, pose, raw_flow_now, now=None):
        """pose: mocaptools.Pose (UAV), raw mocap x/y/z/yaw - deliberately NOT
        converted to body-FRD here. As long as yaw is one of THIS phase's own
        gated criteria (held near its reference value), raw mocap X/Y
        dominance is an adequate live proxy for body X/Y dominance; the
        precise FRD conversion happens offline in derive_pi_cal.py against the
        saved recording, which is the authoritative pass.

        raw_flow_now: img_node.getRawOptFlowAngVel() this tick, used ONLY to
        detect coast - bit-identical to the previous tick's raw value means
        held (not decoded this frame). This is the SAME frozen-during-coast
        signature independently confirmed on real recordings earlier this
        session (100% frozen during coast, 0% while tracking), not a new
        assumption introduced here.

        now: clock value for THIS tick, injectable for offline testing of the
        state machine without real sleeps (see the PhaseGate test block this
        was added for). Defaults to time.perf_counter() for real use.
        """
        now = time.perf_counter() if now is None else now
        dt = 0.0 if self.last_t is None else max(now - self.last_t, 1e-3)
        self.last_t = now

        tracked = self.prev_raw_flow is None or not np.array_equal(raw_flow_now, self.prev_raw_flow)
        self.prev_raw_flow = np.array(raw_flow_now, copy=True)

        if self.ref_yaw is None:
            self.ref_yaw = pose.yaw
            self.ref_alt = pose.z

        vx = vy = vz = 0.0
        if self.prev_pose is not None and dt > 0:
            vx = (pose.x - self.prev_pose.x) / dt
            vy = (pose.y - self.prev_pose.y) / dt
            vz = (pose.z - self.prev_pose.z) / dt
        self.prev_pose = pose

        yaw_drift = abs(((pose.yaw - self.ref_yaw + 180) % 360) - 180)
        alt_drift = abs(pose.z - self.ref_alt)

        ph = self.phase
        reasons = []
        ok = True

        if ph in ("settle1", "settle2"):
            speed = (vx ** 2 + vy ** 2 + vz ** 2) ** 0.5
            if speed > MAX_SETTLE_SPEED_MS:
                ok = False; reasons.append(f"hold still (speed {speed:.2f} m/s)")
            if not tracked:
                ok = False; reasons.append("marker not tracked")
        elif ph in ("sweep_x", "sweep_y"):
            main_v, off_v = (abs(vx), abs(vy)) if ph == "sweep_x" else (abs(vy), abs(vx))
            main_lbl, off_lbl = ("X", "Y") if ph == "sweep_x" else ("Y", "X")
            if main_v < MIN_SPEED_MS:
                ok = False
                reasons.append(f"move faster along {main_lbl} ({main_v:.2f} < {MIN_SPEED_MS:.2f} m/s)")
            elif main_v / max(off_v, 1e-6) < ISOLATION_RATIO:
                ok = False
                reasons.append(f"too much {off_lbl} motion (ratio {main_v/max(off_v,1e-6):.1f} "
                               f"< {ISOLATION_RATIO})")
            if yaw_drift > MAX_YAW_DRIFT_DEG:
                ok = False; reasons.append(f"yaw drifted {yaw_drift:.1f} deg (keep it fixed)")
            if alt_drift > MAX_ALT_DRIFT_M:
                ok = False; reasons.append(f"altitude drifted {alt_drift:.2f} m (keep it fixed)")
            if not tracked:
                ok = False; reasons.append("marker not tracked")

        need = SETTLE_S if ph.startswith("settle") else SWEEP_GOOD_S
        if ok:
            self.good_s += dt

        advanced = False
        if self.good_s >= need and ph != "done":
            self.phase_i += 1
            self._reset_phase()
            advanced = True

        return dict(phase=ph, ok=ok, reasons=reasons, good_s=min(self.good_s, need),
                    need_s=need, advanced=advanced, vx=vx, vy=vy)


async def main():
    UAV_pose, target_pose = [], []
    opt_flow_ang_vel, img_feature_param = [], []
    raw_opt_flow_ang_vel, raw_img_feature_param, raw_ring_flow_ang_vel = [], [], []
    centroid_map_raw, alpha_map_raw = [], []
    t_c, gt_pose_stamp, img_time_stamp = [], [], []
    start_time = time.perf_counter()
    start_pose = None

    FC_node = mocap_node = img_node = None
    gate = PhaseGate()
    _last_print = 0.0

    try:
        FC_node = FC()
        await FC_node.start()
        mocap_node = QTMWrapper(QTM_IP)

        start_time = time.perf_counter()
        while not FC_node.has_quat():
            if (time.perf_counter() - start_time) > 20:
                raise Exception("Unable to get data from Flight Controller.")
            await asyncio.sleep(0.05)

        _pose_wait_t0 = time.perf_counter()
        while start_pose is None:
            start_pose = mocap_node.getPose()["UAV"]
            if (time.perf_counter() - _pose_wait_t0) > 20:
                raise Exception("Unable to get UAV pose from QTM - check that the "
                                 "QTM project's rigid body is named 'UAV'.")
            await asyncio.sleep(0.05)

        img_node = ID.IMG_PROCESSOR(capRate=OC.CAPTURE_RATE, resolution=OC.RESOLUTION,
                                    controller=FC_node)
        print("Recording (no save prompt until all phases complete)...")
        print(f"\n>>> {INSTRUCTIONS['settle1']}\n")

        controller_ready = False
        while img_node.is_alive() and mocap_node.is_alive():
            if controller_ready:
                t_c.append(time.perf_counter() - start_time)
            else:
                start_time = time.perf_counter()
                controller_ready = True
                t_c = [0.0]

            pose = mocap_node.getPose()["UAV"]
            UAV_pose.append(pose)
            target_pose.append(mocap_node.getPose()["target"])
            opt_flow_ang_vel.append(img_node.getOptFlowAngVel())
            img_feature_param.append(img_node.getImgFeatureParam())
            raw_flow_now = img_node.getRawOptFlowAngVel()
            raw_opt_flow_ang_vel.append(raw_flow_now)
            raw_img_feature_param.append(img_node.getRawImgFeatureParam())
            raw_ring_flow_ang_vel.append(img_node.getRawRingFlowAngVel())
            centroid_map_raw.append(img_node.getRawCentroidMapFeature())
            alpha_map_raw.append(img_node.getRawAlphaMapFeature())
            gt_pose_stamp.append(
                (mocap_node.getPoseStamp() - start_time) if mocap_node.getPoseStamp() is not None else None)
            img_time_stamp.append(
                (img_node.getLastTimeStamp() - start_time) if img_node.getLastTimeStamp() is not None else None)

            phase_before = gate.phase
            status = gate.tick(pose, raw_flow_now)

            now = time.perf_counter()
            if now - _last_print >= 0.5:
                _last_print = now
                bar_n = int(20 * status["good_s"] / max(status["need_s"], 1e-6))
                bar = "#" * bar_n + "." * (20 - bar_n)
                if status["ok"]:
                    print(f"\r[{bar}] {status['good_s']:.1f}/{status['need_s']:.1f}s  "
                          f"OK  vx={status['vx']:+.2f} vy={status['vy']:+.2f}          ",
                          end="", flush=True)
                else:
                    print(f"\r[{bar}] {status['good_s']:.1f}/{status['need_s']:.1f}s  "
                          f"WAIT: {'; '.join(status['reasons'])}                    ",
                          end="", flush=True)

            if status["advanced"]:
                print(f"\n\n>>> phase '{phase_before}' complete.")
                if gate.phase != "done":
                    print(f">>> {INSTRUCTIONS[gate.phase]}\n")
                else:
                    print(">>> All phases complete - stopping.\n")
                    break

            await asyncio.sleep(OC.SLEEP_TIME)

        img_node.close()
        await FC_node.close()

    finally:
        image_data = telemetry_data = img_params = None
        if img_node is not None:
            try:
                image_data = img_node.getLogData()
                img_params = img_node.getParams()
            except Exception:
                pass
            if img_node.is_alive():
                img_node.close(); img_node.join()
        if FC_node is not None:
            try:
                telemetry_data = FC_node.getLogData()
            except Exception:
                pass
        if mocap_node is not None and mocap_node.is_alive():
            mocap_node.close(); mocap_node.join()

        gt_data = {"Start Time": start_time, "Time": t_c, "GT Pose Stamp": gt_pose_stamp,
                   "Img Time Stamp": img_time_stamp, "Start Pose": start_pose,
                   "UAV Pose": UAV_pose, "Target Pose": target_pose,
                   "Opt Flow Ang Vel": opt_flow_ang_vel, "Img Feature Params": img_feature_param,
                   "Raw Opt Flow Ang Vel": raw_opt_flow_ang_vel,
                   "Raw Img Feature Params": raw_img_feature_param,
                   "Raw Ring Flow Ang Vel": raw_ring_flow_ang_vel,
                   "Centroid Map Raw": centroid_map_raw, "Alpha Map Raw": alpha_map_raw,
                   "Command": []}

        if len(t_c) > 10 and image_data is not None:
            OC._print_run_validity(image_data, gt_data)
            x = input("Save this isolated-translation recording? (y/n) ")
            if x != "n":
                timestamp = time.ctime().replace(":", "-")
                dir_name = f"Test_Data/Calibration/LeverArm/{timestamp}"
                os.makedirs(dir_name, exist_ok=True)
                np.save(f"{dir_name}/Img_Data", image_data)
                np.save(f"{dir_name}/Telemetry_Data", telemetry_data)
                np.save(f"{dir_name}/Ground_Truth", gt_data)
                with open(f"{dir_name}/Img_Params.txt", "w") as f:
                    f.write(img_params)
                print(f"Saved to {dir_name}/")
            else:
                print("Closed without saving!")
        else:
            print("Recording too short - nothing saved.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nClean exit on Ctrl+C")
