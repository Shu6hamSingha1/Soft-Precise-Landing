"""Low-latency body-rate + thrust setpoints over uXRCE-DDS.

Replaces the MAVSDK hot path (offboard.set_attitude_rate -> MAVLink ->
mavsdk_server -> UDP :14540 -> PX4, ~30 ms) with a DIRECT DDS publish to PX4's
/fmu/in/vehicle_rates_setpoint (+ /fmu/in/offboard_control_mode heartbeat).
Same uORB topics MAVSDK ultimately feeds; only the transport changes — still
rate-mode. Cold path (arm/takeoff/offboard-entry/land) stays MAVSDK; this is
used only for the high-rate setpoints in the control loop (CMD_TRANSPORT=dds).

Integration: mirrors gz_subscriber's PROVEN pattern — a Node on the DEFAULT
rclpy context (already init'd by the pipeline) wrapped in its OWN
SingleThreadedExecutor spinning in a daemon thread. The 3 gz_subscribers do
exactly this and coexist fine; a 4th publisher node does too. (An isolated
private rclpy Context "worked" standalone but raced to "context invalid" in the
live multi-node pipeline; a lazily-created node with no executor deadlocked on
the shared context lock mid-spin. This pattern avoids both.)
"""
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint

DEG2RAD = math.pi / 180.0


class _RateNode(Node):
    def __init__(self):
        super().__init__("plasmc_dds_rate")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_ocm = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_rates = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos)


class DDSRateSender:
    """Publishes OffboardControlMode(body_rate) + VehicleRatesSetpoint."""

    def __init__(self, node_name="plasmc_dds_rate", time_keeper=None):
        if not rclpy.ok():            # pipeline normally init'd the default ctx already
            rclpy.init()
        self._node = _RateNode()
        self._time = time_keeper
        # Own executor + daemon thread (gz_subscriber pattern). A pub-only node
        # doesn't need to receive, but spinning keeps it consistent with the
        # working nodes and processes middleware events without contending the
        # other executors' locks during publish.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        print("[dds_setpoint] DDS rate sender up "
              "(/fmu/in/offboard_control_mode, /fmu/in/vehicle_rates_setpoint)")

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:
            pass

    def _ts_us(self):
        return int(self._node.get_clock().now().nanoseconds / 1000)

    def send_rates(self, roll_dps=0.0, pitch_dps=0.0, yaw_dps=0.0, thrust=0.0):
        """rates deg/s (FRD), thrust [0,1] — mirrors FC.send_attitude_rate.
        VehicleRatesSetpoint wants rad/s (FRD) + thrust_body[2] = -throttle."""
        ts = self._ts_us()

        ocm = OffboardControlMode()
        ocm.timestamp = ts
        ocm.position = False
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = True
        ocm.thrust_and_torque = False
        ocm.direct_actuator = False
        self._node.pub_ocm.publish(ocm)

        rs = VehicleRatesSetpoint()
        rs.timestamp = ts
        rs.roll = float(roll_dps * DEG2RAD)
        rs.pitch = float(pitch_dps * DEG2RAD)
        rs.yaw = float(yaw_dps * DEG2RAD)
        rs.thrust_body = [0.0, 0.0, float(-thrust)]
        rs.reset_integral = False
        self._node.pub_rates.publish(rs)

    def close(self):
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
