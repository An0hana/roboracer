"""Command multiplexer: the only node wired to the VESC command topics.

Two input sources feed it:
  - autonomous: whatever your stack already publishes (remapped in on launch)
  - manual: the gamepad node

When the latched takeover flag is true, the mux forwards ONLY the manual
source and drops autonomous commands entirely. This is what makes manual
control exclusive -- the autonomous nodes keep publishing, but their commands
stop at the mux instead of reaching the VESC.

A watchdog zeroes the motor if the active source goes quiet, so a crashed
controller or an unplugged gamepad coasts to a stop rather than latching the
last command.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float64


class ManualMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("manual_mux")

        self.declare_parameter("autonomous_motor_topic", "/autonomous/motor")
        self.declare_parameter("autonomous_servo_topic", "/autonomous/servo")
        self.declare_parameter("manual_motor_topic", "/manual/motor")
        self.declare_parameter("manual_servo_topic", "/manual/servo")
        self.declare_parameter("takeover_topic", "/manual/takeover")
        self.declare_parameter("initial_takeover", True)

        # Output: the real VESC command topics.
        self.declare_parameter("motor_out_topic", "/commands/motor/speed")
        self.declare_parameter("servo_out_topic", "/commands/servo/position")

        # Watchdog: zero the motor if the active source is silent this long.
        self.declare_parameter("command_timeout", 0.3)
        self.declare_parameter("servo_center", 0.48)

        g = self.get_parameter
        self._timeout = float(g("command_timeout").value)
        self._servo_center = float(g("servo_center").value)

        self._takeover = bool(g("initial_takeover").value)
        self._last_motor_stamp = self.get_clock().now()

        # --- outputs ------------------------------------------------------
        self._motor_out = self.create_publisher(
            Float64, str(g("motor_out_topic").value), 10)
        self._servo_out = self.create_publisher(
            Float64, str(g("servo_out_topic").value), 10)

        # --- inputs -------------------------------------------------------
        self.create_subscription(
            Float64, str(g("autonomous_motor_topic").value),
            self._auto_motor_cb, 10)
        self.create_subscription(
            Float64, str(g("autonomous_servo_topic").value),
            self._auto_servo_cb, 10)
        self.create_subscription(
            Float64, str(g("manual_motor_topic").value),
            self._manual_motor_cb, 10)
        self.create_subscription(
            Float64, str(g("manual_servo_topic").value),
            self._manual_servo_cb, 10)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, str(g("takeover_topic").value), self._takeover_cb, latched)

        self._watchdog = self.create_timer(0.05, self._check_watchdog)

        initial_source = "MANUAL" if self._takeover else "AUTONOMOUS"
        self.get_logger().info(
            f"manual_mux running. Initial source: {initial_source}.")

    # --- takeover ---------------------------------------------------------
    def _takeover_cb(self, msg: Bool) -> None:
        if msg.data == self._takeover:
            return
        self._takeover = msg.data
        # On any switch, stop the motor so neither source's stale last command
        # carries across the handover.
        self._motor_out.publish(Float64(data=0.0))
        self._last_motor_stamp = self.get_clock().now()
        self.get_logger().warn(
            "Source switched to " + ("MANUAL" if self._takeover else "AUTONOMOUS"))

    # --- autonomous source ------------------------------------------------
    def _auto_motor_cb(self, msg: Float64) -> None:
        if self._takeover:
            return  # locked out
        self._motor_out.publish(msg)
        self._last_motor_stamp = self.get_clock().now()

    def _auto_servo_cb(self, msg: Float64) -> None:
        if self._takeover:
            return
        self._servo_out.publish(msg)

    # --- manual source ----------------------------------------------------
    def _manual_motor_cb(self, msg: Float64) -> None:
        if not self._takeover:
            return  # manual node is idle / not in control
        self._motor_out.publish(msg)
        self._last_motor_stamp = self.get_clock().now()

    def _manual_servo_cb(self, msg: Float64) -> None:
        if not self._takeover:
            return
        self._servo_out.publish(msg)

    # --- watchdog ---------------------------------------------------------
    def _check_watchdog(self) -> None:
        age = (self.get_clock().now() - self._last_motor_stamp).nanoseconds * 1e-9
        if age > self._timeout:
            # Active source went quiet: coast to stop. Republish so the VESC's
            # own command timeout never trips on a stale value.
            self._motor_out.publish(Float64(data=0.0))
            self._last_motor_stamp = self.get_clock().now()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._motor_out.publish(Float64(data=0.0))
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
