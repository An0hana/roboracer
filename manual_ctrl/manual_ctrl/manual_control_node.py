"""Manual gamepad control node with a latching takeover.

The initial source is configurable. With ``initial_takeover`` enabled, manual
control owns the car at startup and the first X press releases it to autonomy.
Every later X press toggles between the two sources.

This node does not talk to the VESC directly. It publishes to the command
mux (manual_mux_node), which is the only node wired to the VESC command
topics. That is what makes the takeover exclusive: releasing autonomy is not
this node's job, the mux simply stops forwarding it.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float64

from .controller import ControlConfig, ManualController
from .gamepad import F710Gamepad


class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__("manual_ctrl")

        # --- parameters ---------------------------------------------------
        self.declare_parameter("gamepad_device", "/dev/input/event1")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("max_rpm", 5000)
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("smoothing", 0.15)
        # evdev reports changes, not a periodic heartbeat. A non-zero
        # inactivity timeout would therefore stop a valid held-stick command.
        # Physical disconnects are reported through GamepadSnapshot.connected.
        self.declare_parameter("stale_timeout", 0.0)

        # Motor command sent to the mux. Match your autonomous stack's units:
        # if ackermann_to_vesc publishes ERPM on /commands/motor/speed, keep
        # "speed"; if you drive duty_cycle, switch here and set duty_limit.
        self.declare_parameter("motor_mode", "speed")   # speed | duty_cycle | current
        self.declare_parameter("motor_scale", 1.0)
        self.declare_parameter("duty_limit", 0.20)
        self.declare_parameter("current_limit", 3.0)

        # Servo mapping, preserved from the original: center 0.48, +/- span.
        self.declare_parameter("servo_center", 0.48)
        self.declare_parameter("servo_span", 0.35)
        self.declare_parameter("servo_min", 0.15)
        self.declare_parameter("servo_max", 0.85)

        # Mux-facing topics.
        self.declare_parameter("manual_motor_topic", "/manual/motor")
        self.declare_parameter("manual_servo_topic", "/manual/servo")
        self.declare_parameter("takeover_topic", "/manual/takeover")
        self.declare_parameter("initial_takeover", True)

        g = self.get_parameter
        self._rate = float(g("rate").value)
        self._max_rpm = int(g("max_rpm").value)
        self._motor_mode = str(g("motor_mode").value)
        self._motor_scale = float(g("motor_scale").value)
        self._duty_limit = float(g("duty_limit").value)
        self._current_limit = float(g("current_limit").value)
        self._servo_center = float(g("servo_center").value)
        self._servo_span = float(g("servo_span").value)
        self._servo_min = float(g("servo_min").value)
        self._servo_max = float(g("servo_max").value)

        # --- control state ------------------------------------------------
        self._controller = ManualController(
            ControlConfig(
                max_rpm=self._max_rpm,
                deadzone=float(g("deadzone").value),
                smoothing=float(g("smoothing").value),
                stale_timeout=float(g("stale_timeout").value),
            )
        )
        self._takeover = bool(g("initial_takeover").value)
        self._prev_x = False

        # --- gamepad ------------------------------------------------------
        self._gamepad = F710Gamepad(str(g("gamepad_device").value))
        try:
            self._gamepad.start()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to open gamepad: {exc}")
            raise

        # --- publishers ---------------------------------------------------
        # The takeover flag is latched (transient_local) so the mux gets the
        # current state even if it starts after this node.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._motor_pub = self.create_publisher(
            Float64, str(g("manual_motor_topic").value), 10)
        self._servo_pub = self.create_publisher(
            Float64, str(g("manual_servo_topic").value), 10)
        self._takeover_pub = self.create_publisher(
            Bool, str(g("takeover_topic").value), latched)

        # Announce the configured initial state so the mux knows we exist.
        self._publish_takeover()

        period = 1.0 / max(1.0, self._rate)
        self._timer = self.create_timer(period, self._tick)

        initial_source = "MANUAL" if self._takeover else "AUTONOMOUS"
        next_source = "AUTONOMOUS" if self._takeover else "MANUAL"
        self.get_logger().info(
            f"manual_ctrl running. Initial source: {initial_source}; "
            f"press X on the F710 to switch to {next_source}."
        )

    # ---------------------------------------------------------------------
    def _publish_takeover(self) -> None:
        self._takeover_pub.publish(Bool(data=self._takeover))

    def _tick(self) -> None:
        pad = self._gamepad.snapshot()

        # Rising edge on X toggles takeover.
        x = pad.takeover_pressed
        if x and not self._prev_x:
            self._takeover = not self._takeover
            self._publish_takeover()
            if self._takeover:
                # Immediate stop the instant we grab control.
                self._controller.reset()
                self._motor_pub.publish(Float64(data=0.0))
                self._servo_pub.publish(
                    Float64(data=self._clip_servo(self._servo_center)))
                self.get_logger().warn("TAKEOVER engaged: car stopped, manual only.")
            else:
                # Release: park the wheels straight and stop before handing back.
                self._motor_pub.publish(Float64(data=0.0))
                self._servo_pub.publish(
                    Float64(data=self._clip_servo(self._servo_center)))
                self.get_logger().warn("Takeover released: autonomy may resume.")
        self._prev_x = x

        if not self._takeover:
            return  # silent in the background until X

        out = self._controller.update(pad)
        self._motor_pub.publish(Float64(data=self._motor_command(out.rpm)))
        servo = self._servo_center + out.steering * self._servo_span
        self._servo_pub.publish(Float64(data=self._clip_servo(servo)))

    # ---------------------------------------------------------------------
    def _motor_command(self, rpm: int) -> float:
        if self._motor_mode == "speed":
            return float(rpm) * self._motor_scale
        norm = 0.0
        if self._max_rpm > 0:
            norm = max(-1.0, min(1.0, float(rpm) / float(self._max_rpm)))
        if self._motor_mode == "duty_cycle":
            return norm * self._duty_limit
        if self._motor_mode == "current":
            return norm * self._current_limit
        # Unknown mode: safest is zero.
        return 0.0

    def _clip_servo(self, v: float) -> float:
        return max(self._servo_min, min(self._servo_max, v))

    def destroy_node(self) -> bool:
        try:
            # Best effort: stop the car. Do not release takeover here: if this
            # node crashes while the mux remains alive, keeping manual selected
            # lets the mux watchdog hold zero instead of exposing autonomy.
            self._motor_pub.publish(Float64(data=0.0))
        except Exception:  # noqa: BLE001
            pass
        try:
            self._gamepad.stop()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
