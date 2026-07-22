import math
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .can_interface import CANInterface
from .motor import Motor


class MotorControllerNode(Node):
    """The only process allowed to access the drive and steering motors over CAN."""

    STEER_CONFIG_NAMES = {
        "steer_pos_max_speed_rpm",
        "steer_speed_acceleration_rpm_s",
        "steer_pos_kp",
        "steer_pos_ki",
        "steer_speed_kp",
        "steer_speed_ki",
        "steer_max_q_current_a",
    }

    def __init__(self):
        super().__init__("motor_controller_node")

        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("drive_addr", 1)
        self.declare_parameter("steer_addr", 2)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("cmd_timeout", 0.2)

        # Motor-internal steering-loop parameters. These are B2/B3/B5-B9.
        self.declare_parameter("steer_pos_max_speed_rpm", 80.0)
        self.declare_parameter("steer_speed_acceleration_rpm_s", 180.0)
        self.declare_parameter("steer_pos_kp", 1.0)
        self.declare_parameter("steer_pos_ki", 0.0)
        self.declare_parameter("steer_speed_kp", 3.0)
        self.declare_parameter("steer_speed_ki", 0.5)
        self.declare_parameter("steer_max_q_current_a", 8.0)

        self._can_channel = self.get_parameter("can_channel").value
        self._drive_addr = self.get_parameter("drive_addr").value
        self._steer_addr = self.get_parameter("steer_addr").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self._lock = threading.Lock()
        self._target_rpm = None
        self._target_steer_deg = None
        self._last_cmd_time = time.monotonic()
        self._watchdog_triggered = False
        # Only parameters explicitly changed through ROS are written to hardware.
        self._pending_steer_config_names = set()

        self._can = None
        self._drive_motor = None
        self._steer_motor = None
        self._can_ok = False
        self._can_timeout_configured = False

        self.state_pub = self.create_publisher(JointState, "/motor/state", 10)
        self.cmd_sub = self.create_subscription(
            JointState, "/motor/command", self.command_callback, 10
        )
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(
            f"Combined Motor Node Started (Drive ID: {self._drive_addr}, "
            f"Steer ID: {self._steer_addr})"
        )

    def _on_parameters_changed(self, parameters):
        for parameter in parameters:
            if parameter.name not in self.STEER_CONFIG_NAMES:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False, reason=f"{parameter.name} must be numeric"
                )
            if not math.isfinite(value) or value < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} must be finite and non-negative",
                )
        changed_names = {
            p.name for p in parameters if p.name in self.STEER_CONFIG_NAMES
        }
        if changed_names:
            with self._lock:
                self._pending_steer_config_names.update(changed_names)
        return SetParametersResult(successful=True)

    def _ensure_can_open(self):
        if self._can_ok:
            return True
        try:
            self._can = CANInterface(channel=self._can_channel, baudrate=1_000_000)
            self._can.open()
            self._drive_motor = Motor(
                self._can, dev_addr=self._drive_addr, timeout=self._cmd_timeout
            )
            self._steer_motor = Motor(
                self._can, dev_addr=self._steer_addr, timeout=self._cmd_timeout
            )
            self._drive_motor.enable()
            self._steer_motor.enable()
            self._can_ok = True
            self._can_timeout_configured = False
            self.get_logger().info("CAN bus opened; motors initialized")
            return True
        except Exception as error:
            self.get_logger().warning(f"CAN not available yet: {error}")
            if self._can is not None:
                try:
                    self._can.close()
                except Exception:
                    pass
            self._can = None
            self._drive_motor = None
            self._steer_motor = None
            self._can_ok = False
            return False

    def command_callback(self, message):
        with self._lock:
            for index, name in enumerate(message.name):
                if name in ("drive_motor", "motor_1") and len(message.velocity) > index:
                    self._target_rpm = float(message.velocity[index]) * 60.0 / (2.0 * math.pi)
                    self._last_cmd_time = time.monotonic()
                    self._watchdog_triggered = False
                elif name in ("steer_motor", "motor_2") and len(message.position) > index:
                    self._target_steer_deg = math.degrees(float(message.position[index]))

    def _apply_steer_config(self, names):
        """Run only from timer_callback so no two callbacks access CAN concurrently."""
        values = {
            name: float(self.get_parameter(name).value)
            for name in self.STEER_CONFIG_NAMES
        }
        motor = self._steer_motor
        readbacks = []
        if "steer_pos_max_speed_rpm" in names:
            motor.position_max_speed = int(values["steer_pos_max_speed_rpm"] / 0.01)
            readbacks.append(f"max_speed={values['steer_pos_max_speed_rpm']:g} Rpm")
        if "steer_speed_acceleration_rpm_s" in names:
            motor.speed_acceleration = int(
                values["steer_speed_acceleration_rpm_s"] / 0.01
            )
            readbacks.append(
                f"accel={values['steer_speed_acceleration_rpm_s']:g} Rpm/s"
            )
        if "steer_pos_kp" in names:
            motor.pos_loop_kp = values["steer_pos_kp"]
            readbacks.append(f"KP={values['steer_pos_kp']:g}")
        if "steer_pos_ki" in names:
            motor.pos_loop_ki = values["steer_pos_ki"]
            readbacks.append(f"KI={values['steer_pos_ki']:g}")
        if "steer_speed_kp" in names:
            motor.speed_loop_kp = values["steer_speed_kp"]
            readbacks.append(f"speed_KP={values['steer_speed_kp']:g}")
        if "steer_speed_ki" in names:
            motor.speed_loop_ki = values["steer_speed_ki"]
            readbacks.append(f"speed_KI={values['steer_speed_ki']:g}")
        if "steer_max_q_current_a" in names:
            motor.max_q_current = int(values["steer_max_q_current_a"] / 0.001)
            readbacks.append(f"max_current={values['steer_max_q_current_a']:g} A")
        self.get_logger().info(
            "Steer config write acknowledged: " + ", ".join(readbacks)
        )

    def timer_callback(self):
        if not self._ensure_can_open():
            return

        with self._lock:
            target_rpm = self._target_rpm
            target_steer = self._target_steer_deg
            pending_config_names = set(self._pending_steer_config_names)
            self._target_rpm = None
            self._target_steer_deg = None
            self._pending_steer_config_names.clear()

        if pending_config_names:
            try:
                self._apply_steer_config(pending_config_names)
            except Exception as error:
                self.get_logger().error(f"Steer parameter update failed: {error}")

        drive_status = None
        steer_status = None
        try:
            drive_status = self._drive_motor.read_status_all()
        except Exception as error:
            self.get_logger().debug(f"Drive read failed: {error}")
        try:
            steer_status = self._steer_motor.read_status_all()
        except Exception as error:
            self.get_logger().debug(f"Steer read failed: {error}")

        if not self._can_timeout_configured and (
            drive_status is not None or steer_status is not None
        ):
            try:
                if drive_status is not None:
                    self._drive_motor.configure_can_timeout(True, timeout_ms=1000)
                if steer_status is not None:
                    self._steer_motor.configure_can_timeout(True, timeout_ms=1000)
                self._can_timeout_configured = True
            except Exception as error:
                self.get_logger().warning(f"CAN timeout configuration failed: {error}")

        if (
            not self._watchdog_triggered
            and time.monotonic() - self._last_cmd_time > 1.0
            and drive_status is not None
            and abs(drive_status.speed) > 0.5
        ):
            try:
                self._drive_motor.set_speed(0.0)
                self._watchdog_triggered = True
            except Exception as error:
                self.get_logger().warning(f"Drive watchdog command failed: {error}")

        if target_rpm is not None and drive_status is not None:
            try:
                self._drive_motor.set_speed(target_rpm)
            except Exception as error:
                self.get_logger().warning(f"Drive command failed: {error}")
        if target_steer is not None and steer_status is not None:
            try:
                self._steer_motor.set_absolute_position_degrees(target_steer)
            except Exception as error:
                self.get_logger().warning(f"Steer command failed: {error}")

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["drive_motor", "steer_motor"]
        message.position = [
            math.radians(drive_status.angle) if drive_status else 0.0,
            math.radians(steer_status.angle) if steer_status else 0.0,
        ]
        message.velocity = [
            drive_status.speed * 2.0 * math.pi / 60.0 if drive_status else 0.0,
            steer_status.speed * 2.0 * math.pi / 60.0 if steer_status else 0.0,
        ]
        message.effort = [
            drive_status.q_current if drive_status else 0.0,
            steer_status.q_current if steer_status else 0.0,
        ]
        self.state_pub.publish(message)

    def destroy_node(self):
        self.get_logger().info("Shutting down motor controller")
        self._can_ok = False
        if self.timer:
            self.timer.cancel()
        for motor in (self._drive_motor, self._steer_motor):
            if motor is not None:
                try:
                    motor.disable()
                except Exception:
                    pass
        if self._can is not None and self._can.is_open:
            try:
                self._can.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
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
