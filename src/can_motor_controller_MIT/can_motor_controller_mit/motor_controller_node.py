import math
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .can_interface import CANInterface
from .motor import Motor
from .protocol import (
    circular_error_deg,
    mit_position_from_software_zero,
    steering_angle_from_software_zero,
    unwrap_target_deg,
)


class MotorControllerMITNode(Node):
    """Single CAN owner: ID1 uses C1 speed control; ID2 uses MIT control."""

    def __init__(self):
        super().__init__("motor_controller_node")

        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("drive_addr", 1)
        self.declare_parameter("steer_addr", 2)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("cmd_timeout", 0.2)
        self.declare_parameter("drive_watchdog_sec", 1.0)

        # MIT limits must match F0 values already stored in the motor.
        self.declare_parameter("steer_mit_pos_max_rad", 95.5)
        self.declare_parameter("steer_mit_vel_max_rad_s", 45.0)
        self.declare_parameter("steer_mit_torque_max_nm", 18.0)
        self.declare_parameter("steer_mit_kp", 5.0)
        self.declare_parameter("steer_mit_kd", 0.2)
        self.declare_parameter("steer_mit_velocity_ff_rad_s", 0.0)
        self.declare_parameter("steer_mit_torque_ff_nm", 0.0)
        self.declare_parameter("steer_center_deg", 0.0)
        self.declare_parameter("steer_max_delta_deg", 24.0)
        # Asymmetric ROS-frame limits (+left / -right). Motor applies an extra -1.
        self.declare_parameter("steer_max_left_deg", 24.5)
        self.declare_parameter("steer_max_right_deg", 30.0)
        self.declare_parameter("steer_zero_epoch", 0)
        self.declare_parameter("write_mit_limits_to_motor", False)

        # Startup mechanical home via 0xC2 (not firmware 0xB1 origin rewrite).
        self.declare_parameter("home_on_start", True)
        self.declare_parameter("home_on_reconnect", False)
        self.declare_parameter("startup_home_position_deg", 0.0)
        self.declare_parameter("startup_home_tolerance_deg", 2.0)
        self.declare_parameter("startup_home_speed_limit_rpm", 100.0)
        self.declare_parameter("startup_home_current_limit_a", 6.0)
        self.declare_parameter("startup_home_timeout_sec", 8.0)
        self.declare_parameter("steer_pos_max_speed_rpm", 500.0)
        self.declare_parameter("steer_max_q_current_a", 12.0)

        self._can_channel = self.get_parameter("can_channel").value
        self._drive_addr = int(self.get_parameter("drive_addr").value)
        self._steer_addr = int(self.get_parameter("steer_addr").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._drive_watchdog_sec = float(
            self.get_parameter("drive_watchdog_sec").value
        )
        publish_rate = float(self.get_parameter("publish_rate").value)

        self._lock = threading.Lock()
        # Drive is sticky (C1 needs refresh). Steer stays oneshot from /motor/command.
        self._held_drive_rpm = 0.0
        self._target_steer_rad = None
        self._last_drive_cmd_time = time.monotonic()
        self._drive_watchdog_triggered = False

        self._can = None
        self._drive_motor = None
        self._steer_motor = None
        self._can_ok = False
        self._last_drive_status = None
        self._last_steer_status = None
        self._mit_zero_rad = None
        self._last_mit_warning_time = 0.0
        self._startup_home_done = False

        self.state_pub = self.create_publisher(JointState, "/motor/state", 10)
        self.cmd_sub = self.create_subscription(
            JointState, "/motor/command", self.command_callback, 10
        )
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(
            f"MIT Motor Node started: drive ID={self._drive_addr} (C1), "
            f"steer ID={self._steer_addr} (MIT)"
        )

    def _parameter_float(self, name):
        return float(self.get_parameter(name).value)

    def _validate_parameters(self, parameters):
        for parameter in parameters:
            try:
                if parameter.name == "steer_mit_kp":
                    value = float(parameter.value)
                    if not 0.0 <= value <= 500.0:
                        raise ValueError("steer_mit_kp must be in [0, 500]")
                elif parameter.name == "steer_mit_kd":
                    value = float(parameter.value)
                    if not 0.0 <= value <= 5.0:
                        raise ValueError("steer_mit_kd must be in [0, 5]")
                elif parameter.name in {
                    "steer_mit_pos_max_rad",
                    "steer_mit_vel_max_rad_s",
                    "steer_mit_torque_max_nm",
                    "steer_max_delta_deg",
                }:
                    if float(parameter.value) <= 0.0:
                        raise ValueError(f"{parameter.name} must be positive")
            except (TypeError, ValueError) as error:
                return SetParametersResult(successful=False, reason=str(error))
        if any(parameter.name == "steer_zero_epoch" for parameter in parameters):
            if self._last_steer_status is None:
                return SetParametersResult(
                    successful=False, reason="MIT status is unavailable for zero capture"
                )
            if abs(self._last_steer_status.velocity) > 0.1:
                return SetParametersResult(
                    successful=False,
                    reason="Steering must be stationary before zero capture",
                )
            self._mit_zero_rad = self._last_steer_status.position
            self.get_logger().info(
                "MIT software zero recaptured for test: "
                f"zero={self._mit_zero_rad:.4f} rad"
            )
        return SetParametersResult(successful=True)

    def _set_host_mit_limits(self):
        self._steer_motor.set_mit_limits_local(
            self._parameter_float("steer_mit_pos_max_rad"),
            self._parameter_float("steer_mit_vel_max_rad_s"),
            self._parameter_float("steer_mit_torque_max_nm"),
        )

    def _home_steering_on_start(self):
        """Gently move steering to firmware 0° via 0xC2 before MIT zero capture."""
        steer = self._steer_motor
        drive = self._drive_motor
        target = self._parameter_float("startup_home_position_deg")
        tolerance = self._parameter_float("startup_home_tolerance_deg")
        home_rpm = self._parameter_float("startup_home_speed_limit_rpm")
        home_amps = self._parameter_float("startup_home_current_limit_a")
        timeout_sec = self._parameter_float("startup_home_timeout_sec")
        daily_rpm = self._parameter_float("steer_pos_max_speed_rpm")
        daily_amps = self._parameter_float("steer_max_q_current_a")

        try:
            drive.set_speed(0.0)
        except Exception as error:
            self.get_logger().warning(f"Drive stop before home failed: {error}")

        steer.set_position_max_speed_rpm(home_rpm)
        steer.set_max_q_current_amps(home_amps)
        self.get_logger().info(
            f"startup home limits 0x{self._steer_addr:02X}: "
            f"max_speed={home_rpm:g} RPM, max_q_current={home_amps:.2f} A"
        )

        settle_rpm = 1.0  # ~0.1 rad/s
        measured = steer.read_angle()
        status = steer.read_status_all()
        error = circular_error_deg(measured.multi_turn_angle, target)
        if error <= tolerance and abs(status.speed) <= settle_rpm:
            self.get_logger().info(
                f"startup: steering already near {target:g} deg "
                f"(err={error:.2f} deg); skip home move"
            )
            steer.set_position_max_speed_rpm(daily_rpm)
            steer.set_max_q_current_amps(daily_amps)
            time.sleep(0.15)
            return

        absolute = unwrap_target_deg(target, measured.multi_turn_angle)
        self.get_logger().info(
            f"startup: steering motor 0x{self._steer_addr:02X} → {target:g} deg "
            f"(gentle home, cmd_multi={absolute:.2f})"
        )
        steer.set_absolute_position_degrees(absolute)

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            time.sleep(0.05)
            measured = steer.read_angle()
            status = steer.read_status_all()
            if (
                circular_error_deg(measured.multi_turn_angle, target) <= tolerance
                and abs(status.speed) <= settle_rpm
            ):
                break
        else:
            self.get_logger().warning(
                f"startup home timed out after {timeout_sec:g}s "
                f"(last_multi={measured.multi_turn_angle:.2f} deg, "
                f"speed={status.speed:.2f} RPM); "
                "capturing MIT zero at current pose"
            )

        time.sleep(0.15)
        steer.set_position_max_speed_rpm(daily_rpm)
        steer.set_max_q_current_amps(daily_amps)

    def _capture_mit_software_zero(self):
        deadline = time.monotonic() + 2.0
        mit_status = None
        while time.monotonic() < deadline:
            mit_status = self._steer_motor.mit_read_status()
            if abs(mit_status.velocity) <= 0.1:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(
                "Cannot capture MIT software zero while steering is moving: "
                f"{mit_status.velocity:.3f} rad/s"
            )
        self._mit_zero_rad = mit_status.position
        left_deg = self._parameter_float("steer_max_left_deg")
        right_deg = self._parameter_float("steer_max_right_deg")
        if left_deg <= 0.0:
            left_deg = self._parameter_float("steer_max_delta_deg")
        if right_deg <= 0.0:
            right_deg = self._parameter_float("steer_max_delta_deg")
        delta_limit = math.radians(max(left_deg, right_deg))
        pos_max = self._parameter_float("steer_mit_pos_max_rad")
        if abs(self._mit_zero_rad) + delta_limit > pos_max:
            raise RuntimeError("MIT software-zero limits exceed configured Pos_Max")
        self._last_steer_status = mit_status
        self.get_logger().info(
            "MIT software zero captured after home: "
            f"zero={self._mit_zero_rad:.4f} rad, "
            f"allowed left=+{left_deg:g} deg / right=-{right_deg:g} deg"
        )

    def _ensure_can_open(self):
        if self._can_ok:
            return True
        try:
            self._can = CANInterface(
                channel=self._can_channel,
                baudrate=1_000_000,
                timeout=self._cmd_timeout,
            )
            self._can.open()
            self._drive_motor = Motor(
                self._can, self._drive_addr, timeout=self._cmd_timeout
            )
            self._steer_motor = Motor(
                self._can, self._steer_addr, timeout=self._cmd_timeout
            )
            self._drive_motor.enable()
            self._steer_motor.enable()
            self._set_host_mit_limits()

            if bool(self.get_parameter("write_mit_limits_to_motor").value):
                config = self._steer_motor.mit_configure(
                    self._parameter_float("steer_mit_pos_max_rad"),
                    self._parameter_float("steer_mit_vel_max_rad_s"),
                    self._parameter_float("steer_mit_torque_max_nm"),
                )
                self.get_logger().warning(
                    "MIT F0 limits written to non-volatile motor memory: "
                    f"pos={config.pos_max:g}, vel={config.vel_max:g}, "
                    f"torque={config.t_max:g}"
                )
            else:
                self.get_logger().info(
                    "Using host MIT limits without writing F0: "
                    f"pos={self._parameter_float('steer_mit_pos_max_rad'):g} rad, "
                    f"vel={self._parameter_float('steer_mit_vel_max_rad_s'):g} rad/s, "
                    f"torque={self._parameter_float('steer_mit_torque_max_nm'):g} Nm"
                )

            should_home = bool(self.get_parameter("home_on_start").value) and (
                bool(self.get_parameter("home_on_reconnect").value)
                or not self._startup_home_done
            )
            if should_home:
                self._home_steering_on_start()
                self._startup_home_done = True

            self._capture_mit_software_zero()

            self._can_ok = True
            self.get_logger().info("CAN opened; ID1 C1 and ID2 MIT are ready")
            return True
        except Exception as error:
            self.get_logger().warning(f"CAN/MIT initialization failed: {error}")
            if self._can is not None:
                try:
                    self._can.close()
                except Exception:
                    pass
            self._can = None
            self._drive_motor = None
            self._steer_motor = None
            self._mit_zero_rad = None
            self._can_ok = False
            return False

    def command_callback(self, message):
        with self._lock:
            for index, name in enumerate(message.name):
                if name in ("drive_motor", "motor_1") and len(message.velocity) > index:
                    target_rad_s = -float(message.velocity[index])
                    self._held_drive_rpm = target_rad_s * 60.0 / (2.0 * math.pi)
                    self._last_drive_cmd_time = time.monotonic()
                    self._drive_watchdog_triggered = False
                elif name in ("steer_motor", "motor_2") and len(message.position) > index:
                    center = -math.radians(self._parameter_float("steer_center_deg"))
                    left_deg = self._parameter_float("steer_max_left_deg")
                    right_deg = self._parameter_float("steer_max_right_deg")
                    if left_deg <= 0.0:
                        left_deg = self._parameter_float("steer_max_delta_deg")
                    if right_deg <= 0.0:
                        right_deg = self._parameter_float("steer_max_delta_deg")
                    # ROS position = +left / -right; motor target = -ROS.
                    # So motor clamp is [-left, +right] about center.
                    lo = center - math.radians(left_deg)
                    hi = center + math.radians(right_deg)
                    requested = -float(message.position[index])
                    self._target_steer_rad = max(lo, min(hi, requested))

    def _warn_mit(self, message):
        now = time.monotonic()
        if now - self._last_mit_warning_time >= 1.0:
            self.get_logger().warning(message)
            self._last_mit_warning_time = now

    def timer_callback(self):
        if not self._ensure_can_open():
            return

        with self._lock:
            held_drive_rpm = self._held_drive_rpm
            target_steer_rad = self._target_steer_rad
            self._target_steer_rad = None
            last_drive_cmd_time = self._last_drive_cmd_time

        if time.monotonic() - last_drive_cmd_time > self._drive_watchdog_sec:
            if abs(held_drive_rpm) > 0.5:
                if not self._drive_watchdog_triggered:
                    self.get_logger().info("Drive watchdog → 0 Rpm (no command)")
                with self._lock:
                    self._held_drive_rpm = 0.0
                held_drive_rpm = 0.0
                self._drive_watchdog_triggered = True

        # C1 first — do not gate on A4 success (that was dropping speed under CAN load).
        try:
            self._drive_motor.set_speed(held_drive_rpm)
        except Exception as error:
            self.get_logger().warning(f"Drive C1 command failed: {error}")

        try:
            self._last_drive_status = self._drive_motor.read_status_all()
        except Exception as error:
            self.get_logger().debug(f"Drive A4 read failed: {error}")

        try:
            if target_steer_rad is None:
                self._last_steer_status = self._steer_motor.mit_read_status()
            else:
                mit_target_rad = mit_position_from_software_zero(
                    target_steer_rad,
                    math.radians(self._parameter_float("steer_center_deg")),
                    self._mit_zero_rad,
                )
                self._last_steer_status = self._steer_motor.mit_control(
                    position_rad=mit_target_rad,
                    velocity_rad_s=self._parameter_float(
                        "steer_mit_velocity_ff_rad_s"
                    ),
                    kp=self._parameter_float("steer_mit_kp"),
                    kd=self._parameter_float("steer_mit_kd"),
                    torque_nm=self._parameter_float("steer_mit_torque_ff_nm"),
                )
                if not self._last_steer_status.is_mit_mode:
                    self._warn_mit("ID2 replied but did not report MIT mode active")
                if self._last_steer_status.has_fault:
                    self._warn_mit("ID2 MIT status reports a motor fault")
        except Exception as error:
            self._warn_mit(f"Steer MIT communication failed: {error}")

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["drive_motor", "steer_motor"]
        message.position = [
            math.radians(self._last_drive_status.angle) * -1
            if self._last_drive_status
            else 0.0,
            steering_angle_from_software_zero(
                self._last_steer_status.position,
                math.radians(self._parameter_float("steer_center_deg")),
                self._mit_zero_rad,
            ) * -1
            if self._last_steer_status and self._mit_zero_rad is not None
            else 0.0,
        ]
        message.velocity = [
            self._last_drive_status.speed * 2.0 * math.pi / 60.0 * -1
            if self._last_drive_status
            else 0.0,
            self._last_steer_status.velocity * -1 if self._last_steer_status else 0.0,
        ]
        # Drive effort is Q-current (A); steering effort is MIT torque (Nm).
        message.effort = [
            self._last_drive_status.q_current * -1 if self._last_drive_status else 0.0,
            self._last_steer_status.torque * -1 if self._last_steer_status else 0.0,
        ]
        self.state_pub.publish(message)

    def destroy_node(self):
        self.get_logger().info("Shutting down MIT motor controller")
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
        self._can_ok = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerMITNode()
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
