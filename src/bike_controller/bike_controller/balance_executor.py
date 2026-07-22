#!/usr/bin/env python3
"""
50Hz Autonomous Bicycle Balance Node (MIT Steering Mode).

Kickstands are handled by kickstand_speed node (not here).
"""

import math
import time
import csv
import os
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist

from bike_controller.bicycle_model import ScaleBikeModel
from bike_controller.controller import NominalBikeController


class BikeBalanceExecutor(Node):
    def __init__(self):
        super().__init__('bike_balance_executor')

        # --- Control & Speed Parameters ---
        self.declare_parameter('control_dt', 0.02)
        self.declare_parameter('control_method', 'lqr')
        self.declare_parameter('wheel_radius', 0.0725)
        self.declare_parameter('target_speed', 1.5)
        self.declare_parameter('acceleration_time', 3.0)
        self.declare_parameter('min_scheduling_speed', 0.5)
        # One-shot steer target when leaving Kill → Manual/Auto (deg). 0 = unchanged.
        self.declare_parameter('initial_steer_deg', 0.0)

        # --- LQR Tuning & Limits ---
        self.declare_parameter('lqr_q_steer', 4.0)
        self.declare_parameter('lqr_q_roll', 0.0)
        self.declare_parameter('lqr_q_roll_rate', 0.0)
        self.declare_parameter('lqr_r_steer_rate', 50.0)
        self.declare_parameter('max_steer_velocity', 5.0)
        self.declare_parameter('pole_wc', -5.0)
        # Distinct closed-loop poles [steer, roll, roll_rate] (rad/s).
        self.declare_parameter('poles_ctr', [-4.0, -7.0, -11.0])

        # --- Steering Center & Swing Limits ---
        # +steer_rel = physical left; - = right.
        self.declare_parameter('steer_center_deg', 0.0)
        self.declare_parameter('steer_max_delta_deg', 23.30)
        self.declare_parameter('steer_max_left_deg', 24.50)
        self.declare_parameter('steer_max_right_deg', 30.00)
        self.declare_parameter('drive_sign', -1.0)

        # --- Saturation anti-lock (breaks coordinated-turn trap at full steer) ---
        # When |roll| exceeds this while steer is at the into-fall limit, force
        # unwind toward center instead of freezing the integrator at the wall.
        self.declare_parameter('sat_unwind_enable', True)
        self.declare_parameter('sat_unwind_roll_deg', 12.0)
        self.declare_parameter('sat_unwind_rate_rad_s', 2.5)
        # Soft cap: |steer_rel| <= max(floor, gain*|roll|). 0 gain disables.
        self.declare_parameter('sat_steer_roll_gain', 1.6)
        self.declare_parameter('sat_steer_floor_deg', 8.0)

        # --- IMU & Topics ---
        self.declare_parameter('sensor_to_bike_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('gyro_bias_sensor_dps', [0.0, 0.0, 0.0])
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('motor_state_topic', '/motor/state')
        self.declare_parameter('motor_cmd_topic', '/motor/command')
        self.declare_parameter('teleop_topic', '/teleop/reference')
        self.declare_parameter('roll_limit_deg', 20.0)
        # False = left lean → left steer (into the fall). True only if mount needs it.
        self.declare_parameter('flip_roll_sign', False)
        self.declare_parameter('log_dir', '')
        self.declare_parameter('roll_rate_filter_alpha', 0.15)

        # --- Initialize System ---
        self.dt = self._p('control_dt')
        self.method = self._p('control_method')
        self.model = ScaleBikeModel(
            dt=self.dt, min_forward_speed=self._p('min_scheduling_speed')
        )
        self.controller = self._make_controller()

        self.steer_center_rad = math.radians(self._p('steer_center_deg'))
        left_deg = float(self._p('steer_max_left_deg'))
        right_deg = float(self._p('steer_max_right_deg'))
        if left_deg <= 0.0:
            left_deg = float(self._p('steer_max_delta_deg'))
        if right_deg <= 0.0:
            right_deg = float(self._p('steer_max_delta_deg'))
        self.steer_max_left_rad = math.radians(left_deg)
        self.steer_max_right_rad = math.radians(right_deg)

        # --- State Variables ---
        self.imu_quat_wxyz = [1.0, 0.0, 0.0, 0.0]
        self.imu_gyro_xyz = [0.0, 0.0, 0.0]
        self.steer_angle_rad = 0.0
        self.steer_rate_rad_s = 0.0
        self.rear_speed_rad_s = 0.0
        self.rear_speed_ms = 0.0

        self.steer_rel_target = 0.0
        self._initial_steer_applied = False
        self.start_time = time.time()
        self.roll_deg = 0.0
        self.roll_rate_rad_s = 0.0

        # Teleop References
        self.control_mode = -1  # -1: Safety Kill, 0: Manual, 1: Auto
        self.teleop_speed = 0.0
        self.cmd_speed = 0.0
        self.cmd_roll = 0.0
        self._ramp_start_time: Optional[float] = None

        rpy = self._p('sensor_to_bike_rpy_deg')
        self.R_sb = Rotation.from_euler("ZYX", np.deg2rad([rpy[2], rpy[1], rpy[0]]))
        self.gyro_bias = np.deg2rad(self._p('gyro_bias_sensor_dps'))

        # --- CSV Logging Setup ---
        self.log_dir = self._p('log_dir') or os.path.expanduser('~/bike_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(self.log_dir, f"bike_mit_log_{timestamp}.csv")
        self.log_file = open(self.log_path, 'w', newline='')
        self._log_fields = [
            't', 'roll_deg', 'steer_deg', 'roll_rate_dps',
            'speed_ms', 'speed_ref_ms', 'steer_rate_ref_dps',
            'steer_target_deg', 'saturated',
        ]
        self.csv_writer = csv.DictWriter(self.log_file, fieldnames=self._log_fields)
        self.csv_writer.writeheader()
        self.print_counter = 0

        # --- ROS 2 Interfaces ---
        self.imu_sub = self.create_subscription(
            Imu, self._p('imu_topic'), self._on_imu, 10
        )
        self.state_sub = self.create_subscription(
            JointState, self._p('motor_state_topic'), self._on_motor_state, 10
        )
        self.teleop_sub = self.create_subscription(
            Twist, self._p('teleop_topic'), self._on_teleop, 10
        )
        self.cmd_pub = self.create_publisher(JointState, self._p('motor_cmd_topic'), 10)

        self.timer = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info("MIT-Compatible Balance Executor Started Successfully!")
        if self.method == "place_distinct_poles":
            poles_txt = [float(np.real(p)) for p in self._p('poles_ctr')]
            self.get_logger().info(
                f"Control method=place_distinct_poles poles={poles_txt}"
            )
        elif self.method != "lqr":
            self.get_logger().info(
                f"Control method=place_multiple_poles wc={self._p('pole_wc')}"
            )
        if bool(self._p('sat_unwind_enable')):
            self.get_logger().info(
                f"sat_unwind ON: roll>{self._p('sat_unwind_roll_deg')}° at limit "
                f"→ unwind {self._p('sat_unwind_rate_rad_s')} rad/s; "
                f"soft |δ|≤max({self._p('sat_steer_floor_deg')}°, "
                f"{self._p('sat_steer_roll_gain')}|φ|)"
            )

    def _p(self, name):
        return self.get_parameter(name).value

    def _make_controller(self) -> NominalBikeController:
        common = dict(
            u_min=-self._p('max_steer_velocity'),
            u_max=self._p('max_steer_velocity'),
        )
        if self.method == "lqr":
            Q = np.diag([
                self._p('lqr_q_steer'),
                self._p('lqr_q_roll'),
                self._p('lqr_q_roll_rate'),
            ])
            R = np.array([[self._p('lqr_r_steer_rate')]])
            return NominalBikeController(
                self.model.sys, method="lqr", Qc=Q, Rc=R, **common
            )
        if self.method == "place_distinct_poles":
            poles = [complex(float(p)) for p in self._p('poles_ctr')]
            if len(poles) != 3:
                raise ValueError(f"poles_ctr must have 3 entries, got {len(poles)}")
            return NominalBikeController(
                self.model.sys,
                method="place_distinct_poles",
                poles_ctr=poles,
                **common,
            )
        return NominalBikeController(
            self.model.sys,
            method="place_multiple_poles",
            wc=self._p('pole_wc'),
            **common,
        )

    def _on_imu(self, msg: Imu):
        self.imu_quat_wxyz = [
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        ]
        self.imu_gyro_xyz = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]

    def _on_motor_state(self, msg: JointState):
        """Extracts telemetry from the combined MIT node JointState message."""
        try:
            d_idx = msg.name.index('drive_motor')
            s_idx = msg.name.index('steer_motor')
            self.steer_angle_rad = float(msg.position[s_idx])
            self.steer_rate_rad_s = float(msg.velocity[s_idx])
            self.rear_speed_rad_s = float(msg.velocity[d_idx])
        except (ValueError, IndexError):
            pass

    def _on_teleop(self, msg: Twist):
        self.control_mode = int(msg.linear.z)
        self.teleop_speed = msg.linear.x
        self.cmd_roll = msg.angular.z

    def _estimate_state(self) -> np.ndarray:
        w, x, y, z = self.imu_quat_wxyz
        xyzw = np.array([x, y, z, w])
        norm = np.linalg.norm(xyzw)
        xyzw = xyzw / norm if norm >= 1e-6 else np.array([0.0, 0.0, 0.0, 1.0])

        R_wb = Rotation.from_quat(xyzw) * self.R_sb.inv()
        _, pitch, roll = R_wb.as_euler("ZYX")

        p, q, r = self.R_sb.apply(np.asarray(self.imu_gyro_xyz) - self.gyro_bias)
        cos_pitch = math.cos(pitch)
        roll_rate = (
            0.0
            if abs(cos_pitch) < 1e-3
            else p
            + math.sin(roll) * math.tan(pitch) * q
            + math.cos(roll) * math.tan(pitch) * r
        )

        if bool(self._p('flip_roll_sign')):
            roll = -roll
            roll_rate = -roll_rate

        alpha = float(self._p('roll_rate_filter_alpha'))
        alpha = min(1.0, max(0.01, alpha))
        if not hasattr(self, '_filtered_roll_rate'):
            self._filtered_roll_rate = float(roll_rate)
        self._filtered_roll_rate = (
            alpha * float(roll_rate) + (1.0 - alpha) * self._filtered_roll_rate
        )
        roll_rate = self._filtered_roll_rate

        steer_rel = self.steer_angle_rad - self.steer_center_rad
        self.roll_deg = float(math.degrees(roll))
        self.roll_rate_rad_s = float(roll_rate)
        self.rear_speed_ms = self.rear_speed_rad_s * self._p('wheel_radius')

        return np.array([[steer_rel], [roll - self.cmd_roll], [roll_rate]])

    def _scheduling_speed(self) -> float:
        min_sched = float(self._p('min_scheduling_speed'))
        measured = float(self.rear_speed_ms)
        if abs(measured) >= min_sched:
            return measured
        if abs(self.cmd_speed) > 1e-6:
            return math.copysign(min_sched, self.cmd_speed)
        return min_sched

    def _soft_steer_limit_rad(self) -> tuple[float, float]:
        """Lean-dependent steer caps to avoid locking at coordinated-turn δ_max."""
        left = self.steer_max_left_rad
        right = self.steer_max_right_rad
        gain = float(self._p('sat_steer_roll_gain'))
        if gain <= 1e-9 or not bool(self._p('sat_unwind_enable')):
            return left, right
        floor = math.radians(float(self._p('sat_steer_floor_deg')))
        soft = max(floor, gain * abs(math.radians(self.roll_deg)))
        return min(left, soft), min(right, soft)

    def _apply_sat_unwind(self, steer_rate_ref: float, left_lim: float, right_lim: float) -> float:
        """If stuck at into-fall steer limit with large lean, force unwind to center."""
        if not bool(self._p('sat_unwind_enable')):
            return steer_rate_ref
        roll_th = float(self._p('sat_unwind_roll_deg'))
        unwind = float(self._p('sat_unwind_rate_rad_s'))
        eps = 1e-4
        at_left = self.steer_rel_target >= left_lim - eps
        at_right = self.steer_rel_target <= -right_lim + eps
        # Into-fall lock: left lean (roll<0) at left limit, or right lean at right limit.
        if at_left and self.roll_deg <= -roll_th:
            return -abs(unwind)  # leave left limit toward center
        if at_right and self.roll_deg >= roll_th:
            return +abs(unwind)  # leave right limit toward center
        return steer_rate_ref

    def _control_loop(self):
        now = time.time()
        elapsed = now - self.start_time

        if self.control_mode == -1:
            self._ramp_start_time = None
            self._initial_steer_applied = False
            self._send_stop(hold_steer=True)
            if self.print_counter % 50 == 0:
                self.get_logger().info(
                    "SWA=-1: Safety Disabled. (Kickstands: kickstand_speed node)"
                )
            self.print_counter += 1
            return
        elif self.control_mode == 0:
            self._ramp_start_time = None
            self.cmd_speed = self.teleop_speed
            self._apply_initial_steer_once()
        elif self.control_mode == 1:
            if self._ramp_start_time is None:
                self._ramp_start_time = now
            ramp_elapsed = now - self._ramp_start_time
            accel_time = self._p('acceleration_time')
            self.cmd_speed = self._p('target_speed') * min(1.0, ramp_elapsed / accel_time)
            self._apply_initial_steer_once()

        state = self._estimate_state()
        scheduling_speed = self._scheduling_speed()
        self.model.updateSysParam(
            scheduling_speed, min_forw_vel=self._p('min_scheduling_speed')
        )
        self.controller.update_system_and_gain(self.model.sys)

        steer_rate_ref = float(self.controller.step([[0.0]], state)[0, 0])

        left_lim, right_lim = self._soft_steer_limit_rad()
        steer_rate_ref = self._apply_sat_unwind(steer_rate_ref, left_lim, right_lim)

        proposed_target = self.steer_rel_target + (steer_rate_ref * self.dt)
        if proposed_target > left_lim:
            self.steer_rel_target = left_lim
            if steer_rate_ref > 0.0:
                steer_rate_ref = 0.0
        elif proposed_target < -right_lim:
            self.steer_rel_target = -right_lim
            if steer_rate_ref < 0.0:
                steer_rate_ref = 0.0
        else:
            self.steer_rel_target = proposed_target

        # If soft cap shrank below current target, pull target in immediately.
        if self.steer_rel_target > left_lim:
            self.steer_rel_target = left_lim
        elif self.steer_rel_target < -right_lim:
            self.steer_rel_target = -right_lim

        steer_abs_target = self.steer_center_rad + self.steer_rel_target
        rear_rate_ref = self.cmd_speed / self._p('wheel_radius')

        if abs(self.roll_deg) > self._p('roll_limit_deg'):
            self.get_logger().warn(
                f"⚠️ Roll > {self._p('roll_limit_deg')}°. EMERGENCY STOP!",
                throttle_duration_sec=0.5,
            )
            self._send_stop(hold_steer=True)
            return

        self._send_command(rear_rate_ref, steer_abs_target)

        self.csv_writer.writerow({
            't': elapsed,
            'roll_deg': self.roll_deg,
            'steer_deg': math.degrees(self.steer_angle_rad),
            'roll_rate_dps': math.degrees(self.roll_rate_rad_s),
            'speed_ms': self.rear_speed_ms,
            'speed_ref_ms': self.cmd_speed,
            'steer_rate_ref_dps': math.degrees(steer_rate_ref),
            'steer_target_deg': math.degrees(steer_abs_target),
            'saturated': int(
                abs(steer_rate_ref) >= self._p('max_steer_velocity') - 0.01
            ),
        })

        if self.print_counter % 25 == 0:
            mode_str = "MANUAL" if self.control_mode == 0 else "AUTO"
            steer_meas_deg = math.degrees(self.steer_angle_rad)
            steer_tgt_deg = math.degrees(steer_abs_target)
            self.get_logger().info(
                f"[{mode_str}] cmd: {self.cmd_speed:+.2f}m/s | meas: {self.rear_speed_ms:+.2f} | "
                f"Roll: {self.roll_deg:+.1f}° | "
                f"Steer: {steer_meas_deg:+.1f}° → tgt {steer_tgt_deg:+.1f}°"
            )
        self.print_counter += 1

    def _send_command(self, rear_rad_s: float, steer_abs_rad: float):
        """Dispatches commands to index 0 (drive velocity) and index 1 (MIT steer position)."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['drive_motor', 'steer_motor']
        msg.velocity = [float(rear_rad_s), 0.0]
        msg.position = [0.0, float(steer_abs_rad)]
        self.cmd_pub.publish(msg)

    def _apply_initial_steer_once(self) -> None:
        """Set steer target once when leaving Kill (mass-imbalance startup aid)."""
        if self._initial_steer_applied:
            return
        deg = float(self._p('initial_steer_deg'))
        if abs(deg) > 1e-6:
            rad = math.radians(deg)
            rad = max(-self.steer_max_right_rad, min(self.steer_max_left_rad, rad))
            self.steer_rel_target = rad
            self.get_logger().info(
                f"initial_steer applied: {math.degrees(rad):+.1f} deg"
            )
        self._initial_steer_applied = True

    def _send_stop(self, hold_steer: bool = True):
        """Stop drive. Default: hold last steer target (avoid sudden return-to-zero)."""
        if hold_steer:
            steer_cmd = self.steer_center_rad + self.steer_rel_target
        else:
            steer_cmd = self.steer_center_rad
            self.steer_rel_target = 0.0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['drive_motor', 'steer_motor']
        msg.velocity = [0.0, 0.0]
        msg.position = [0.0, float(steer_cmd)]
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        try:
            if getattr(self, 'log_file', None) and not self.log_file.closed:
                self.log_file.close()
        except Exception:
            pass
        try:
            if rclpy.ok():
                self._send_stop(hold_steer=True)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BikeBalanceExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
