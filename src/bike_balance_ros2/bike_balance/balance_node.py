#!/usr/bin/env python3
"""
50Hz 双环自行车平衡控制 — 真车 ROS2 节点
===========================================

算法来源: 07_MuJoCo实战_50Hz双环自行车平衡.ipynb
真车适配: GIM3510 自带三环 PID → 发绝对角度 (steer) + 速度 (rear)

架构:
  /imu/data ────────────┐
  /motor/state ─────────┤→ 状态估计 → LQR/极点配置 → 积分 → /motor/command
                        │     [δ, φ, φ_dot]      u = -Kx     steer: 绝对角度(rad)
                        │                                      drive: 速度(rad/s)

真车 vs 仿真差异:
  - 仿真: steer_rate_ref → PI内环 → torque → MuJoCo
  - 真车: steer_rate_ref → 积分 → 绝对角度 → GIM3510 (电机自带位置环)
  - 仿真: rear_rate_ref → P内环 → torque → MuJoCo
  - 真车: rear_rate_ref → 直接发速度 → GIM3510 (电机自带速度环)
"""

import math
import time
import csv
import os
from dataclasses import dataclass, field
from typing import Optional, List, Dict

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, Joy


# =============================================================================
# 嵌入式 bicycle model + controller (从 nominal_bike_control 精简，避免路径问题)
# =============================================================================

@dataclass
class BicycleParams:
    """自行车物理参数 (已实测)"""
    rear_contact_to_com: float = 0.117
    wheelbase: float = 0.28223
    trail: float = 0.0140
    com_height: float = 0.105
    mass: float = 2.218
    roll_inertia: float = 0.02445
    steering_axis_angle: float = 0.179770  # rad ≈ 10.3°
    gravity: float = 9.81

    @property
    def effective_roll_inertia(self) -> float:
        if self.roll_inertia is not None and self.roll_inertia > 0:
            return float(self.roll_inertia)
        return 4.0 / 3.0 * self.mass * self.com_height ** 2


class BicycleModel:
    """三状态线性自行车模型: x = [δ, φ, φ_dot], u = steer_rate"""

    def __init__(self, dt: float, params: BicycleParams | None = None,
                 min_speed: float = 0.5):
        self.dt = float(dt)
        self.params = params or BicycleParams()
        self.min_speed = float(min_speed)
        self.speed = 0.0
        self.scheduled_speed = self.min_speed
        self.A = np.zeros((3, 3))
        self.Bu = np.zeros((3, 1))
        self._update(self.min_speed)

    def _update(self, speed: float) -> None:
        """根据速度更新 A, B 矩阵 (教程第1-2讲的小角度模型)"""
        p = self.params
        g = p.gravity
        v = max(self.min_speed, abs(float(speed)))

        a = p.rear_contact_to_com
        b = p.wheelbase
        c = p.trail
        h = p.com_height
        m = p.mass
        I = p.effective_roll_inertia
        cl = math.cos(p.steering_axis_angle)

        a1 = m * a * h * v * cl / (b * I)
        a2 = (m * v ** 2 * h - m * a * c * g) * cl / (b * I)
        a4 = m * g * h / I

        self.A = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [a2, a4, 0.0],
        ])
        self.Bu = np.array([[1.0], [0.0], [a1]])
        self.speed = speed
        self.scheduled_speed = v

    def update(self, speed: float) -> None:
        self._update(speed)


class LQRController:
    """连续时间 LQR + 极点配置 状态反馈控制器

    u = -K @ (x - x_eq),  regulator 模式 x_eq = 0 → u = -K @ x
    """

    def __init__(self, A: np.ndarray, B: np.ndarray,
                 method: str = "lqr", **kwargs):
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float)
        self.method = method
        self.kwargs = kwargs
        self.K = np.zeros((1, self.A.shape[0]))
        self._compute_gain()

    def _compute_gain(self):
        from scipy.linalg import solve_continuous_are

        if self.method == "lqr":
            Q = np.asarray(self.kwargs.get("Q", np.eye(3)), dtype=float)
            R = np.asarray(self.kwargs.get("R", np.eye(1)), dtype=float)
            P = solve_continuous_are(self.A, self.B, Q, R)
            self.K = np.linalg.solve(R, self.B.T @ P)
        elif self.method == "pole":
            from scipy.signal import place_poles
            wc = self.kwargs.get("wc", -5.0)
            poles = np.full(self.A.shape[0], wc)
            self.K = place_poles(self.A, self.B, poles).gain_matrix
        else:
            raise ValueError(f"Unknown method: {self.method}")

    def update_model(self, A: np.ndarray, B: np.ndarray):
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float)
        self._compute_gain()

    def step(self, state: np.ndarray, u_min: float = -np.inf,
             u_max: float = np.inf) -> float:
        u = float(-self.K @ state)
        return float(np.clip(u, u_min, u_max))


# =============================================================================
# 主 ROS2 节点
# =============================================================================

class BikeBalanceNode(Node):
    """50Hz 自行车平衡控制节点"""

    def __init__(self):
        super().__init__('bike_balance_node')

        # ── 声明参数 (可通过 launch 文件 / 命令行覆盖) ──
        self._declare_all_params()

        # ── 读取参数 ──
        dt = self._p('control_dt')
        self.dt = dt
        self.method = self._p('control_method')

        # ── 物理参数 → 模型 → 控制器 ──
        self.bike_params = BicycleParams(
            rear_contact_to_com=self._p('rear_contact_to_com'),
            wheelbase=self._p('wheelbase'),
            trail=self._p('trail'),
            com_height=self._p('com_height'),
            mass=self._p('mass'),
            roll_inertia=self._p('roll_inertia'),
            steering_axis_angle=self._p('steering_axis_angle'),
        )
        self.model = BicycleModel(dt=dt, params=self.bike_params,
                                  min_speed=self._p('min_scheduling_speed'))
        self.controller = self._make_controller()

        # ── 转向零位 (绝对角度 → 相对角度的转换) ──
        self.steer_center_rad = math.radians(self._p('steer_center_deg'))
        self.steer_max_delta_rad = math.radians(self._p('steer_max_delta_deg'))

        # ── 状态变量 ──
        self.imu_quat_wxyz = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
        self.imu_gyro_xyz = [0.0, 0.0, 0.0]          # rad/s
        self.steer_angle_rad = 0.0       # 当前绝对角度 (来自电机反馈)
        self.steer_rate_rad_s = 0.0      # 当前转向角速度
        self.rear_speed_rad_s = 0.0      # 后轮角速度 (rad/s)
        self.rear_speed_ms = 0.0         # 后轮线速度 (m/s)

        self.steer_rel_target = 0.0      # 目标相对转向角 (rad), 积分得到
        self.steer_rel_startup = None    # 启动时车把初始位置 (首次采样填入)
        self.speed_target_ms = 0.0       # 目标线速度
        self.start_time = time.time()
        self.balance_active = False
        self.roll_deg = 0.0
        self.roll_rate_rad_s = 0.0

        # ── 遥控器 (RC) 状态 ──
        self.rc_speed_cmd = 0.0       # RC 油门 → 目标速度 (m/s)
        self.rc_roll_cmd = 0.0        # RC 转向 → roll 角偏移 (rad)
        self.rc_active = False        # SWA 是否激活
        self.rc_speed_active = False # 油门是否推出死区 (接管速度)
        self.rc_ever_received = False # 是否曾经收到过 RC 信号
        self._last_joy_time = 0.0     # 上次收到 Joy 的时间戳

        # ── IMU 安装 & 零偏 ──
        rpy = self._p('sensor_to_bike_rpy_deg')
        self.R_sb = Rotation.from_euler(
            "ZYX", np.deg2rad([rpy[2], rpy[1], rpy[0]])
        )
        self.gyro_bias = np.deg2rad(self._p('gyro_bias_sensor_dps'))

        # ── 数据记录 ──
        self.log: List[Dict] = []
        self._log_fields = [
            't', 'roll_deg', 'steer_deg', 'roll_rate_dps',
            'speed_ms', 'speed_ref_ms', 'steer_rate_ref_dps',
            'steer_target_deg', 'K_delta', 'K_roll', 'K_roll_rate',
            'saturated', 'rc_active', 'rc_speed_cmd', 'rc_roll_cmd',
        ]

        # ── ROS2 接口 ──
        self.imu_sub = self.create_subscription(
            Imu, self._p('imu_topic'), self._on_imu, 10)
        self.state_sub = self.create_subscription(
            JointState, self._p('motor_state_topic'), self._on_motor_state, 10)
        self.cmd_pub = self.create_publisher(
            JointState, self._p('motor_cmd_topic'), 10)

        # ── 遥控器订阅 (可选, 配合 sbus_receiver 使用) ──
        self.joy_sub = self.create_subscription(
            Joy, '/sbus/joy', self._on_joy, 10)

        # ── 50Hz 定时器 ──
        self.timer = self.create_timer(dt, self._control_loop)

        # ── 低速状态机 ──
        self._ramp_start_time: Optional[float] = None

        self.get_logger().info(
            f"BikeBalanceNode 初始化完毕\n"
            f"  控制方法: {self.method}\n"
            f"  频率: {1.0/dt:.0f} Hz\n"
            f"  K = {self.controller.K.flatten()}\n"
            f"  转向零位: {self._p('steer_center_deg')}°\n"
            f"  转向范围: ±{self._p('steer_max_delta_deg')}°\n"
            f"  RC 遥控: {'启用' if self._p('rc_enabled') else '禁用'} "
            f"(max_speed={self._p('rc_max_speed')} m/s, "
            f"max_roll={math.degrees(self._p('rc_max_roll_rad')):.0f}°)\n"
            f"  订阅: {self._p('imu_topic')}, {self._p('motor_state_topic')}, /sbus/joy\n"
            f"  发布: {self._p('motor_cmd_topic')}"
        )

    # ────────────────────────────────────────────────────────────────
    # 参数声明
    # ────────────────────────────────────────────────────────────────

    def _declare_all_params(self):
        # 控制
        self.declare_parameter('control_dt', 0.02)
        self.declare_parameter('control_method', 'lqr')

        # 物理参数
        self.declare_parameter('rear_contact_to_com', 0.117)
        self.declare_parameter('wheelbase', 0.28223)
        self.declare_parameter('trail', 0.0140)
        self.declare_parameter('com_height', 0.105)
        self.declare_parameter('mass', 2.218)
        self.declare_parameter('roll_inertia', 0.02445)
        self.declare_parameter('steering_axis_angle', 0.179770)

        # 轮子 & 速度
        self.declare_parameter('wheel_radius', 0.0725)
        self.declare_parameter('target_speed', 1.5)
        self.declare_parameter('acceleration_time', 3.0)
        self.declare_parameter('min_scheduling_speed', 0.5)

        # LQR
        self.declare_parameter('lqr_q_steer', 4.0)
        self.declare_parameter('lqr_q_roll', 100.0)
        self.declare_parameter('lqr_q_roll_rate', 2.0)
        self.declare_parameter('lqr_r', 10.0)
        self.declare_parameter('max_steer_velocity', 5.0)  # rad/s

        # 极点配置
        self.declare_parameter('pole_wc', -5.0)

        # 转向零位 & 范围
        self.declare_parameter('steer_center_deg', 90.0)
        self.declare_parameter('steer_max_delta_deg', 25.0)

        # IMU 标定
        self.declare_parameter('sensor_to_bike_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('gyro_bias_sensor_dps', [0.0, 0.0, 0.0])

        # ROS2 topics
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('motor_state_topic', '/motor/state')
        self.declare_parameter('motor_cmd_topic', '/motor/command')

        # 安全
        self.declare_parameter('roll_limit_deg', 20.0)
        self.declare_parameter('start_balance_after_s', 0.5)
        self.declare_parameter('startup_smooth_time', 2.0)  # 软启动时长 (秒)
        self.declare_parameter('auto_center_on_start', True) # 启动时自动将当前位置视为正中

        # RC 遥控
        self.declare_parameter('rc_enabled', True)
        self.declare_parameter('rc_max_speed', 2.0)          # m/s, 油门最大速度
        self.declare_parameter('rc_max_roll_rad', 0.175)     # rad ≈ 10°, 最大 roll 偏移
        self.declare_parameter('rc_timeout', 0.5)            # 秒, RC 信号超时回退
        self.declare_parameter('rc_throttle_deadzone', 0.1)  # 油门死区, ±10% 内视为未操作

        # 日志
        self.declare_parameter('log_dir', '')

    def _p(self, name):
        return self.get_parameter(name).value

    # ────────────────────────────────────────────────────────────────
    # 控制器工厂
    # ────────────────────────────────────────────────────────────────

    def _make_controller(self) -> LQRController:
        method = self.method
        if method == "lqr":
            Q = np.diag([
                self._p('lqr_q_steer'),
                self._p('lqr_q_roll'),
                self._p('lqr_q_roll_rate'),
            ])
            R = np.array([[self._p('lqr_r')]])
            return LQRController(self.model.A, self.model.Bu,
                                 method="lqr", Q=Q, R=R)
        else:
            return LQRController(self.model.A, self.model.Bu,
                                 method="pole", wc=self._p('pole_wc'))

    # ────────────────────────────────────────────────────────────────
    # 回调: 传感器数据
    # ────────────────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu):
        self.imu_quat_wxyz = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]
        self.imu_gyro_xyz = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]

    def _on_motor_state(self, msg: JointState):
        """解析 /motor/state → 获取反馈角度和速度"""
        try:
            d_idx = msg.name.index('drive_motor')
            s_idx = msg.name.index('steer_motor')
            self.steer_angle_rad = float(msg.position[s_idx])   # 绝对角度 (rad)
            self.steer_rate_rad_s = float(msg.velocity[s_idx])  # 角速度 (rad/s)
            self.rear_speed_rad_s = float(msg.velocity[d_idx])  # 后轮角速度 (rad/s)
        except ValueError:
            pass

    # ────────────────────────────────────────────────────────────────
    # 回调: 遥控器 (RC)
    # ────────────────────────────────────────────────────────────────

    def _on_joy(self, msg: Joy):
        """解析 /sbus/joy → 更新 RC 速度指令 (油门回中 = 交还算法)"""
        if not self._p('rc_enabled'):
            return

        self._last_joy_time = time.time()
        self.rc_ever_received = True

        # SWA 三档开关: 1=DOWN(激活), 0=MID(关), -1=UP(关)
        self.rc_active = (msg.buttons[0] == 1)

        if not self.rc_active:
            self.rc_speed_active = False
            return

        # 油门: axes[1] 取反 → -1~1
        throttle = float(np.clip(-msg.axes[1], -1.0, 1.0))
        deadzone = self._p('rc_throttle_deadzone')
        if abs(throttle) < deadzone:
            self.rc_speed_active = False   # 回中, 交还给自主速度斜坡
        else:
            self.rc_speed_active = True    # 推出死区, RC 接管速度
            self.rc_speed_cmd = throttle * self._p('rc_max_speed')

    # ────────────────────────────────────────────────────────────────
    # 状态估计 (模仿 notebook 的 bicycle_state_from_imu)
    # ────────────────────────────────────────────────────────────────

    def _estimate_state(self):
        """从 IMU 四元数+陀螺 → roll 角 + roll_rate. 数据未就绪返回 None."""
        w, x, y, z = self.imu_quat_wxyz
        if w*w + x*x + y*y + z*z < 0.001:  # IMU 数据还没到
            return None
        R_ws = Rotation.from_quat([x, y, z, w])  # world←sensor
        R_wb = R_ws * self.R_sb.inv()             # world←bike
        _, pitch, roll = R_wb.as_euler("ZYX")

        # ② 陀螺 → 自行车角速度
        gyro_corrected = self.R_sb.apply(
            np.asarray(self.imu_gyro_xyz) - self.gyro_bias
        )
        p, q, r = gyro_corrected

        # ③ roll_rate (欧拉角导数转换)
        cos_pitch = math.cos(pitch)
        if abs(cos_pitch) < 1e-3:
            roll_rate = 0.0
        else:
            roll_rate = (p + math.sin(roll) * math.tan(pitch) * q
                         + math.cos(roll) * math.tan(pitch) * r)

        # ④ 转向相对角 = 当前绝对角度 - 零位 (处理 0°/360° 回绕)
        steer_rel_deg = (math.degrees(self.steer_angle_rad)
                         - math.degrees(self.steer_center_rad)) % 360.0
        if steer_rel_deg > 180.0:
            steer_rel_deg -= 360.0
        steer_rel = math.radians(steer_rel_deg)

        self.roll_deg = float(math.degrees(roll))
        self.roll_rate_rad_s = float(roll_rate)
        self.rear_speed_ms = self.rear_speed_rad_s * self._p('wheel_radius')

        return np.array([[steer_rel], [roll], [roll_rate]])

    # ────────────────────────────────────────────────────────────────
    # 50Hz 控制主循环
    # ────────────────────────────────────────────────────────────────

    def _control_loop(self):
        now = time.time()
        elapsed = now - self.start_time

        # ── 0. RC 超时检测 ──
        rc_enabled = self._p('rc_enabled')
        rc_timeout = self._p('rc_timeout')
        if rc_enabled and self.rc_active:
            if (now - self._last_joy_time) > rc_timeout:
                self.rc_active = False
                self.get_logger().warn(
                    f"⚠️ RC 信号超时 ({rc_timeout}s), 回退自主模式",
                    throttle_duration_sec=1.0)

        # ── 0b. SWA 关闭 → 急停 (仅在收到过 RC 信号后生效) ──
        if rc_enabled and self.rc_ever_received and not self.rc_active:
            self._send_stop()
            self.steer_rel_target = 0.0
            self._ramp_start_time = None
            return

        # ── 1. 状态估计 ──
        state = self._estimate_state()
        if state is None:
            return  # IMU 数据还没到, 等下一帧

        # ── 2. 自动归零: 首次收到数据时, 将当前车把位置视为正中 ──
        smooth_time = self._p('startup_smooth_time')
        if self.steer_rel_startup is None:
            if self._p('auto_center_on_start'):
                self.steer_center_rad = self.steer_angle_rad
                self.get_logger().info(
                    f"自动归零: steer_center = {math.degrees(self.steer_center_rad):.1f}°")
                state = self._estimate_state()

            self.steer_rel_startup = float(state[0, 0])
            if abs(self.steer_rel_startup) > 0.01:
                self.get_logger().info(
                    f"软启动: 初始偏移={math.degrees(self.steer_rel_startup):.1f}°, "
                    f"{smooth_time:.1f}s 内平滑归零")

        # ── 3. 速度源: RC 油门 vs 自主斜坡 ──
        if rc_enabled and self.rc_active and self.rc_speed_active:
            # RC 油门推出死区: 接管速度
            self.speed_target_ms = self.rc_speed_cmd
            self._ramp_start_time = None   # 复位斜坡, 退出时从0重新加速
        else:
            # 自主模式: 速度斜坡到 target_speed
            target_speed = self._p('target_speed')
            accel_time = self._p('acceleration_time')
            if self._ramp_start_time is None:
                self._ramp_start_time = now
            ramp_elapsed = now - self._ramp_start_time
            if ramp_elapsed < accel_time:
                self.speed_target_ms = target_speed * (ramp_elapsed / accel_time)
            else:
                self.speed_target_ms = target_speed

        # ── 4. 增益调度 ──
        scheduling_speed = max(abs(self.rear_speed_ms),
                               self._p('min_scheduling_speed'))
        self.model.update(scheduling_speed)
        self.controller.update_model(self.model.A, self.model.Bu)

        # ── 5. 外环: u = -K @ x (软启动期间平滑过渡) ──
        # 注意: RC 激活时跳过软启动, 直接用 LQR 控制
        rc_bypass_smooth = rc_enabled and self.rc_active
        if elapsed < smooth_time and smooth_time > 0 and not rc_bypass_smooth:
            # 软启动: 车把目标从初始位置线性过渡到 0
            blend = elapsed / smooth_time
            self.steer_rel_target = self.steer_rel_startup * (1.0 - blend)
            steer_rate_ref = 0.0
        else:
            # 正常控制 (RC 或自主)
            steer_rate_ref = self.controller.step(
                state,
                u_min=-self._p('max_steer_velocity'),
                u_max=self._p('max_steer_velocity'),
            )

            # 启动延迟: 前 N 秒不转把 (RC 激活时跳过)
            if not rc_bypass_smooth and elapsed < self._p('start_balance_after_s') + smooth_time:
                steer_rate_ref = 0.0

            # ── 6. 积分: steer_rate → steer_angle ──
            self.steer_rel_target += steer_rate_ref * self.dt

        self.steer_rel_target = float(np.clip(
            self.steer_rel_target,
            -self.steer_max_delta_rad,
            self.steer_max_delta_rad,
        ))

        # ── 6. 转成绝对角度 ──
        steer_abs_target = self.steer_center_rad + self.steer_rel_target

        # ── 7. 后轮速度 (m/s → rad/s) ──
        rear_rate_ref = self.speed_target_ms / self._p('wheel_radius')

        # ── 8. 安全检查 ──
        roll_limit = self._p('roll_limit_deg')
        if abs(self.roll_deg) > roll_limit:
            self.get_logger().warn(
                f"⚠️ roll={self.roll_deg:.1f}° > {roll_limit}° 限幅, 急停!",
                throttle_duration_sec=0.5,
            )
            self._send_stop()
            self.steer_rel_target = 0.0  # 重置积分
            self._ramp_start_time = now   # 重置速度斜坡
            return

        # ── 9. 发布指令 ──
        self._send_command(rear_rate_ref, steer_abs_target)

        # ── 10. 数据记录 ──
        self._log_row(elapsed, steer_rate_ref, steer_abs_target)

        # ── 11. 周期日志 (2Hz) ──
        if int(elapsed * 2) != int((elapsed - self.dt) * 2):
            rc_tag = "[RC] " if self.rc_active else ""
            self.get_logger().info(
                f"{rc_tag}t={elapsed:.1f}s | roll={self.roll_deg:+.2f}° | "
                f"steer={math.degrees(self.steer_rel_target):+.1f}° | "
                f"speed={self.rear_speed_ms:.2f} m/s | "
                f"K=[{self.controller.K[0,0]:.1f}, {self.controller.K[0,1]:.1f}, {self.controller.K[0,2]:.1f}]"
            )

    # ────────────────────────────────────────────────────────────────
    # 通信
    # ────────────────────────────────────────────────────────────────

    def _send_command(self, rear_rad_s: float, steer_abs_rad: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['drive_motor', 'steer_motor']
        # drive: velocity 字段 → motor_node 转 RPM → set_speed()
        msg.velocity = [-float(rear_rad_s), 0.0]  # 负号 = 反转后轮方向
        # steer: position 字段 → motor_node 转 degree → set_absolute_position_degrees()
        msg.position = [0.0, float(steer_abs_rad)]
        self.cmd_pub.publish(msg)

    def _send_stop(self):
        """发送停止指令 (速度=0, 转向保持当前位置)"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['drive_motor', 'steer_motor']
        msg.velocity = [0.0, 0.0]
        # 保持在零位
        msg.position = [0.0, float(self.steer_center_rad)]
        self.cmd_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────
    # 数据记录
    # ────────────────────────────────────────────────────────────────

    def _log_row(self, t: float, steer_rate_ref: float, steer_abs_target: float):
        self.log.append({
            't': t,
            'roll_deg': self.roll_deg,
            'steer_deg': math.degrees(self.steer_rel_target),
            'roll_rate_dps': math.degrees(self.roll_rate_rad_s),
            'speed_ms': self.rear_speed_ms,
            'speed_ref_ms': self.speed_target_ms,
            'steer_rate_ref_dps': math.degrees(steer_rate_ref),
            'steer_target_deg': math.degrees(steer_abs_target - self.steer_center_rad),
            'K_delta': float(self.controller.K[0, 0]),
            'K_roll': float(self.controller.K[0, 1]),
            'K_roll_rate': float(self.controller.K[0, 2]),
            'saturated': int(abs(steer_rate_ref) >= self._p('max_steer_velocity') - 0.01),
            'rc_active': int(self.rc_active),
            'rc_speed_cmd': self.rc_speed_cmd,
            'rc_roll_cmd': math.degrees(self.rc_roll_cmd),
        })

    def save_log(self):
        """Ctrl+C 时自动保存 CSV"""
        if not self.log:
            return

        log_dir = self._p('log_dir') or os.path.expanduser('~/bike_balance')
        os.makedirs(log_dir, exist_ok=True)

        fname = os.path.join(
            log_dir,
            f"bike_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        with open(fname, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._log_fields)
            writer.writeheader()
            writer.writerows(self.log)

        self.get_logger().info(f"📝 日志已保存: {fname} ({len(self.log)} 行)")

    def destroy_node(self):
        self.get_logger().info("正在关闭...")
        self.save_log()
        self._send_stop()
        super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = BikeBalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C 收到, 正在保存日志...")
    finally:
        node.save_log()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
