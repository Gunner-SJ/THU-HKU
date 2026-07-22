#!/usr/bin/env python3
"""
符号验证脚本 — 只读传感器 + 算控制量，不控制电机。
用手推车，看终端打印的 steer_rate_ref 符号是否正确。

物理直觉: 车往右倒 → roll>0 → steer_rate_ref<0 (车把左转纠正) ← 正确
         车往左倒 → roll<0 → steer_rate_ref>0 (车把右转纠正) ← 正确

用法:  python3 test_signs.py
依赖:  pip install numpy scipy
       YbImuLib (需在 PYTHONPATH 中)
"""

import sys
import os
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation

# ═══════════════════════════════════════════════════════════════════
# 配置 — 改成你的实际值
# ═══════════════════════════════════════════════════════════════════

IMU_PORT = "/dev/ttyUSB1"               # YbIMU 串口
IMPORT_PATH = os.path.expanduser(
    "~/ws_ros2/src/imu_publisher/YbImuLib"
)                                       # YbImuLib 路径

# 物理参数
REAR_CONTACT_TO_COM = 0.117
WHEELBASE = 0.28223
TRAIL = 0.0140
COM_HEIGHT = 0.105
MASS = 2.218
ROLL_INERTIA = 0.02445
STEERING_AXIS_ANGLE = 0.179770          # rad ≈ 10.3°

# 控制参数
CONTROL_DT = 0.02                        # 50Hz
CONTROL_METHOD = "lqr"                   # "lqr" 或 "pole"
TARGET_SPEED = 1.5                       # 调度速度 (m/s)
MIN_SCHEDULING_SPEED = 0.5

# LQR 权重
Q_STEER = 4.0
Q_ROLL = 100.0
Q_ROLL_RATE = 2.0
R = 10.0
MAX_STEER_VELOCITY = 5.0                # rad/s

# 极点配置
POLE_WC = -5.0

# IMU 标定 (填你的实际值!)
SENSOR_TO_BIKE_RPY_DEG = [3.6196, 2.7760, -2.1262]
GYRO_BIAS_DPS = [-0.0024, 0.0000, 0.0000]

# ═══════════════════════════════════════════════════════════════════
# 嵌入式自行车模型 (精简版, 不依赖外部库)
# ═══════════════════════════════════════════════════════════════════

class BicycleModel:
    def __init__(self, dt, rear_contact_to_com, wheelbase, trail,
                 com_height, mass, roll_inertia, steering_axis_angle,
                 min_speed=0.5):
        self.dt = dt
        self.a = rear_contact_to_com
        self.b = wheelbase
        self.c = trail
        self.h = com_height
        self.m = mass
        self.I = roll_inertia
        self.lam = steering_axis_angle
        self.cos_l = math.cos(steering_axis_angle)
        self.min_speed = min_speed
        self.A = np.zeros((3, 3))
        self.Bu = np.zeros((3, 1))
        self.update(min_speed)

    def update(self, speed):
        g = 9.81
        v = max(self.min_speed, abs(speed))
        a1 = self.m * self.a * self.h * v * self.cos_l / (self.b * self.I)
        a2 = (self.m * v**2 * self.h - self.m * self.a * self.c * g) * self.cos_l / (self.b * self.I)
        a4 = self.m * g * self.h / self.I
        self.A = np.array([[0, 0, 0], [0, 0, 1], [a2, a4, 0]])
        self.Bu = np.array([[1], [0], [a1]])


class LQRController:
    def __init__(self, A, B, method="lqr", **kw):
        self.A = np.asarray(A, float)
        self.B = np.asarray(B, float)
        self.method = method
        self.kw = kw
        self.K = np.zeros((1, 3))
        self._compute()

    def _compute(self):
        from scipy.linalg import solve_continuous_are
        if self.method == "lqr":
            P = solve_continuous_are(self.A, self.B,
                                     np.diag([self.kw['q1'], self.kw['q2'], self.kw['q3']]),
                                     np.array([[self.kw['r']]]))
            self.K = np.linalg.solve(np.array([[self.kw['r']]]), self.B.T @ P)
        else:
            from scipy.signal import place_poles
            self.K = place_poles(self.A, self.B,
                                 np.full(3, self.kw['wc'])).gain_matrix

    def update_model(self, A, B):
        self.A, self.B = np.asarray(A, float), np.asarray(B, float)
        self._compute()

    def step(self, state):
        return float(np.clip(-self.K @ state, -MAX_STEER_VELOCITY, MAX_STEER_VELOCITY))


# ═══════════════════════════════════════════════════════════════════
# 主程序
# ═══════════════════════════════════════════════════════════════════

def main():
    # ── 导入 YbIMU ──
    sys.path.insert(0, IMPORT_PATH)
    try:
        from YbImuLib.YbImuSerialLib import YbImuSerial
    except ImportError:
        print("[ERROR] 找不到 YbImuSerialLib, 检查 IMPORT_PATH")
        print(f"  当前 IMPORT_PATH = {IMPORT_PATH}")
        sys.exit(1)

    # ── 初始化 ──
    print(f"连接 IMU: {IMU_PORT}")
    imu = YbImuSerial(IMU_PORT, debug=False)
    imu.create_receive_threading()
    time.sleep(0.5)

    model = BicycleModel(CONTROL_DT, REAR_CONTACT_TO_COM, WHEELBASE, TRAIL,
                         COM_HEIGHT, MASS, ROLL_INERTIA, STEERING_AXIS_ANGLE,
                         MIN_SCHEDULING_SPEED)
    model.update(TARGET_SPEED)

    ctrl = LQRController(model.A, model.Bu, method=CONTROL_METHOD,
                         q1=Q_STEER, q2=Q_ROLL, q3=Q_ROLL_RATE, r=R, wc=POLE_WC)

    R_sb = Rotation.from_euler("ZYX",
        np.deg2rad([SENSOR_TO_BIKE_RPY_DEG[2], SENSOR_TO_BIKE_RPY_DEG[1], SENSOR_TO_BIKE_RPY_DEG[0]]))
    gyro_bias = np.deg2rad(GYRO_BIAS_DPS)

    print(f"控制器: {CONTROL_METHOD}, K = {ctrl.K.flatten()}")
    print(f"  K_delta={ctrl.K[0,0]:.2f}  K_roll={ctrl.K[0,1]:.2f}  K_roll_rate={ctrl.K[0,2]:.2f}")
    print(f"安装角 RPY: {SENSOR_TO_BIKE_RPY_DEG}")
    print()
    print("=" * 72)
    print("  符号验证 — 手动推车测试")
    print("  车直立 → roll≈0, u≈0")
    print("  车往右倒 → roll>0, u<0 (负=左转) ✓")
    print("  车往左倒 → roll<0, u>0 (正=右转) ✓")
    print("  倒得越厉害 → u 绝对值越大")
    print("  Ctrl+C 退出")
    print("=" * 72)
    print()

    try:
        while True:
            # ① 读 IMU
            qw, qx, qy, qz = imu.get_imu_quaternion_data()
            gx, gy, gz = imu.get_gyroscope_data()

            # ② 姿态
            R_ws = Rotation.from_quat([qx, qy, qz, qw])
            R_wb = R_ws * R_sb.inv()
            _, pitch, roll = R_wb.as_euler("ZYX")

            # ③ 角速度
            gyro_corr = R_sb.apply(np.array([gx, gy, gz]) - gyro_bias)
            p, q, r = gyro_corr
            cp = math.cos(pitch)
            roll_rate = (p + math.sin(roll) * math.tan(pitch) * q
                         + math.cos(roll) * math.tan(pitch) * r) if abs(cp) > 1e-3 else 0.0

            # ④ 构造状态 (假设转向角=0，未连电机也可测)
            state = np.array([[0.0], [roll], [roll_rate]])

            # ⑤ 增益调度
            model.update(TARGET_SPEED)
            ctrl.update_model(model.A, model.Bu)

            # ⑥ 算控制量
            u = ctrl.step(state)
            u_dps = math.degrees(u)

            # ⑦ 判断方向
            if abs(roll) < 0.005:
                direction = "直立"
            elif roll > 0:
                direction = "右倾→应左转 ✓" if u < 0 else "右倾→方向错误 ✗✗✗"
            else:
                direction = "左倾→应右转 ✓" if u > 0 else "左倾→方向错误 ✗✗✗"

            print(f"roll={math.degrees(roll):+6.2f}°  "
                  f"rate={math.degrees(roll_rate):+6.2f}°/s  "
                  f"u={u_dps:+6.1f}°/s  [{direction}]  "
                  f"K_roll={ctrl.K[0,1]:.1f}",
                  end="\r")
            time.sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\n\n测试结束。")
        print("如果符号正确 → 标定完成, 可以进入阶段4闭合控制")
        print("如果符号反了 → 改 SENSOR_TO_BIKE_RPY_DEG 的 roll 分量 +180°")


if __name__ == '__main__':
    main()
