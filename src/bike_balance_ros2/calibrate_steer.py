#!/usr/bin/env python3
"""
转向零位 & 范围标定工具
=======================

用法:
  1. 先启动 can_motor_controller:
     ros2 run can_motor_controller motor_controller_node

  2. 再跑本脚本:
     python3 calibrate_steer.py

  3. 按提示操作, 每次摆好车把后按 Enter 记录。
     屏幕会实时显示当前角度, 方便你精调位置。
"""

import math
import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SteerCalibrator(Node):
    def __init__(self):
        super().__init__('steer_calibrator')
        self.steer_angle_deg = 0.0
        self.has_data = False

        self.sub = self.create_subscription(
            JointState, '/motor/state', self._on_state, 10)

    def _on_state(self, msg: JointState):
        try:
            idx = msg.name.index('steer_motor')
            self.steer_angle_deg = math.degrees(float(msg.position[idx]))
            self.has_data = True
        except ValueError:
            pass

    @property
    def angle(self) -> float:
        return self.steer_angle_deg


def shortest_angular_distance(a: float, b: float) -> float:
    """
    从角度 a 到角度 b 的最短距离 (度).
    正值 = 一侧, 负值 = 另一侧
    """
    diff = (b - a) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff


def record_position(node: SteerCalibrator, label: str) -> float:
    """
    实时显示当前角度, 用户摆好车把后按 Enter 记录.
    采样 20 次取平均.
    """
    print(f"\n{'='*55}")
    print(f"  {label}")
    print(f"{'='*55}")
    print(f"  当前角度会实时刷新, 摆好后按 Enter 记录...")
    print()

    # 启动一个 flag, 主线程 spin 显示角度直到用户按下 Enter
    done = threading.Event()
    samples = []
    lock = threading.Lock()

    def spin_loop():
        """后台线程持续读角度并刷新显示"""
        while not done.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.has_data:
                with lock:
                    samples.append(node.angle)
                    # 只保留最近 5 个用于显示
                    if len(samples) > 100:
                        samples.pop(0)
                # 实时显示
                print(f"\r  >>> 当前角度: {node.angle:6.1f}°  (摆好后按 Enter)", end="", flush=True)

    t = threading.Thread(target=spin_loop, daemon=True)
    t.start()

    input()  # 等用户按 Enter
    done.set()
    t.join(timeout=0.3)

    # 取最后 20 个采样点
    with lock:
        recent = samples[-20:] if len(samples) >= 20 else samples

    if not recent:
        print("\n  [ERROR] 没收到数据! 检查 motor_controller_node 是否在运行")
        return 0.0

    avg = sum(recent) / len(recent)
    print(f"\n  → 采样 20 次, 平均角度 = {avg:.1f}°")
    return avg


def main():
    rclpy.init(args=sys.argv)
    node = SteerCalibrator()

    print("等待电机数据...", end="", flush=True)
    import time
    start = time.time()
    while not node.has_data and time.time() - start < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        print(".", end="", flush=True)

    if not node.has_data:
        print("\n[ERROR] 超时! 请确认:")
        print("  1. can_motor_controller 是否在运行?")
        print("  2. CAN 总线是否已配置?")
        print("  3. 电机是否通电?")
        node.destroy_node()
        rclpy.try_shutdown()
        return

    print(f" OK! 当前角度: {node.angle:.1f}°")

    # ═══════════════════════════════════════════════════
    print()
    print("=" * 55)
    print("  转向零位 & 范围标定")
    print("=" * 55)
    print()
    print("  1. 把车把摆到【正中】(前轮正对前方)")
    print("  2. 把车把【左转到底】")
    print("  3. 把车把【右转到底】")
    print()
    print("  每次摆好后按 Enter, 实时角度会持续刷新")
    print("=" * 55)

    center = record_position(node, "【1/3】车把正中 (前轮正前方)")
    left   = record_position(node, "【2/3】车把左转到底 (机械极限)")
    right  = record_position(node, "【3/3】车把右转到底 (机械极限)")

    # ── 计算 ──
    left_dist  = shortest_angular_distance(center, left)
    right_dist = shortest_angular_distance(center, right)
    left_range  = abs(left_dist)
    right_range = abs(right_dist)
    max_delta = min(left_range, right_range) - 5.0

    print()
    print("=" * 55)
    print("  标定结果")
    print("=" * 55)
    print(f"  正中:  {center:.1f}°")
    print(f"  左极限: {left:.1f}°  (范围 {left_range:.1f}°)")
    print(f"  右极限: {right:.1f}°  (范围 {right_range:.1f}°)")
    print()
    print(f"  >>> steer_center_deg = {center:.1f}")
    print(f"  >>> steer_max_delta_deg = {max_delta:.1f}")
    print()
    print("  填入 launch/balance.launch.py:")
    print(f"    'steer_center_deg': {center:.1f},")
    print(f"    'steer_max_delta_deg': {max_delta:.1f},")

    if max_delta < 5.0:
        print()
        print("  ⚠️ 可用转向范围 < 5°, 可能不够!")

    print("=" * 55)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
