#!/usr/bin/env python3
"""Kickstands from measured drive speed only — no RC, no balance_executor.

Hysteresis:
  speed >= retract → arms UP
  speed <= deploy  → arms DOWN
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class KickstandSpeedNode(Node):
    def __init__(self) -> None:
        super().__init__('kickstand_speed')

        self.declare_parameter('wheel_radius', 0.0725)
        self.declare_parameter('kickstand_speed_deploy', 1.2)
        self.declare_parameter('kickstand_speed_retract', 1.5)
        self.declare_parameter('servo_center_rad_1', 2.82)
        self.declare_parameter('servo_center_rad_2', 0.72)
        self.declare_parameter('servo_max_swing_rad', 0.7854)
        self.declare_parameter('kickstand_lean_bias_rad', 0.0)
        self.declare_parameter('servo_cmd_topic', '/servo/command')
        self.declare_parameter('motor_state_topic', '/motor/state')
        self.declare_parameter('control_dt', 0.05)

        self._speed_ms = 0.0
        self._lift_filt = 0.0
        self._lift_sent = -1.0

        self._pub = self.create_publisher(
            JointState, self.get_parameter('servo_cmd_topic').value, 10
        )
        self.create_subscription(
            JointState,
            self.get_parameter('motor_state_topic').value,
            self._on_motor,
            10,
        )
        dt = float(self.get_parameter('control_dt').value)
        self.create_timer(dt, self._tick)

        self.get_logger().info(
            "kickstand_speed: arms follow measured speed only "
            f"(down≤{self.get_parameter('kickstand_speed_deploy').value} "
            f"up≥{self.get_parameter('kickstand_speed_retract').value} m/s)"
        )

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _on_motor(self, msg: JointState) -> None:
        try:
            idx = msg.name.index('drive_motor')
            omega = float(msg.velocity[idx])
            self._speed_ms = abs(omega * float(self._p('wheel_radius')))
        except (ValueError, IndexError):
            pass

    def _tick(self) -> None:
        v = self._speed_ms
        v_low = float(self._p('kickstand_speed_deploy'))
        v_high = float(self._p('kickstand_speed_retract'))

        if self._lift_filt >= 0.5:
            lift_raw = 0.0 if v <= v_low else 1.0
        else:
            lift_raw = 1.0 if v >= v_high else 0.0

        alpha = 0.12
        self._lift_filt = alpha * lift_raw + (1.0 - alpha) * self._lift_filt
        if self._lift_filt >= 0.95:
            self._lift_filt = 1.0
        elif self._lift_filt <= 0.05:
            self._lift_filt = 0.0

        if abs(self._lift_filt - self._lift_sent) < 0.02:
            return
        self._publish_servos(self._lift_filt)
        self._lift_sent = self._lift_filt

    def _publish_servos(self, lift_ratio: float) -> None:
        c1 = float(self._p('servo_center_rad_1'))
        c2 = float(self._p('servo_center_rad_2'))
        swing = float(self._p('servo_max_swing_rad'))
        bias = float(self._p('kickstand_lean_bias_rad'))
        lift = float(lift_ratio)
        d = bias * max(0.0, min(1.0, 1.0 - lift))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['servo_1', 'servo_2']
        msg.position = [
            float(c1 - lift * swing - d),
            float(c2 + lift * swing + d),
        ]
        self._pub.publish(msg)
        self.get_logger().info(
            f"KS → {'UP' if lift >= 0.95 else 'DOWN' if lift <= 0.05 else f'{lift*100:.0f}%'} "
            f"(v={self._speed_ms:.2f} m/s)",
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KickstandSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
