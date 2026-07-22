#!/usr/bin/env python3
"""RC supervisor: maps SBUS Joy → /teleop/reference for balance_executor.

Kickstands are owned by kickstand_speed (measured speed only) — this node
does NOT publish /servo/command.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TeleopSupervisorNode(Node):
    def __init__(self):
        super().__init__('teleop_supervisor')

        # Publishers — kickstands owned by kickstand_speed (measured speed only).
        self.ref_pub = self.create_publisher(Twist, '/teleop/reference', 10)

        self.create_subscription(Joy, '/sbus/joy', self.joy_callback, 10)

        # --- Tuning Parameters ---
        self.MAX_SPEED_MPS = 2.7                 # Max forward speed in m/s (Manual)
        self.MAX_LEAN_RAD = math.radians(10.0)   # Max lean command (10 deg)

        # --- RC Watchdog Failsafe ---
        self.last_joy_time = self.get_clock().now().nanoseconds / 1e9
        self.rc_timeout_sec = 0.5  # If no signal for 0.5s, engage safety
        self.timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            "Supervisor Node Started! SWA: -1(Kill), 0(Manual), 1(Auto). "
            "Kickstands: kickstand_speed only (same for Manual/Auto)."
        )

    def engage_safety(self):
        """Kill drive only; arms follow speed→0 via kickstand_speed."""
        ref_msg = Twist()
        ref_msg.linear.z = -1.0  # -1.0 means Safety ON / Disabled
        self.ref_pub.publish(ref_msg)

    def watchdog_callback(self):
        """Monitors RC connection and triggers safety if connection is lost."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_joy_time > self.rc_timeout_sec:
            self.get_logger().warn(
                "⚠️ RC SIGNAL LOST! Engaging Safety Failsafe.",
                throttle_duration_sec=1.0,
            )
            self.engage_safety()

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now().nanoseconds / 1e9

        swa_switch = msg.buttons[0]  # Expected to be -1, 0, or 1

        if swa_switch == -1:
            self.engage_safety()
            return

        throttle_input = msg.axes[1]
        steer_input = msg.axes[0]

        ref_msg = Twist()
        ref_msg.linear.x = throttle_input * self.MAX_SPEED_MPS
        ref_msg.angular.z = steer_input * self.MAX_LEAN_RAD
        ref_msg.linear.z = float(swa_switch)  # 0=Manual, 1=Auto
        self.ref_pub.publish(ref_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
