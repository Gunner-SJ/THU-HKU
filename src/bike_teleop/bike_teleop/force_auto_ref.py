#!/usr/bin/env python3
"""No-RC AUTO reference publisher.

Publishes /teleop/reference with linear.z=1 (AUTO) at 10 Hz so balance_executor
runs without a remote. Does not touch teleop_supervisor.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ForceAutoRef(Node):
    def __init__(self):
        super().__init__('force_auto_ref')
        self.pub = self.create_publisher(Twist, '/teleop/reference', 10)
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().warn(
            "force_auto_ref: publishing AUTO (linear.z=1) — no RC. "
            "Bike will use balance_executor target_speed."
        )
        self._tick()

    def _tick(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.z = 1.0  # AUTO
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForceAutoRef()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
