#!/usr/bin/env python3
"""
Automated Steering Limit Discovery Node (MIT Mode Compatible)
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SteerLimitTesterNode(Node):
    def __init__(self):
        super().__init__('steer_limit_tester')

        self.declare_parameter('state_topic', '/motor/state')
        self.declare_parameter('cmd_topic', '/motor/command')
        self.declare_parameter('sweep_rate_rad_s', 0.05)       # Slow sweep: ~2.8 deg/s
        self.declare_parameter('stall_error_rad', 0.025)       # Error threshold: ~1.4 deg
        self.declare_parameter('stall_time_s', 0.5)            # Must be stalled for 0.5s
        self.declare_parameter('max_sweep_dist_rad', 1.2)      # Safety limit: ~68 degrees

        self.state_topic = self.get_parameter('state_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.sweep_rate = float(self.get_parameter('sweep_rate_rad_s').value)
        self.stall_error = float(self.get_parameter('stall_error_rad').value)
        self.stall_time_thresh = float(self.get_parameter('stall_time_s').value)
        self.max_sweep_dist = float(self.get_parameter('max_sweep_dist_rad').value)

        self.state = 'IDLE'
        self.measured_pos_rad: Optional[float] = None
        self.target_pos_rad: float = 0.0
        self.start_pos_rad: float = 0.0
        
        self.stall_start_time: Optional[float] = None
        self.max_limit_rad: Optional[float] = None
        self.min_limit_rad: Optional[float] = None

        self.state_sub = self.create_subscription(JointState, self.state_topic, self._on_state, 10)
        self.cmd_pub = self.create_publisher(JointState, self.cmd_topic, 10)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(f"MIT Limit Tester Started. Listening to {self.state_topic}...")

    def _on_state(self, msg: JointState):
        try:
            if 'steer_motor' in msg.name:
                idx = msg.name.index('steer_motor')
                self.measured_pos_rad = float(msg.position[idx])
            elif len(msg.position) > 1:
                self.measured_pos_rad = float(msg.position[1])
        except (ValueError, IndexError):
            pass

    def _control_loop(self):
        if self.measured_pos_rad is None:
            self.get_logger().info("Waiting for MIT steer telemetry...", throttle_duration_sec=2.0)
            return

        now = time.monotonic()

        # STATE 1: IDLE
        if self.state == 'IDLE':
            self.start_pos_rad = self.measured_pos_rad
            self.target_pos_rad = self.measured_pos_rad
            self.state = 'SWEEP_POS'
            self.get_logger().info(
                f"Starting position established at {self.start_pos_rad:.4f} rad "
                f"({math.degrees(self.start_pos_rad):.1f}°). Sweeping POSITIVE direction..."
            )

        # STATE 2: SWEEP_POS (Right/Upper Limit)
        elif self.state == 'SWEEP_POS':
            self.target_pos_rad += self.sweep_rate * self.dt
            error = self.target_pos_rad - self.measured_pos_rad

            if abs(self.target_pos_rad - self.start_pos_rad) > self.max_sweep_dist:
                self.max_limit_rad = self.measured_pos_rad
                self.get_logger().warn(
                    f"🛑 SWEEP CUTOFF REACHED (POSITIVE DIRECTION)\n"
                    f" 👉 CURRENT POSITION (Right Limit): {self.max_limit_rad:.4f} rad "
                    f"({math.degrees(self.max_limit_rad):.2f}°)"
                )
                self.target_pos_rad = self.measured_pos_rad
                self.state = 'SWEEP_NEG'
                return

            if error > self.stall_error:
                if self.stall_start_time is None: self.stall_start_time = now
                elif (now - self.stall_start_time) > self.stall_time_thresh:
                    self.max_limit_rad = self.measured_pos_rad
                    self.get_logger().info(f"🛑 UPPER LIMIT FOUND: {self.max_limit_rad:.4f} rad ({math.degrees(self.max_limit_rad):.2f}°)")
                    self.target_pos_rad = self.measured_pos_rad
                    self.stall_start_time = None
                    self.state = 'SWEEP_NEG'
                    self.get_logger().info("Sweeping NEGATIVE direction...")
            else:
                self.stall_start_time = None

        # STATE 3: SWEEP_NEG (Left/Lower Limit)
        elif self.state == 'SWEEP_NEG':
            self.target_pos_rad -= self.sweep_rate * self.dt
            error = self.measured_pos_rad - self.target_pos_rad

            if abs(self.target_pos_rad - self.start_pos_rad) > self.max_sweep_dist:
                self.min_limit_rad = self.measured_pos_rad
                self.get_logger().warn(
                    f"🛑 SWEEP CUTOFF REACHED (NEGATIVE DIRECTION)\n"
                    f" 👉 CURRENT POSITION (Left Limit): {self.min_limit_rad:.4f} rad "
                    f"({math.degrees(self.min_limit_rad):.2f}°)"
                )
                self.target_pos_rad = self.measured_pos_rad
                self.state = 'CENTERING'
                return

            if error > self.stall_error:
                if self.stall_start_time is None: self.stall_start_time = now
                elif (now - self.stall_start_time) > self.stall_time_thresh:
                    self.min_limit_rad = self.measured_pos_rad
                    self.get_logger().info(f"🛑 LOWER LIMIT FOUND: {self.min_limit_rad:.4f} rad ({math.degrees(self.min_limit_rad):.2f}°)")
                    self.target_pos_rad = self.measured_pos_rad
                    self.stall_start_time = None
                    self.state = 'CENTERING'
            else:
                self.stall_start_time = None

        # STATE 4: CENTERING
        elif self.state == 'CENTERING':
            center_rad = (self.max_limit_rad + self.min_limit_rad) / 2.0
            self.target_pos_rad = center_rad
            if abs(self.measured_pos_rad - center_rad) < 0.01:
                self.state = 'DONE'
                self._print_final_summary()

        self._send_command(self.target_pos_rad)

    def _send_command(self, pos_rad: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['steer_motor']
        msg.position = [float(pos_rad)]
        self.cmd_pub.publish(msg)

    def _print_final_summary(self):
        center_rad = (self.max_limit_rad + self.min_limit_rad) / 2.0
        delta_rad = (self.max_limit_rad - self.min_limit_rad) / 2.0
        center_deg, delta_deg = math.degrees(center_rad), math.degrees(delta_rad)

        print("\n" + "="*60)
        print(" 🎯 MIT STEERING LIMIT DISCOVERY COMPLETE! ")
        print("="*60)
        print(f" 👉 TRUE CENTER OFFSET : {center_rad:7.4f} rad  ({center_deg:6.2f}°)")
        print(f" 👉 MAX SWING (Δ)      : ±{delta_rad:6.4f} rad  (±{delta_deg:5.2f}°)")
        print("="*60)
        print("\n📋 Copy these parameters into your LQR balance_executor.py:\n")
        print(f"self.declare_parameter('steer_center_deg', {center_deg:.2f})")
        print(f"self.declare_parameter('steer_max_delta_deg', {delta_deg:.2f})")
        print("="*60 + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = SteerLimitTesterNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()