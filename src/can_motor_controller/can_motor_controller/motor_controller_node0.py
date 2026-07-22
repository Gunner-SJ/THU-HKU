import math
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Import from the adjacent driver files
from .can_interface import CANInterface, CANCommunicationError
from .motor import Motor

class MotorControllerNode0(Node):
    """
    ROS 2 node wrapping GIM3510 / GIM3505 motors over a single CAN bus.
    
    ── Features ──
    • Separated Drive and Steer ROS 2 topics.
    • Multi-turn angle tracking on Steer motor to eliminate 0/2pi boundary wrapping.
    • Watchdog safety auto-stop for Drive motor.
    """
    def __init__(self):
        super().__init__('motor_controller_node0')
        
        # --- CAN & Motor Parameters ---
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('drive_addr', 1)
        self.declare_parameter('steer_addr', 2)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.2)
        
        # --- Topic Name Parameters ---
        self.declare_parameter('drive_state_topic', '/motor/drive/state')
        self.declare_parameter('steer_state_topic', '/motor/steer/state')
        self.declare_parameter('drive_cmd_topic', '/motor/drive/command')
        self.declare_parameter('steer_cmd_topic', '/motor/steer/command')
        
        self._can_channel = self.get_parameter('can_channel').value
        self._drive_addr = self.get_parameter('drive_addr').value
        self._steer_addr = self.get_parameter('steer_addr').value
        publish_rate = self.get_parameter('publish_rate').value
        self._cmd_timeout = self.get_parameter('cmd_timeout').value
        
        # --- Pending commands & Threading ---
        self._lock = threading.Lock()
        self._target_rpm: float | None = None
        self._target_steer_deg: float | None = None
        self._last_cmd_time = time.monotonic()
        self._watchdog_triggered = False
        
        # --- CAN & Motors ---
        self._can: CANInterface | None = None
        self._drive_motor: Motor | None = None
        self._steer_motor: Motor | None = None
        self._can_ok = False
        self._can_configured = False
        
        # --- ROS 2 Interfaces (Separated Topics) ---
        drive_state_topic = self.get_parameter('drive_state_topic').value
        steer_state_topic = self.get_parameter('steer_state_topic').value
        drive_cmd_topic = self.get_parameter('drive_cmd_topic').value
        steer_cmd_topic = self.get_parameter('steer_cmd_topic').value
        
        self.drive_state_pub = self.create_publisher(JointState, drive_state_topic, 10)
        self.steer_state_pub = self.create_publisher(JointState, steer_state_topic, 10)
        
        self.drive_cmd_sub = self.create_subscription(
            JointState, drive_cmd_topic, self.drive_command_callback, 10
        )
        self.steer_cmd_sub = self.create_subscription(
            JointState, steer_cmd_topic, self.steer_command_callback, 10
        )
        
        # --- Timer (sole CAN access point) ---
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self.timer_callback)
        self._tick_count = 0
        
        self.get_logger().info(
            f"Separated Motor Node Started | Drive ID: 0x{self._drive_addr:02X} ({drive_state_topic}) | "
            f"Steer ID: 0x{self._steer_addr:02X} ({steer_state_topic})"
        )
        
    def _ensure_can_open(self) -> bool:
        """Lazy-open CAN and create Motor objects. Returns True if ready."""
        if self._can_ok:
            return True
        try:
            self._can = CANInterface(channel=self._can_channel, baudrate=1000000)
            self._can.open()
            self._drive_motor = Motor(self._can, dev_addr=self._drive_addr, timeout=self._cmd_timeout)
            self._steer_motor = Motor(self._can, dev_addr=self._steer_addr, timeout=self._cmd_timeout)
            
            # Enable motors so they accept commands
            self._drive_motor.enable()
            self._steer_motor.enable()
            
            self._can_ok = True
            self._can_configured = False
            self.get_logger().info('CAN bus opened OK & Motors Enabled.')
            return True
        except Exception as e:
            self.get_logger().warn(f'CAN not available yet: {e}')
            return False
            
    def drive_command_callback(self, msg: JointState):
        """Dedicated callback for Drive Motor commands (Expects velocity in rad/s)."""
        with self._lock:
            # Find velocity index (default to 0 if name matching isn't used)
            idx = 0
            if 'drive_motor' in msg.name:
                idx = msg.name.index('drive_motor')
            elif 'motor_1' in msg.name:
                idx = msg.name.index('motor_1')
                
            if len(msg.velocity) > idx:
                target_rad_s = msg.velocity[idx]
                self._target_rpm = target_rad_s * (60.0 / (2 * math.pi))
                self._last_cmd_time = time.monotonic()
                self._watchdog_triggered = False

    def steer_command_callback(self, msg: JointState):
        """Dedicated callback for Steer Motor commands (Expects position in rad)."""
        with self._lock:
            # Find position index (default to 0 if name matching isn't used)
            idx = 0
            if 'steer_motor' in msg.name:
                idx = msg.name.index('steer_motor')
            elif 'motor_2' in msg.name:
                idx = msg.name.index('motor_2')
                
            if len(msg.position) > idx:
                target_rad = msg.position[idx]
                self._target_steer_deg = math.degrees(target_rad)
                    
    def timer_callback(self):
        self._tick_count += 1
        
        # 1. Ensure CAN is open
        if not self._ensure_can_open():
            return
            
        # 2. Grab pending commands
        with self._lock:
            target_rpm = self._target_rpm
            target_steer = self._target_steer_deg
            self._target_rpm = None
            self._target_steer_deg = None
            
        # 3. Read motor statuses & angles
        drive_status = None
        steer_status = None
        steer_angle_info = None
        
        try:
            drive_status = self._drive_motor.read_status_all()
        except Exception as e:
            self.get_logger().debug(f'Drive read status failed: {e}')
            
        try:
            steer_status = self._steer_motor.read_status_all()
        except Exception as e:
            self.get_logger().debug(f'Steer read status failed: {e}')
            
        # --- CRITICAL FIX: Read Multi-Turn Angle for Steering ---
        # Command 0xA3 returns multi-turn angle, preventing the 0/2pi wrap-around trap!
        try:
            steer_angle_info = self._steer_motor.read_angle()
        except Exception as e:
            self.get_logger().debug(f'Steer read multi-turn angle failed: {e}')
            
        # 4. Configure CAN timeout once
        if not self._can_configured and (drive_status is not None or steer_status is not None):
            self._can_configured = True
            try:
                if drive_status is not None: self._drive_motor.configure_can_timeout(enabled=True, timeout_ms=1000)
                if steer_status is not None: self._steer_motor.configure_can_timeout(enabled=True, timeout_ms=1000)
                self.get_logger().info('CAN timeouts configured: 1s')
            except Exception:
                pass
                
        # 5. Watchdog: auto-stop drive motor if no cmd for >1s
        if (not self._watchdog_triggered 
                and time.monotonic() - self._last_cmd_time > 1.0 
                and drive_status is not None 
                and abs(drive_status.speed) > 0.5):
            try:
                self._drive_motor.set_speed(0.0)
                self._watchdog_triggered = True
                self.get_logger().info('Watchdog: drive speed set to 0 (no cmd for >1s)')
            except Exception:
                pass
                
        # 6. Send commands over CAN
        if target_rpm is not None and drive_status is not None:
            try:
                self._drive_motor.set_speed(target_rpm)
            except Exception as e:
                self.get_logger().warn(f'Drive cmd failed: {e}')
                
        if target_steer is not None and steer_status is not None:
            try:
                # Absolute position command (Command 0xC2)
                self._steer_motor.set_absolute_position_degrees(target_steer)
            except Exception as e:
                self.get_logger().warn(f'Steer cmd failed: {e}')
                
        # 7. Publish Drive JointState (/motor/drive/state)
        drive_msg = JointState()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.name = ['drive_motor']
        drive_msg.position = [math.radians(drive_status.angle) if drive_status else 0.0]
        drive_msg.velocity = [drive_status.speed * (2 * math.pi / 60.0) if drive_status else 0.0]
        drive_msg.effort = [drive_status.q_current if drive_status else 0.0]
        self.drive_state_pub.publish(drive_msg)
        
        # 8. Publish Steer JointState (/motor/steer/state)
        steer_msg = JointState()
        steer_msg.header.stamp = self.get_clock().now().to_msg()
        steer_msg.name = ['steer_motor']
        
        # Use continuous multi_turn_angle if available; fallback to single-turn if read_angle() failed
        if steer_angle_info is not None:
            steer_pos_rad = math.radians(steer_angle_info.multi_turn_angle)
        elif steer_status is not None:
            steer_pos_rad = math.radians(steer_status.angle)
        else:
            steer_pos_rad = 0.0
            
        steer_msg.position = [steer_pos_rad]
        steer_msg.velocity = [steer_status.speed * (2 * math.pi / 60.0) if steer_status else 0.0]
        steer_msg.effort = [steer_status.q_current if steer_status else 0.0]
        self.steer_state_pub.publish(steer_msg)
            
    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        self._can_ok = False
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
        if self._drive_motor:
            try: self._drive_motor.disable()
            except: pass
        if self._steer_motor:
            try: self._steer_motor.disable()
            except: pass
        if self._can and self._can.is_open:
            try: self._can.close()
            except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode0()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    
    node.destroy_node()
    try:
        rclpy.shutdown()
    except:
        pass

if __name__ == '__main__':
    main()