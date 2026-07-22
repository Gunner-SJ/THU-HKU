import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import serial
import time
import math

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        
        # Hardware parameters configured during debugging
        self.port_name = '/dev/ttyAMA2'
        self.baud_rate = 1000000
        self.servo_id = 2
        
        # Initialize Serial Connection
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=0.01
            )
            self.get_logger().info(f"Successfully opened {self.port_name} at {self.baud_rate} bps for Servo ID {self.servo_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to open UART port {self.port_name}: {e}")
            raise e

        # ROS 2 Interfaces
        # 1. Publisher: Outputs current servo angle, velocity, and effort
        self.state_pub = self.create_publisher(JointState, '/servo/state', 10)
        
        # 2. Subscriber: Accepts target position in radians (or degrees if converted)
        self.cmd_sub = self.create_subscription(JointState, '/servo/command', self.command_callback, 10)

        # Timer to poll servo state at 20Hz (50ms)
        self.timer = self.create_timer(0.05, self.publish_servo_state)
        
        # Internal state tracking
        self.current_position_deg = 150.0  # Center position default
        self.current_velocity_deg = 0.0

    def calculate_checksum(self, packet_without_headers):
        """Calculates standard bitwise NOT checksum for serial servo packets."""
        return (~sum(packet_without_headers)) & 0xFF

    def send_servo_command(self, s_id, cmd, params):
        """Constructs and sends a serial byte packet to the URT-2 driver board."""
        length = len(params) + 2
        packet = [s_id, length, cmd] + params
        checksum = self.calculate_checksum(packet)
        full_packet = bytearray([0xFF, 0xFF] + packet + [checksum])
        
        try:
            self.ser.write(full_packet)
        except Exception as e:
            self.get_logger().warn(f"UART write error: {e}")

    def command_callback(self, msg):
        """Handles incoming JointState commands for multiple servos."""
        for i, name in enumerate(msg.name):
            target_rad = msg.position[i]
            target_deg = math.degrees(target_rad)
            target_deg = max(0.0, min(300.0, target_deg))
            
            # Convert degrees to 12-bit resolution (0 to 4095)
            raw_pos = int((target_deg / 300.0) * 4095)
            pos_l = raw_pos & 0xFF
            pos_h = (raw_pos >> 8) & 0xFF
            
            # Determine Target ID based on joint name
            target_id = None
            if "servo_1" in name or "id_1" in name or name == "1":
                target_id = 1
            elif "servo_2" in name or "id_2" in name or name == "2":
                target_id = 2
            elif "broadcast" in name or "all" in name:
                target_id = 254  # Broadcast to all servos!
                
            if target_id is not None:
                # Send standard WRITE_DATA (0x03) to Address 0x2A (Goal Position)
                self.send_servo_command(target_id, 0x03, [0x2A, pos_l, pos_h, 0x00, 0x04])
                self.get_logger().debug(f"Commanded ID {target_id} -> {target_deg:.1f} deg")

    def publish_servo_state(self):
        """Queries servo feedback over UART and publishes to /servo/state."""
        # Instruction 0x02 is READ_DATA; Address 0x38 is Present Position (2 bytes)
        self.send_servo_command(self.servo_id, 0x02, [0x38, 0x02])
        
        # Read response frame if available
        if self.ser.in_waiting >= 8:
            try:
                response = self.ser.read(8)
                if response[0] == 0xFF and response[1] == 0xFF and response[2] == self.servo_id:
                    pos_h = response[6]
                    raw_pos = pos_l | (pos_h << 8)
                    
                    # Convert raw steps back to degrees and radians
                    self.current_position_deg = (raw_pos / 4095.0) * 300.0
            except Exception as e:
                self.get_logger().debug(f"UART read parsing error: {e}")

        # Construct and publish JointState message
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = [f"servo_joint_{self.servo_id}"]
        
        # ROS 2 standard uses radians for angular joints
        pos_rad = math.radians(self.current_position_deg)
        state_msg.position = [pos_rad]
        state_msg.velocity = [0.0]  # Populate if reading velocity registers
        state_msg.effort = [0.0]    # Populate if reading load registers
        
        self.state_pub.publish(state_msg)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()