import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import math

class BikeTeleopNode(Node):
    def __init__(self):
        super().__init__('bike_teleop')
        
        # 1. Unified Publisher to the CAN motor node (using JointState)
        self.motor_pub = self.create_publisher(JointState, '/motor/command', 10)
        
        # 2. Publisher to your UART Servo node (using JointState)
        self.servo_pub = self.create_publisher(JointState, '/servo/command', 10)
        
        # 3. Subscriber to the SBUS Remote
        self.create_subscription(Joy, '/sbus/joy', self.joy_callback, 10)
        
        # --- Tuning Parameters ---
        # Maximum speed for the rear wheel in RPM
        self.MAX_RPM = 200.0 
        
        # Maximum steering angle in degrees (Left and Right)
        self.MAX_STEER_DEG = 30.0 
        
        # UART Servos swing range (in radians).
        self.SERVO_CENTER_RAD_1 = 2.72
        self.SERVO_CENTER_RAD_2 = 0.75
        self.SERVO_MAX_SWING_RAD = math.radians(45.0)

        # Kickstand specific positions (in radians)
        self.KICKSTAND_RAD_1 = 2.72
        self.KICKSTAND_RAD_2 = 0.75
        self.get_logger().info("Bike Teleop Node Started! Awaiting RC input...")
        self.get_logger().info("⚠️ SAFETY: SWA Switch MUST be DOWN (1) to enable movement!")

    def joy_callback(self, msg: Joy):
        # 1. Safety Check using Switch SWA (buttons[0])
        # From our SBUS node: -1 is UP, 0 is MID, 1 is DOWN
        safety_switch = msg.buttons[0]
        
        if safety_switch != 1:
            # If safety is not engaged, force all motors to 0 and DEPLOY KICKSTANDS
            stop_msg = JointState()
            stop_msg.name = ['drive_motor', 'steer_motor']
            stop_msg.velocity = [0.0, 0.0]
            stop_msg.position = [0.0, 0.0]
            self.motor_pub.publish(stop_msg)
            
            servo_msg = JointState()
            servo_msg.name = ['servo_1', 'servo_2']
            servo_msg.position = [self.KICKSTAND_RAD_1, self.KICKSTAND_RAD_2]
            self.servo_pub.publish(servo_msg)
            return

        # 2. CAN Motors Mapping (Drive and Steer)
        motor_msg = JointState()
        motor_msg.name = ['drive_motor', 'steer_motor']
        
        # Convert Throttle to RPM -> rad/s (Expected by motor_node)
        throttle_input = -msg.axes[1] #inverted
        target_rpm = throttle_input * self.MAX_RPM
        target_rad_s = target_rpm * (2 * math.pi / 60.0)
        
        # Convert Steering to Degrees -> rad (Expected by motor_node)
        steer_input = -msg.axes[0]
        # Note: If your steering is backward, change this to `-steer_input * self.MAX_STEER_DEG`
        target_steer_deg = steer_input * self.MAX_STEER_DEG 
        target_steer_rad = math.radians(target_steer_deg)
        
        motor_msg.velocity = [target_rad_s, 0.0]
        motor_msg.position = [0.0, target_steer_rad]
        self.motor_pub.publish(motor_msg)
        
        # 3. Map UART Servos (Left Joystick Horizontal -> msg.axes[3])
        servo_input = msg.axes[3]
        target_servo_1 = self.SERVO_CENTER_RAD_1 + (servo_input * self.SERVO_MAX_SWING_RAD)
        target_servo_2 = self.SERVO_CENTER_RAD_2 - (servo_input * self.SERVO_MAX_SWING_RAD) # Invert if needed
        
        servo_msg = JointState()
        servo_msg.name = ['servo_1', 'servo_2']
        servo_msg.position = [target_servo_1, target_servo_2]
        self.servo_pub.publish(servo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BikeTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send one last stop command before shutting down
        stop_msg = JointState()
        stop_msg.name = ['drive_motor', 'steer_motor']
        stop_msg.velocity = [0.0, 0.0]
        stop_msg.position = [0.0, 0.0]
        node.motor_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()