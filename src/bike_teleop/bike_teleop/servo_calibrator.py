import json
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger # Or a custom service, using Trigger for simplicity

class ServoCalibrator(Node):
    def __init__(self):
        super().__init__('servo_calibrator')
        self.calib_file = os.path.expanduser('~/.bike_servo_calib.json')
        
        self.create_subscription(JointState, '/servo/state', self.state_callback, 10)
        self._current_pos = {"1": 0.0, "2": 0.0}
        
        # Dictionary structure for both servos
        self.calibrations = {
            "1": {"zero": 0.0, "kickstand": 0.0, "max": 0.0},
            "2": {"zero": 0.0, "kickstand": 0.0, "max": 0.0}
        }
        self.load_calibrations()
        
        # Create a service for each servo and position combination
        for s_id in ["1", "2"]:
            for key in ["zero", "kickstand", "max"]:
                srv_name = f'/calibrate/servo{s_id}/{key}'
                self.create_service(Trigger, srv_name, self.make_callback(s_id, key))

        self.get_logger().info("Calibrator ready.")
        self.get_logger().info("Call /calibrate/servo1/{zero|kickstand|max} to save Servo 1.")
        self.get_logger().info("Call /calibrate/servo2/{zero|kickstand|max} to save Servo 2.")

    def make_callback(self, s_id, key):
        """Helper to capture the s_id and key correctly for the lambda."""
        return lambda req, res: self.handle_save(req, res, s_id, key)

    def handle_save(self, request, response, s_id, key):
        self.save_position(s_id, key)
        response.success = True
        response.message = f"Saved Servo {s_id} {key} as {self._current_pos[s_id]:.4f}"
        return response

    def state_callback(self, msg):
        # Parse the JointState arrays to match positions to the right servo IDs
        for i, name in enumerate(msg.name):
            if "1" in name:
                self._current_pos["1"] = msg.position[i]
            elif "2" in name:
                self._current_pos["2"] = msg.position[i]

    def load_calibrations(self):
        if os.path.exists(self.calib_file):
            try:
                with open(self.calib_file, 'r') as f:
                    data = json.load(f)
                    # Safely update existing dict to ensure no keys are missing
                    for s_id in ["1", "2"]:
                        if s_id in data:
                            self.calibrations[s_id].update(data[s_id])
            except Exception as e:
                self.get_logger().warn(f"Error loading calibrations: {e}")

    def save_position(self, s_id, key):
        self.calibrations[s_id][key] = self._current_pos[s_id]
        with open(self.calib_file, 'w') as f:
            json.dump(self.calibrations, f)
        self.get_logger().info(f"Saved Servo {s_id} {key} position: {self._current_pos[s_id]:.4f} rad")

def main():
    rclpy.init()
    node = ServoCalibrator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()