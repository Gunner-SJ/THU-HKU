import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
import numpy as np
from scipy.spatial.transform import Rotation

# Import your custom bike control models (must be in your PYTHONPATH)
from nominal_bike_control import NominalBikeController, ScaleBikeModel

class BikeBalanceNode(Node):
    def __init__(self):
        super().__init__('bike_balance_controller')
        
        # --- Parameters (Extracted from Notebook) ---
        self.declare_parameter('control_method', 'lqr') # 'lqr' or 'pole'
        self.declare_parameter('control_dt', 0.02)
        self.declare_parameter('wheel_radius', 0.0725)
        self.declare_parameter('target_speed', 0.1)
        self.declare_parameter('min_scheduling_speed', 1.0)
        self.declare_parameter('balance_start_time', 0.1)
        
        # Pole & LQR params
        self.declare_parameter('pole_wc', -5.0)
        self.declare_parameter('lqr_q_steer', 4.0)
        self.declare_parameter('lqr_q_roll', 100.0)
        self.declare_parameter('lqr_q_roll_rate', 2.0)
        self.declare_parameter('lqr_r_steer_rate', 10.0)
        self.declare_parameter('max_steer_velocity', 10.0)
        
        # Inner loop params
        self.declare_parameter('steer_kp', 0.01)
        self.declare_parameter('steer_ki', 0.01)
        self.declare_parameter('steer_integral_limit', 10.0)
        self.declare_parameter('rear_kp', 0.1)
        self.declare_parameter('rear_torque_limit', 2.0)
        
        # IMU Offsets
        self.declare_parameter('sensor_to_bike_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('gyro_bias_sensor_dps', [0.0, 0.0, 0.0])

        # --- Subscriptions & Publications ---
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.motor_state_sub = self.create_subscription(JointState, '/motor/state', self.motor_state_callback, 10)
        self.motor_cmd_pub = self.create_publisher(JointState, '/motor/command', 10)

        # --- State Variables ---
        self.current_steer = 0.0
        self.current_steer_rate = 0.0
        self.current_rear_rate = 0.0
        self.imu_quat = [1.0, 0.0, 0.0, 0.0] # w, x, y, z
        self.imu_gyro = [0.0, 0.0, 0.0]
        
        self.steer_integral = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # --- Initialize Bike Model ---
        dt = self.get_parameter('control_dt').value
        self.bike = ScaleBikeModel(dt=dt, track_roll=False)
        self.controller = self.make_controller()

        # 50Hz Control Loop Timer
        self.timer = self.create_timer(dt, self.control_loop)
        self.get_logger().info(f"Balance node started using {self.get_parameter('control_method').value.upper()} control.")

    def make_controller(self):
        method = self.get_parameter('control_method').value
        max_steer_vel = self.get_parameter('max_steer_velocity').value
        common = dict(u_min=-max_steer_vel, u_max=max_steer_vel)

        if method == "pole":
            return NominalBikeController(
                self.bike.sys,
                method="place_multiple_poles",
                wc=self.get_parameter('pole_wc').value,
                **common
            )
        elif method == "lqr":
            Q = np.diag([
                self.get_parameter('lqr_q_steer').value,
                self.get_parameter('lqr_q_roll').value,
                self.get_parameter('lqr_q_roll_rate').value,
            ])
            R = np.array([[self.get_parameter('lqr_r_steer_rate').value]])
            return NominalBikeController(
                self.bike.sys,
                method="lqr",
                Qc=Q,
                Rc=R,
                **common
            )
        else:
            raise ValueError("control_method must be 'pole' or 'lqr'")

    def imu_callback(self, msg: Imu):
        # Store latest IMU data. Note: transforms assume w, x, y, z
        self.imu_quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.imu_gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def motor_state_callback(self, msg: JointState):
        try:
            steer_idx = msg.name.index('steer_motor')
            drive_idx = msg.name.index('drive_motor')
            self.current_steer = msg.position[steer_idx]
            self.current_steer_rate = msg.velocity[steer_idx]
            self.current_rear_rate = msg.velocity[drive_idx]
        except ValueError:
            pass # Joint names don't match

    def get_bicycle_state(self):
        # Construct rotation from configuration parameters
        rpy_deg = self.get_parameter('sensor_to_bike_rpy_deg').value
        roll, pitch, yaw = np.deg2rad(rpy_deg)
        sensor_to_bike = Rotation.from_euler("ZYX", [yaw, pitch, roll])
        
        # Quaternion conversion (w, x, y, z -> x, y, z, w for scipy)
        xyzw = np.r_[self.imu_quat[1:], self.imu_quat[0]]
        world_from_sensor = Rotation.from_quat(xyzw)
        world_from_bike = world_from_sensor * sensor_to_bike.inv()
        
        b_yaw, b_pitch, b_roll = world_from_bike.as_euler("ZYX")

        # Gyro bias compensation
        gyro_bias = np.deg2rad(self.get_parameter('gyro_bias_sensor_dps').value)
        p, q, r = sensor_to_bike.apply(np.asarray(self.imu_gyro) - gyro_bias)
        
        if abs(np.cos(b_pitch)) < 1e-3:
             self.get_logger().warn("Pitch near singularity!")
             
        roll_rate = p + np.sin(b_roll) * np.tan(b_pitch) * q + np.cos(b_roll) * np.tan(b_pitch) * r
        state = np.array([[self.current_steer], [b_roll], [roll_rate]])
        return state

    def control_loop(self):
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        state = self.get_bicycle_state()
        
        # 1. Update Speed and System Params
        speed = self.get_parameter('wheel_radius').value * self.current_rear_rate
        min_sched_speed = self.get_parameter('min_scheduling_speed').value
        scheduling_speed = max(abs(speed), min_sched_speed)
        
        self.bike.updateSysParam(scheduling_speed, min_forw_vel=min_sched_speed)
        self.controller.updateSysAndGain(self.bike.sys)
        
        # 2. Outer Loop: Get Reference Steer Rate
        steer_rate_ref = float(self.controller.step([[0.0]], state, current_time)[0, 0])
        
        if current_time < self.get_parameter('balance_start_time').value:
            steer_rate_ref = 0.0

        # 3. Inner Loop: Steering Torque (PI Control)
        dt = self.get_parameter('control_dt').value
        steer_error = steer_rate_ref - self.current_steer_rate
        
        int_limit = self.get_parameter('steer_integral_limit').value
        self.steer_integral = float(np.clip(
            self.steer_integral + steer_error * dt, -int_limit, int_limit
        ))
        
        steer_torque = (self.get_parameter('steer_kp').value * steer_error + 
                        self.get_parameter('steer_ki').value * self.steer_integral)

        # 4. Inner Loop: Rear Torque (P Control)
        target_speed = self.get_parameter('target_speed').value
        # Assuming instantaneous target speed based on acceleration profile isn't strictly needed for the node,
        # but maintaining target_speed is.
        rear_rate_ref = target_speed / self.get_parameter('wheel_radius').value
        rear_torque = self.get_parameter('rear_kp').value * (rear_rate_ref - self.current_rear_rate)
        
        rear_limit = self.get_parameter('rear_torque_limit').value
        rear_torque = float(np.clip(rear_torque, -rear_limit, rear_limit))

        # 5. Publish to Motor Node
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = ['drive_motor', 'steer_motor']
        cmd_msg.effort = [rear_torque, steer_torque] 
        self.motor_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BikeBalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()