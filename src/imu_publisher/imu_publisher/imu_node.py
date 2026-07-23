import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys
import glob
import threading
import time

try:
    from YbImuLib.YbImuLib.YbImuSerialLib import YbImuSerial
except ImportError:
    try:
        from YbImuLib.YbImuSerialLib import YbImuSerial
    except ImportError as e:
        print(f"[ERROR] Could not import YbImuSerialLib: {e}")
        sys.exit(1)

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        # ---------- connection state ----------
        self.imu = None
        self.imu_connected = False
        self.lock = threading.Lock()

        # list of ports to try (in order) – can be extended via config if needed
        self.candidate_ports = ['/dev/ybimu'] + sorted(glob.glob('/dev/ttyUSB*'))

        # try initial connection
        self._connect()

        # timer publishes only if connected
        self.timer = self.create_timer(0.02, self.timer_callback)

        # background thread that watches the connection and reconnects if needed
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self.get_logger().info("IMU node started with auto‑reconnection")

    # ----------------------------------------------------------------------
    def _connect(self):
        """Scan candidate ports and try to open the IMU. Returns True on success."""
        with self.lock:
            # clean up old connection if any
            if self.imu is not None:
                try:
                    self.imu.stop_receive_thread()
                except Exception:
                    pass
                self.imu = None
            self.imu_connected = False

            for port in self.candidate_ports:
                try:
                    self.get_logger().info(f"Trying IMU on {port} ...")
                    imu = YbImuSerial(port, debug=False)
                    # The constructor starts background thread? Check library.
                    # We start it explicitly.
                    imu.create_receive_threading()
                    # Small wait and then verify we actually get data
                    time.sleep(0.2)
                    quat = imu.get_imu_quaternion_data()
                    if quat is None or len(quat) < 4:
                        self.get_logger().warn(f"No quaternion from {port}, not an IMU")
                        imu.stop_receive_thread()
                        continue
                    # additional sanity: quaternion norm ~1
                    import math
                    norm = math.sqrt(sum(x*x for x in quat[:4]))
                    if abs(norm - 1.0) > 0.1:
                        self.get_logger().warn(f"Bad quaternion norm {norm:.2f} on {port}, not an IMU")
                        imu.stop_receive_thread()
                        continue

                    self.imu = imu
                    self.imu_connected = True
                    self.get_logger().info(f"IMU connected on {port}")
                    return True
                except Exception as e:
                    self.get_logger().debug(f"Failed on {port}: {e}")
                    continue
            self.get_logger().warn("No IMU found on any port")
            return False

    # ----------------------------------------------------------------------
    def _watchdog_loop(self):
        """Periodically check connection health; if broken, try to reconnect."""
        while rclpy.ok():
            with self.lock:
                connected = self.imu_connected
            if not connected:
                self.get_logger().info("Watchdog: IMU disconnected, reconnecting...")
                self._connect()
            time.sleep(1.0)

    # ----------------------------------------------------------------------
    def timer_callback(self):
        with self.lock:
            if not self.imu_connected or self.imu is None:
                return
            imu = self.imu

        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            accel = imu.get_accelerometer_data()
            if accel and len(accel) >= 3:
                GRAVITY = 9.80665
                msg.linear_acceleration.x = float(accel[0]) * GRAVITY
                msg.linear_acceleration.y = float(accel[1]) * GRAVITY
                msg.linear_acceleration.z = float(accel[2]) * GRAVITY

            gyro = imu.get_gyroscope_data()
            if gyro and len(gyro) >= 3:
                msg.angular_velocity.x = float(gyro[0])
                msg.angular_velocity.y = float(gyro[1])
                msg.angular_velocity.z = float(gyro[2])

            quat = imu.get_imu_quaternion_data()
            if quat and len(quat) >= 4:
                msg.orientation.w = float(quat[0])
                msg.orientation.x = float(quat[1])
                msg.orientation.y = float(quat[2])
                msg.orientation.z = float(quat[3])

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}")
            # mark disconnected so watchdog will reconnect
            with self.lock:
                self.imu_connected = False
                try:
                    self.imu.stop_receive_thread()
                except Exception:
                    pass
                self.imu = None

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()