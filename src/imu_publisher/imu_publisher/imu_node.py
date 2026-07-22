import math
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

        self.imu = None
        self.imu_connected = False
        self.lock = threading.Lock()
        # Ports that failed quat/protocol checks (often SBUS CH340 aliased as ybimu).
        self._blacklist_until = {}
        self._reconnect_sec = 5.0

        self._connect()
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
        self.get_logger().info("IMU node started with auto-reconnection")

    def _candidate_ports(self):
        ports = ['/dev/ybimu'] + sorted(glob.glob('/dev/ttyUSB*'))
        # De-duplicate symlinks (ybimu -> ttyUSB0).
        resolved = []
        seen = set()
        for port in ports:
            try:
                key = os_realpath(port)
            except Exception:
                key = port
            if key in seen:
                continue
            seen.add(key)
            resolved.append(port)
        return resolved

    def _connect(self):
        """Scan candidate ports and try to open the IMU. Returns True on success."""
        with self.lock:
            if self.imu is not None:
                try:
                    self.imu.stop_receive_thread()
                except Exception:
                    pass
                self.imu = None
            self.imu_connected = False

            now = time.monotonic()
            for port in self._candidate_ports():
                until = self._blacklist_until.get(port, 0.0)
                if now < until:
                    continue
                try:
                    self.get_logger().info(f"Trying IMU on {port} ...")
                    imu = YbImuSerial(port, debug=False)
                    imu.create_receive_threading()
                    time.sleep(0.25)
                    quat = imu.get_imu_quaternion_data()
                    if quat is None or len(quat) < 4:
                        self.get_logger().warn(f"No quaternion from {port}, not an IMU")
                        imu.stop_receive_thread()
                        self._blacklist_until[port] = now + 60.0
                        continue
                    norm = math.sqrt(sum(x * x for x in quat[:4]))
                    if abs(norm - 1.0) > 0.1:
                        self.get_logger().warn(
                            f"Bad quaternion norm {norm:.2f} on {port}, not an IMU "
                            f"(blacklisted 60s — often SBUS on same CH340)"
                        )
                        imu.stop_receive_thread()
                        self._blacklist_until[port] = now + 60.0
                        continue

                    self.imu = imu
                    self.imu_connected = True
                    self.get_logger().info(f"IMU connected on {port}")
                    return True
                except Exception as e:
                    self.get_logger().debug(f"Failed on {port}: {e}")
                    self._blacklist_until[port] = now + 10.0
                    continue
            self.get_logger().warn(
                "No IMU found (will retry). If only SBUS is plugged in, "
                "LQR cannot work until the real IMU is connected."
            )
            return False

    def _watchdog_loop(self):
        while rclpy.ok():
            with self.lock:
                connected = self.imu_connected
            if not connected:
                self.get_logger().info(
                    "Watchdog: IMU disconnected, reconnecting...",
                    throttle_duration_sec=10.0,
                )
                self._connect()
            time.sleep(self._reconnect_sec)

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
            with self.lock:
                self.imu_connected = False
                try:
                    self.imu.stop_receive_thread()
                except Exception:
                    pass
                self.imu = None


def os_realpath(path):
    import os
    return os.path.realpath(path)


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
