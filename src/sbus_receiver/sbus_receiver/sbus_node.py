import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import sys
import glob
import threading
import time

class SbusReceiverNode(Node):
    def __init__(self):
        super().__init__('sbus_node')
        
        self.publisher_ = self.create_publisher(Joy, '/sbus/joy', 10)

        # ----- SBUS protocol constants -----
        self.START_BYTE = 0x0F
        self.END_BYTES = [0x00, 0x04, 0x14, 0x24, 0x34]
        self.FRAME_LEN = 25
        self.BAUDRATE = 115200

        self.SBUS_MIN = 173.0
        self.SBUS_CENTER = 992.0
        self.SBUS_MAX = 1811.0
        self.DEADZONE = 0.05

        # ----- connection state -----
        self.ser = None
        self.connected = False
        self.lock = threading.Lock()
        self.buffer = bytearray()

        # try initial connection
        self._connect()

        # timer polls serial data
        self.timer = self.create_timer(0.01, self.read_sbus_buffer)

        # watchdog for reconnection
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self.get_logger().info("SBUS node started with auto‑reconnection")

    # ----------------------------------------------------------------------
    def _connect(self):
        """Scan /dev/sbus then /dev/ttyUSB* for a device that speaks SBUS."""
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self.connected = False

            candidate_ports = (['/dev/sbus'] + sorted(glob.glob('/dev/ttyUSB*'))
                               if not glob.glob('/dev/sbus')
                               else ['/dev/sbus'])
            for port in candidate_ports:
                try:
                    self.get_logger().info(f"Trying SBUS on {port} ...")
                    s = serial.Serial(port, self.BAUDRATE, timeout=0.02)
                    s.reset_input_buffer()
                    time.sleep(0.08)  # slightly longer settle
                    raw = s.read(200)  # more data for stronger validation
                    if len(raw) < self.FRAME_LEN * 2:  # need at least 2 frames
                        s.close()
                        continue
                    # Require multiple valid SBUS frames (not just one lucky byte pair)
                    valid_frames = 0
                    for i in range(len(raw) - self.FRAME_LEN + 1):
                        if raw[i] == self.START_BYTE and raw[i + 24] in self.END_BYTES:
                            valid_frames += 1
                            i += self.FRAME_LEN  # skip past this frame
                    if valid_frames < 2:  # need ≥2 frames to avoid IMU false-positive
                        s.close()
                        continue
                    self.ser = s
                    self.buffer = bytearray()
                    self.connected = True
                    self.get_logger().info(f"SBUS receiver found on {port}")
                    return True
                except Exception as e:
                    self.get_logger().debug(f"Failed on {port}: {e}")
                    continue
            self.get_logger().warn("No SBUS receiver found")
            return False

    # ----------------------------------------------------------------------
    def _watchdog_loop(self):
        """If connection drops, try to reconnect."""
        while rclpy.ok():
            with self.lock:
                alive = self.connected
            if not alive:
                self.get_logger().info("Watchdog: SBUS disconnected, reconnecting...")
                self._connect()
            time.sleep(1.0)

    # ----------------------------------------------------------------------
    def normalize_axis(self, raw_val):
        if raw_val < self.SBUS_CENTER:
            val = (raw_val - self.SBUS_CENTER) / (self.SBUS_CENTER - self.SBUS_MIN)
        else:
            val = (raw_val - self.SBUS_CENTER) / (self.SBUS_MAX - self.SBUS_CENTER)
        val = max(-1.0, min(1.0, val))
        if abs(val) < self.DEADZONE:
            return 0.0
        return float(val)

    def parse_3pos_switch(self, raw_val):
        if raw_val < 600:
            return -1
        elif raw_val > 1400:
            return 1
        else:
            return 0

    # ----------------------------------------------------------------------
    def read_sbus_buffer(self):
        with self.lock:
            ser = self.ser
            if not self.connected or ser is None:
                return

        try:
            if ser.in_waiting > 0:
                self.buffer.extend(ser.read(ser.in_waiting))

            while len(self.buffer) >= self.FRAME_LEN:
                if self.buffer[0] != self.START_BYTE:
                    self.buffer.pop(0)
                    continue
                if self.buffer[24] not in self.END_BYTES and (self.buffer[24] & 0x0F) != 0x00:
                    self.buffer.pop(0)
                    continue

                frame = self.buffer[:self.FRAME_LEN]
                del self.buffer[:self.FRAME_LEN]
                self.parse_and_publish_frame(frame)

        except serial.SerialException as e:
            self.get_logger().warn(f"Serial error: {e}")
            with self.lock:
                self.connected = False
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def parse_and_publish_frame(self, frame):
        ch = [0] * 16
        for i in range(10):
            idx = 1 + (i * 2)
            ch[i] = (frame[idx] << 8) | frame[idx+1]

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sbus_remote"

        msg.axes = [
            self.normalize_axis(ch[0]),  # CH1
            self.normalize_axis(ch[1]),  # CH2
            self.normalize_axis(ch[2]),  # CH3
            self.normalize_axis(ch[3]),  # CH4
            self.normalize_axis(ch[8]),  # CH9
            self.normalize_axis(ch[9]),  # CH10
        ]
        msg.buttons = [
            self.parse_3pos_switch(ch[4]),  # SWA
            self.parse_3pos_switch(ch[5]),  # SWB
            self.parse_3pos_switch(ch[6]),  # SWC
            self.parse_3pos_switch(ch[7]),  # SWD
        ]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SbusReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with node.lock:
            if node.ser and node.ser.is_open:
                node.ser.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()