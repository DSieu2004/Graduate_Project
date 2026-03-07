import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy

import json
import math
import serial
import threading
import time
import numpy as np


# ------------------ UTILS ------------------
def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


# ------------------ NODE ------------------
class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        # ---------- QoS ----------
        qos_cmd_vel = QoSProfile(depth=10)
        qos_cmd_vel.reliability = ReliabilityPolicy.BEST_EFFORT

        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE

        # ---------- SUBSCRIBERS ----------
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_callback,
            qos_cmd_vel
        )

        self.sub_opentcs = self.create_subscription(
            String,
            '/opentcs/vehicle_command',
            self.opentcs_callback,
            qos_reliable
        )

        # ---------- PUBLISHERS ----------
        self.uart_pub = self.create_publisher(String, 'uart_cmd', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- ROBOT CONFIG ----------
        self.WHEEL_BASE = 0.30
        self.WHEEL_RADIUS = 0.0325
        # Không cần TICKS_PER_REV nữa vì ta dùng vận tốc trực tiếp

        # ---------- ODOM STATE ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.w = 0.0

        self.last_time = self.get_clock().now()
        self.lock = threading.Lock()

        # ---------- CONTROL ----------
        self.manual_mode = False
        self.last_manual_time = 0.0
        self.target_x = None
        self.target_y = None

        # ---------- UART ----------
        self.declare_parameter('serial_port', '/dev/serial0')
        port = self.get_parameter('serial_port').value

        try:
            self.ser = serial.Serial(port, 115200, timeout=0.05)
            self.get_logger().info(f"UART connected: {port}")
        except Exception as e:
            self.get_logger().error(f"UART error: {e}")
            self.ser = None

        threading.Thread(target=self.read_uart, daemon=True).start()

        # ---------- TIMER ----------
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("DiffDriveController READY (Velocity-based Odom)")

    # ------------------ CALLBACKS ------------------
    def vel_callback(self, msg: Twist):
        self.manual_mode = True
        self.last_manual_time = time.time()
        self.target_x = None

        v = msg.linear.x
        w = msg.angular.z

        v_l = v - w * self.WHEEL_BASE / 2.0
        v_r = v + w * self.WHEEL_BASE / 2.0

        self.send_cmd(v_r, v_l)

    def opentcs_callback(self, msg: String):
        if self.manual_mode:
            return

        try:
            data = json.loads(msg.data)
            state = data.get('state')

            if state == "START":
                with self.lock:
                    self.x = data['x']
                    self.y = data['y']
                    self.theta = 0.0
                self.target_x = None

            elif state == "PROCESSING":
                self.target_x = data['x']
                self.target_y = data['y']

            elif state == "END":
                self.target_x = None
                self.send_cmd(0.0, 0.0)

        except Exception as e:
            self.get_logger().warn(f"OpenTCS parse error: {e}")

    # ------------------ MAIN LOOP ------------------
    def control_loop(self):
        self.publish_tf()

        if self.manual_mode:
            if time.time() - self.last_manual_time > 1.0:
                self.send_cmd(0.0, 0.0)
                self.manual_mode = False
            return

        if self.target_x is None:
            return

        with self.lock:
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            theta = self.theta

        dist = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        err = self.normalize_angle(angle - theta)

        v = max(-0.4, min(0.4, 0.5 * dist))
        w = max(-1.0, min(1.0, 1.2 * err))

        if dist < 0.05:
            v = 0.0
            w = 0.0

        v_l = v - w * self.WHEEL_BASE / 2.0
        v_r = v + w * self.WHEEL_BASE / 2.0

        self.send_cmd(v_r, v_l)

    # ------------------ UART ------------------
    def send_cmd(self, v_r, v_l):
        cmd = f"CMD,{v_r:.3f},{v_l:.3f}\n"
        if self.ser:
            try:
                self.ser.write(cmd.encode())
            except Exception as e:
                self.get_logger().error(f"UART write error: {e}")

        self.uart_pub.publish(String(data=cmd.strip()))

    def read_uart(self):
        buffer = ""
        while rclpy.ok():
            if self.ser and self.ser.in_waiting:
                c = self.ser.read().decode(errors='ignore')
                if c == '\n':
                    self.parse_feedback(buffer.strip())
                    buffer = ""
                else:
                    buffer += c
            else:
                time.sleep(0.001)

    def parse_feedback(self, line):
        # STM32 gửi: FB,vel_R,vel_L
        if not line.startswith("FB"):
            return
        try:
            parts = line.split(',')
            if len(parts) >= 3:
                v_r = float(parts[1])
                v_l = float(parts[2])
                self.update_odometry(v_r, v_l)
        except Exception as e:
            pass

    # ------------------ ODOM (VELOCITY BASED) ------------------
    def update_odometry(self, v_r, v_l):

        with self.lock:
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now

            if dt <= 0:
                return

            # Tính vận tốc tịnh tiến (v) và vận tốc góc (w) của tâm robot
            self.v = (v_r + v_l) / 2.0
            self.w = (v_r - v_l) / self.WHEEL_BASE

            # Tính quãng đường di chuyển trong khoảng thời gian dt (s = v * t)
            ds = self.v * dt
            dtheta = self.w * dt

            # Cập nhật tọa độ x, y, theta trên bản đồ
            self.x += ds * math.cos(self.theta + dtheta/2.0)
            self.y += ds * math.sin(self.theta + dtheta/2.0)
            self.theta = self.normalize_angle(self.theta + dtheta)

    # ------------------ TF + ODOM ------------------
    def publish_tf(self):

        with self.lock:
            x = self.x
            y = self.y
            theta = self.theta
            v = self.v
            w = self.w

        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


# ------------------ MAIN ------------------
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()