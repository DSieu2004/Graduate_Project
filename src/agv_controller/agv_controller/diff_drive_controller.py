# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from rclpy.qos import QoSProfile, ReliabilityPolicy

# import math
# import serial
# import threading
# import time
# import numpy as np

# # ------------------ UTILS ------------------
# def quaternion_from_euler(roll, pitch, yaw):
#     """Chuyển đổi Euler (Radian) sang Quaternion chuẩn ROS"""
#     qz = np.sin(yaw / 2)
#     qw = np.cos(yaw / 2)
#     return [0.0, 0.0, qz, qw]

# # ------------------ NODE ------------------
# class DiffDriveController(Node):

#     def __init__(self):
#         super().__init__('diff_drive_controller')

#         # ---------- QoS ----------
#         qos_reliable = QoSProfile(depth=10)
#         qos_reliable.reliability = ReliabilityPolicy.RELIABLE

#         # ---------- SUBSCRIBERS ----------
#         # Lắng nghe lệnh vận tốc từ Nav2 hoặc Teleop
#         self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
#         self.sub_opentcs = self.create_subscription(String, '/opentcs/vehicle_command', self.opentcs_callback, qos_reliable)#
        
#         # Lắng nghe lệnh nạp PID từ Web GUI
#         self.sub_pid = self.create_subscription(String, '/pid_command', self.pid_callback, 10)

#         # ---------- PUBLISHERS ----------
#         # Phát dữ liệu thô cho EKF xử lý
#         self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
#         self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

#         # ---------- ROBOT CONFIG ----------
#         self.WHEEL_BASE = 0.30 # Khoảng cách 2 bánh (m) - Lưu ý đo lại thực tế xem có chính xác 30cm không nhé!

#         # ---------- STATE ----------
#         self.x = 0.0
#         self.y = 0.0
#         self.theta_imu = 0.0
#         self.v = 0.0
#         self.w = 0.0

#         self.last_time = self.get_clock().now()
#         self.lock = threading.Lock()

#         # ---------- UART SETUP ----------
#         self.declare_parameter('serial_port', '/dev/serial0')
#         port = self.get_parameter('serial_port').value

#         try:
#             self.ser = serial.Serial(port, 115200, timeout=0.1)
#             self.get_logger().info(f"🚀 UART Connected to STM32 at {port}")
#         except Exception as e:
#             self.get_logger().error(f"❌ UART Error: {e}")
#             self.ser = None

#         # Chạy luồng đọc UART riêng biệt để không gây trễ hệ thống
#         threading.Thread(target=self.read_uart, daemon=True).start()

#         # ---------- TIMER ----------
#         # Phát topic /odom định kỳ 20Hz
#         self.create_timer(0.05, self.control_loop) 

#     # ------------------ CALLBACKS ------------------
#     def vel_callback(self, msg: Twist):
#         """Nhận v, w từ ROS và gửi lệnh xuống STM32"""
#         v = msg.linear.x
#         w = msg.angular.z
#         v_l = v - w * self.WHEEL_BASE / 2.0
#         v_r = v + w * self.WHEEL_BASE / 2.0
#         self.send_cmd(v_r, v_l)

#     def opentcs_callback(self, msg: String):
#         # Giữ chỗ cho logic điều khiển từ OpenTCS nếu cần
#         pass

#     def pid_callback(self, msg: String):
#         """Nhận Kp, Ki từ Web và gửi xuống STM32"""
#         if self.ser and self.ser.is_open:
#             try:
#                 # Đảm bảo có ký tự \n ở cuối để STM32 dùng readline() hoặc đọc hết chuỗi
#                 cmd = msg.data
#                 if not cmd.endswith('\n'):
#                     cmd += '\n'
                    
#                 self.ser.write(cmd.encode())
#                 self.get_logger().info(f"⚙️ Nạp PID mới xuống STM32: {cmd.strip()}")
#             except Exception as e:
#                 self.get_logger().error(f"❌ Lỗi gửi PID: {e}")

#     # ------------------ UART COMMUNICATION ------------------
#     def send_cmd(self, v_r, v_l):
#         """Gửi chuỗi CMD xuống STM32"""
#         cmd = f"CMD,{v_r:.3f},{v_l:.3f}\n"
#         if self.ser and self.ser.is_open:
#             try:
#                 self.ser.write(cmd.encode())
#             except Exception as e:
#                 self.get_logger().error(f"UART Write Error: {e}")

#     def read_uart(self):
#         """Luồng đọc dữ liệu phản hồi (Feedback) từ STM32"""
#         while rclpy.ok():
#             if self.ser and self.ser.is_open:
#                 if self.ser.in_waiting > 0:
#                     try:
#                         # Đọc nguyên dòng để đảm bảo tính toàn vẹn dữ liệu
#                         line = self.ser.readline().decode('utf-8', errors='ignore').strip()
#                         if line.startswith("FB,"):
#                             self.parse_feedback(line)
#                     except Exception as e:
#                         self.get_logger().warn(f"UART Read Error: {e}")
#                 else:
#                     time.sleep(0.002) # Giảm tải CPU khi không có dữ liệu
#             else:
#                 time.sleep(0.5)

#     def parse_feedback(self, line):
#         """Phân tích chuỗi FB,vR,vL,theta và cập nhật tọa độ"""
#         try:
#             parts = line.split(',')
#             if len(parts) >= 4:
#                 v_r = float(parts[1])
#                 v_l = float(parts[2])
#                 theta_now = float(parts[3]) # Góc Yaw từ BNO055 (Radian)

#                 with self.lock:
#                     now = self.get_clock().now()
#                     dt = (now - self.last_time).nanoseconds / 1e9
#                     self.last_time = now

#                     # Kiểm tra dt hợp lệ để tránh nhảy tọa độ khi lag
#                     if 0.0 < dt < 1.0:
#                         theta_old = self.theta_imu     # Lưu lại góc cũ
#                         self.theta_imu = theta_now     # Cập nhật góc mới từ IMU
                        
#                         self.v = (v_r + v_l) / 2.0
#                         self.w = (v_r - v_l) / self.WHEEL_BASE 

#                         # =========================================================
#                         # ODOMETRY FUSION (Nâng cấp Runge-Kutta bậc 2)
#                         # =========================================================
#                         ds = self.v * dt
                        
#                         # Tính góc trung bình của quỹ đạo cong để nội suy chuẩn hơn
#                         theta_mid = theta_old + (self.w * dt) / 2.0
                        
#                         # Cập nhật tọa độ X, Y dựa trên góc trung bình
#                         self.x += ds * math.cos(theta_mid)
#                         self.y += ds * math.sin(theta_mid)
                        
#                         # Phát IMU ngay lập tức để đồng bộ thời gian với Encoder
#                         self.publish_imu(theta_now)
#         except Exception as e:
#             pass

#     # ------------------ PUBLISHERS ------------------
#     def publish_imu(self, theta):
#         """Phát topic /imu/data cho EKF"""
#         msg = Imu()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "imu_link"

#         q = quaternion_from_euler(0, 0, theta)
#         msg.orientation.z = q[2]
#         msg.orientation.w = q[3]

#         # Ma trận sai số cực thấp (Tin tưởng IMU tuyệt đối về hướng)
#         msg.orientation_covariance[8] = 0.001 
#         self.imu_pub.publish(msg)

#     def control_loop(self):
#         """Phát topic /odom định kỳ để RViz và Nav2 sử dụng"""
#         now = self.get_clock().now().to_msg()
#         odom = Odometry()
#         odom.header.stamp = now
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"

#         with self.lock:
#             odom.pose.pose.position.x = self.x
#             odom.pose.pose.position.y = self.y
#             q = quaternion_from_euler(0, 0, self.theta_imu)
#             odom.pose.pose.orientation.z = q[2]
#             odom.pose.pose.orientation.w = q[3]
            
#             odom.twist.twist.linear.x = self.v
#             odom.twist.twist.angular.z = self.w

#             # MA TRẬN HIỆP PHƯƠNG SAI (Giúp EKF hoạt động chính xác)
#             odom.pose.covariance[0] = 0.01  # Sai số x
#             odom.pose.covariance[7] = 0.01  # Sai số y
#             odom.pose.covariance[35] = 0.01 # Sai số yaw
            
#             odom.twist.covariance[0] = 0.01 # Sai số vx
#             odom.twist.covariance[35] = 0.01 # Sai số wz

#         self.odom_pub.publish(odom)

# # ------------------ MAIN ------------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = DiffDriveController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


import math
import serial
import threading
import time
import numpy as np

# ------------------ UTILS ------------------
def quaternion_from_euler(roll, pitch, yaw):
    """Chuyển đổi Euler (Radian) sang Quaternion chuẩn ROS"""
    qz = np.sin(yaw / 2)
    qw = np.cos(yaw / 2)
    return [0.0, 0.0, float(qz), float(qw)]

# ------------------ NODE ------------------
class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        # ---------- QoS ----------

        # ---------- SUBSCRIBERS ----------
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.sub_pid     = self.create_subscription(String, '/pid_command', self.pid_callback, 10)

        # ---------- PUBLISHERS ----------
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub  = self.create_publisher(Imu, 'imu/data', 10)

        # ---------- ROBOT CONFIG ----------
        self.WHEEL_BASE = 0.30  # Khoảng cách 2 bánh (m)

        # ---------- STATE ----------
        self.x = 0.0
        self.y = 0.0

        # SỬA #2: Tách 2 góc riêng biệt
        # - theta_enc: tích phân từ encoder → dùng để tính x, y trong odometry
        # - theta_imu: góc tuyệt đối từ BNO055 → chỉ dùng để publish /imu/data
        # Lý do: nếu dùng theta_imu để tính x,y rồi lại gửi /imu/data lên EKF
        # thì IMU bị EKF tính 2 lần → kết quả định vị sai hoàn toàn
        self.theta_enc = 0.0  # Góc tích phân từ encoder
        self.theta_imu = 0.0  # Góc tuyệt đối từ BNO055

        self.v = 0.0
        self.w = 0.0

        self.last_time = self.get_clock().now()
        self.lock = threading.Lock()

        # ---------- UART SETUP ----------
        self.declare_parameter('serial_port', '/dev/serial0')
        port = self.get_parameter('serial_port').value

        try:
            # SỬA #1: Tăng timeout từ 0.1s lên 1.0s
            # Lý do: STM32 gửi FB mỗi 100ms. Timeout 0.1s quá sát →
            # readline() thường xuyên hết giờ giữa chừng → trả về
            # chuỗi dở dang như '002\rFB' → parse_feedback bị lỗi
            self.ser = serial.Serial(port, 115200, timeout=1.0)
            self.get_logger().info(f"🚀 UART Connected to STM32 at {port}")
        except Exception as e:
            self.get_logger().error(f"❌ UART Error: {e}")
            self.ser = None

        threading.Thread(target=self.read_uart, daemon=True).start()

        # ---------- TIMER ----------
        self.create_timer(0.05, self.control_loop)  # 20Hz

    # ------------------ CALLBACKS ------------------
    def vel_callback(self, msg: Twist):
        v   = msg.linear.x
        w   = msg.angular.z
        v_l = v - w * self.WHEEL_BASE / 2.0
        v_r = v + w * self.WHEEL_BASE / 2.0
        self.send_cmd(v_r, v_l)

    def pid_callback(self, msg: String):
        if self.ser and self.ser.is_open:
            try:
                cmd = msg.data
                if not cmd.endswith('\n'):
                    cmd += '\n'
                self.ser.write(cmd.encode())
                self.get_logger().info(f"⚙️ Nạp PID mới: {cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f"❌ Lỗi gửi PID: {e}")

    # ------------------ UART COMMUNICATION ------------------
    def send_cmd(self, v_r, v_l):
        cmd = f"CMD,{v_r:.3f},{v_l:.3f}\n"
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
            except Exception as e:
                self.get_logger().error(f"UART Write Error: {e}")

    def read_uart(self):
        while rclpy.ok():
            if self.ser and self.ser.is_open:
                try:
                    # SỬA #1: Bỏ check in_waiting
                    # Lý do: in_waiting > 0 chỉ báo có ít nhất 1 byte trong buffer,
                    # không đảm bảo đủ 1 dòng hoàn chỉnh. Kết hợp timeout ngắn →
                    # readline() trả về chuỗi dở. Để readline() tự chờ đến '\n'
                    # với timeout=1.0s là cách đúng và ổn định hơn.
                    raw = self.ser.readline()
                    if not raw:
                        continue  # Timeout, không có data → đọc lại

                    line = raw.decode('utf-8', errors='ignore').strip()

                    # Kiểm tra chặt: phải bắt đầu FB, và có đủ 3 dấu phẩy
                    if line.startswith("FB,") and line.count(',') >= 3:
                        self.parse_feedback(line)

                except Exception as e:
                    self.get_logger().warn(f"UART Read Error: {e}")
            else:
                time.sleep(0.5)

    def parse_feedback(self, line):
        """Phân tích chuỗi FB,vR,vL,theta từ STM32"""
        try:
            parts     = line.split(',')
            v_r       = float(parts[1])
            v_l       = float(parts[2])
            theta_now = float(parts[3])  # Góc Yaw tuyệt đối từ BNO055 (radian)

            with self.lock:
                now = self.get_clock().now()
                dt  = (now - self.last_time).nanoseconds / 1e9
                self.last_time = now

                if not (0.0 < dt < 1.0):
                    return

                self.v = (v_r + v_l) / 2.0
                self.w = (v_r - v_l) / self.WHEEL_BASE

                # SỬA #2: Odometry thuần encoder
                # Dùng theta_enc (tích phân w từ encoder) để tính x, y
                # KHÔNG dùng theta_imu → tránh EKF tính IMU 2 lần
                ds = self.v * dt

                # Runge-Kutta bậc 2: dùng góc giữa khoảng thời gian
                theta_mid      = self.theta_enc + (self.w * dt) / 2.0
                self.theta_enc += self.w * dt  # Tích phân góc encoder

                self.x += ds * math.cos(theta_mid)
                self.y += ds * math.sin(theta_mid)

                # Lưu góc IMU riêng, chỉ dùng cho /imu/data
                self.theta_imu = theta_now

                # Publish /imu/data ngay khi có data mới
                self.publish_imu(theta_now)

        except Exception as e:
            self.get_logger().warn(f"parse_feedback error: {e}")

    # ------------------ PUBLISHERS ------------------
    def publish_imu(self, theta):
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        q = quaternion_from_euler(0.0, 0.0, theta)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        # Lý do: BNO055 chế độ IMUPLUS chỉ cho góc yaw đáng tin cậy.
        # Nếu không set -1, EKF sẽ cố dùng cả roll/pitch = 0 → fusion sai
        msg.orientation_covariance[0] = 1000.0   # Roll  → EKF bỏ qua
        msg.orientation_covariance[4] = 1000.0    #Pitch → rất không chắc
        msg.orientation_covariance[8] =  0.001  # Yaw   → tin tưởng cao


        # EKF sẽ chỉ dùng orientation yaw
        msg.angular_velocity_covariance[0] = -1.0

        self.imu_pub.publish(msg)

    def control_loop(self):
        """Phát topic /odom định kỳ 20Hz"""
        now  = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        with self.lock:
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y

            # SỬA #4: Dùng theta_enc cho pose orientation của odom
            # Lý do: x, y được tính từ encoder → orientation cũng phải
            # từ encoder để nhất quán. EKF sẽ fusion với IMU sau.
            q = quaternion_from_euler(0.0, 0.0, self.theta_enc)
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            odom.twist.twist.linear.x  = self.v
            odom.twist.twist.angular.z = self.w

            # Ma trận hiệp phương sai
            odom.pose.covariance[0]  = 0.01   # Sai số x
            odom.pose.covariance[7]  = 0.01   # Sai số y
            odom.pose.covariance[35] = 0.05   # Sai số yaw (encoder drift)

            odom.twist.covariance[0]  = 0.01  # Sai số vx
            odom.twist.covariance[35] = 0.05  # Sai số wz

        self.odom_pub.publish(odom)

# ------------------ MAIN ------------------
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()