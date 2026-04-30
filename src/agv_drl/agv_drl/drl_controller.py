# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import math
# import os
# import torch

# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Path
# from sensor_msgs.msg import LaserScan
# from tf2_ros import Buffer, TransformListener
# from ament_index_python.packages import get_package_share_directory
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# # Import cấu trúc mạng PPO từ drl_agent.py cùng thư mục
# from agv_drl.drl_agent import PPOActorCritic

# MAX_LIDAR_RANGE = 3.5
# LOOKAHEAD_DISTANCE = 0.60 

# def normalize_angle(angle):
#     """Đưa góc về khoảng [-pi, pi]"""
#     return math.atan2(math.sin(angle), math.cos(angle))

# class DRLRealController(Node):
#     def __init__(self):
#         super().__init__('drl_controller_node')

#         # --- 1. LOAD MODEL AI ---
#         self.device = torch.device("cpu")
#         self.policy = PPOActorCritic(action_dim=2).to(self.device)

#         try:
#             package_share = get_package_share_directory('agv_drl')
#             model_path = os.path.join(package_share, "models", "ppo_nav_best.pth")
#             self.policy.load_state_dict(torch.load(model_path, map_location=self.device))
#             self.policy.eval()
#             self.get_logger().info(f"✅ NÃO BỘ ĐÃ KHỚP VÀ SẴN SÀNG: {model_path}")
#         except Exception as e:
#             self.get_logger().error(f"❌ LỖI KHỚP NỐI MODEL: {e}")
#             raise SystemExit

#         # --- 2. ROS INTERFACE ---
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )
        
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile)
#         self.plan_sub = self.create_subscription(Path, '/plan', self.plan_cb, 10) 

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Chạy 10Hz (0.1 giây mỗi chu kỳ)
#         self.timer = self.create_timer(0.1, self.control_loop)

#         # Biến trạng thái
#         self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0
#         self.laser_data = np.ones(360) * MAX_LIDAR_RANGE
#         self.global_path = []

#     def scan_cb(self, msg):
#         data = np.array(msg.ranges)
#         data[np.isnan(data) | np.isinf(data) | (data < 0.05)] = MAX_LIDAR_RANGE
#         self.laser_data = np.clip(data, 0.0, MAX_LIDAR_RANGE)

#     def plan_cb(self, msg):
#         if len(msg.poses) > 0:
#             self.global_path = msg.poses

#     def update_robot_pose(self):
#         """Cập nhật vị trí robot từ TF Tree (map -> base_link)"""
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
#             self.robot_x = trans.transform.translation.x
#             self.robot_y = trans.transform.translation.y
#             q = trans.transform.rotation
#             self.robot_yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
#             return True
#         except Exception:
#             return False

#     def control_loop(self):
#         if not self.global_path or not self.update_robot_pose():
#             return

#         # 1. Thông tin đích cuối cùng (Final Goal)
#         final_pose = self.global_path[-1].pose
#         target_x = final_pose.position.x
#         target_y = final_pose.position.y
        
#         dx = target_x - self.robot_x
#         dy = target_y - self.robot_y
#         dist_to_goal = math.hypot(dx, dy)
        
#         # Góc nhìn thẳng vào tâm điểm đích
#         angle_to_goal = normalize_angle(math.atan2(dy, dx) - self.robot_yaw)

#         # =========================================================================
#         # CHIẾN THUẬT 3 GIAI ĐOẠN (Hybrid Logic)
#         # =========================================================================

#         # GIAI ĐOẠN 1: TIẾP CẬN CHÍNH XÁC (Vùng 0.8m quanh đích)
#         if dist_to_goal < 0.8:
#             # A. XOAY HƯỚNG TIẾP CẬN (Nếu lệch góc > 15 độ)
#             if abs(angle_to_goal) > 0.26 and dist_to_goal > 0.15:
#                 self.get_logger().info(f"🔄 Xoay hướng tiếp cận: {math.degrees(angle_to_goal):.1f}°")
#                 w_cmd = np.clip(0.8 * angle_to_goal, -0.4, 0.4)
#                 self.publish_cmd(0.0, float(w_cmd))
#                 return

#             # B. TIẾN/LÙI VÀO VỊ TRÍ (Sai số 5cm)
#             if dist_to_goal > 0.05:
#                 # direction = 1.0 (Tiến) nếu đích phía trước, -1.0 (Lùi) nếu đi lố
#                 direction = 1.0 if abs(angle_to_goal) < math.pi/2 else -1.0
                
#                 v_raw = np.clip(0.6 * dist_to_goal, 0.06, 0.22)
#                 v_cmd = v_raw * direction
                
#                 # Chỉ chỉnh hướng nhẹ khi TIẾN để bám trục
#                 w_cmd = np.clip(0.4 * angle_to_goal, -0.15, 0.15) if direction > 0 else 0.0
                
#                 status = "TIẾN" if direction > 0 else "LÙI (Sửa lố)"
#                 self.get_logger().info(f"🚀 {status}... Còn: {dist_to_goal:.2f}m")
#                 self.publish_cmd(float(v_cmd), float(w_cmd))
#                 return
            
#             # C. XOAY KHỚP HEADING CUỐI (Khớp mũi tên trên RViz)
#             else:
#                 q = final_pose.orientation
#                 target_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
#                 final_angle_error = normalize_angle(target_yaw - self.robot_yaw)
                
#                 if abs(final_angle_error) > 0.08: # Sai số ~5 độ
#                     self.get_logger().info(f"📐 Chỉnh Heading cuối: {math.degrees(final_angle_error):.1f}°")
#                     w_final = np.clip(0.6 * final_angle_error, -0.4, 0.4)
#                     self.publish_cmd(0.0, float(w_final))
#                     return
#                 else:
#                     self.get_logger().info("🎯 HOÀN THÀNH: Đã đậu xe chuẩn xác!")
#                     self.global_path = []
#                     self.publish_cmd(0.0, 0.0)
#                     return

#         # GIAI ĐOẠN 2: CHẠY AI DRL (Khi ở xa > 0.8m)
#         # Chạy tốc độ cao 0.25 m/s và né vật cản linh hoạt
#         carrot_x, carrot_y = self.get_lookahead_point()
#         lidar = self.laser_data[::15][:24] / MAX_LIDAR_RANGE
#         d_c = math.hypot(carrot_x - self.robot_x, carrot_y - self.robot_y)
#         a_c = normalize_angle(math.atan2(carrot_y - self.robot_y, carrot_x - self.robot_x) - self.robot_yaw)
        
#         state_arr = np.concatenate([lidar, [np.clip(d_c/5.0, 0, 1), a_c/math.pi]]).astype(np.float32)
#         state_tensor = torch.FloatTensor(state_arr).unsqueeze(0).to(self.device)

#         with torch.no_grad():
#             mean_action, _ = self.policy(state_tensor)
#             action = torch.tanh(mean_action).cpu().numpy()[0]

#         v_ai = (action[0] + 1.0) / 2.0 * 0.25
#         w_ai = action[1] * 0.6 # Giữ w nhỏ để đi nhanh ổn định hơn
        
#         # Phanh khẩn cấp nếu vật cản quá sát
#         if np.min(self.laser_data) < 0.25:
#             v_ai = min(v_ai, 0.05)

#         self.publish_cmd(v_ai, w_ai)

#     def get_lookahead_point(self):
#         """Tìm điểm mục tiêu tạm thời trên đường xanh Nav2"""
#         min_dist = float('inf')
#         closest_idx = 0
#         for i, pose in enumerate(self.global_path):
#             dist = math.hypot(pose.pose.position.x - self.robot_x, pose.pose.position.y - self.robot_y)
#             if dist < min_dist:
#                 min_dist, closest_idx = dist, i

#         t_x, t_y = self.global_path[-1].pose.position.x, self.global_path[-1].pose.position.y
#         for i in range(closest_idx, len(self.global_path)):
#             d = math.hypot(self.global_path[i].pose.position.x - self.robot_x, self.global_path[i].pose.position.y - self.robot_y)
#             if d >= LOOKAHEAD_DISTANCE:
#                 t_x, t_y = self.global_path[i].pose.position.x, self.global_path[i].pose.position.y
#                 break
#         return t_x, t_y

#     def publish_cmd(self, v, w):
#         msg = Twist()
#         msg.linear.x = float(v)
#         msg.angular.z = float(w)
#         self.cmd_pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DRLRealController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.publish_cmd(0.0, 0.0)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import os
import torch
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from agv_drl.drl_agent import PPOActorCritic

MAX_LIDAR_RANGE    = 3.5
LOOKAHEAD_DISTANCE = 0.60

def normalize_angle(angle):
    """Đưa góc về khoảng [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

class DRLRealController(Node):

    def __init__(self):
        super().__init__('drl_controller_node')

        # --- 1. LOAD MODEL AI ---
        self.device = torch.device("cpu")
        self.policy = PPOActorCritic(action_dim=2).to(self.device)

        try:
            package_share = get_package_share_directory('agv_drl')
            model_path    = os.path.join(package_share, "models", "ppo_nav_best.pth")
            self.policy.load_state_dict(torch.load(model_path, map_location=self.device))
            self.policy.eval()
            self.get_logger().info(f"✅ NÃO BỘ ĐÃ KHỚP VÀ SẴN SÀNG: {model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ LỖI KHỚP NỐI MODEL: {e}")
            raise SystemExit

        # --- 2. ROS INTERFACE ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub  = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile)
        self.plan_sub  = self.create_subscription(Path, '/plan', self.plan_cb, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # --- 3. BIẾN TRẠNG THÁI ---
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.global_path = []

        # [SỬA HIGH]: Thêm threading.Lock() cho laser_data
        # scan_cb (luồng callback) và control_loop (luồng timer) chạy đồng thời
        # cùng đọc/ghi self.laser_data mà không có lock → race condition
        # → dữ liệu LiDAR bị đọc dở giữa chừng → AI nhận input sai → hành vi bất thường
        self.laser_lock = threading.Lock()
        self.laser_data = np.ones(360) * MAX_LIDAR_RANGE

    # ------------------ CALLBACKS ------------------

    def scan_cb(self, msg):
        data = np.array(msg.ranges)
        data[np.isnan(data) | np.isinf(data) | (data < 0.05)] = MAX_LIDAR_RANGE
        data = np.clip(data, 0.0, MAX_LIDAR_RANGE)

        # [SỬA HIGH]: Ghi laser_data trong lock
        with self.laser_lock:
            self.laser_data = data

    def plan_cb(self, msg):
        if len(msg.poses) > 0:
            self.global_path = msg.poses

    def update_robot_pose(self):
        """Cập nhật vị trí robot từ TF Tree (map → base_link)"""
        try:
            trans          = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_x   = trans.transform.translation.x
            self.robot_y   = trans.transform.translation.y
            q              = trans.transform.rotation
            self.robot_yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )
            return True
        except Exception:
            return False

    # ------------------ VÒNG LẶP ĐIỀU KHIỂN ------------------

    def control_loop(self):
        if not self.global_path or not self.update_robot_pose():
            return

        # Thông tin đích cuối cùng
        final_pose   = self.global_path[-1].pose
        target_x     = final_pose.position.x
        target_y     = final_pose.position.y

        dx           = target_x - self.robot_x
        dy           = target_y - self.robot_y
        dist_to_goal = math.hypot(dx, dy)
        angle_to_goal = normalize_angle(math.atan2(dy, dx) - self.robot_yaw)

        # =========================================================
        # CHIẾN THUẬT 3 GIAI ĐOẠN (Hybrid Logic)
        # =========================================================

        # GIAI ĐOẠN 1: TIẾP CẬN CHÍNH XÁC (vùng 0.8m quanh đích)
        if dist_to_goal < 0.8:

            # A. XOAY HƯỚNG TIẾP CẬN (nếu lệch góc > 15 độ)
            if abs(angle_to_goal) > 0.26 and dist_to_goal > 0.15:
                self.get_logger().info(f"🔄 Xoay hướng tiếp cận: {math.degrees(angle_to_goal):.1f}°")
                w_cmd = np.clip(0.8 * angle_to_goal, -0.4, 0.4)
                self.publish_cmd(0.0, float(w_cmd))
                return

            # B. TIẾN/LÙI VÀO VỊ TRÍ (sai số 5cm)
            if dist_to_goal > 0.05:
                direction = 1.0 if abs(angle_to_goal) < math.pi / 2 else -1.0
                v_raw     = np.clip(0.6 * dist_to_goal, 0.06, 0.22)
                v_cmd     = v_raw * direction
                w_cmd     = np.clip(0.4 * angle_to_goal, -0.15, 0.15) if direction > 0 else 0.0

                status = "TIẾN" if direction > 0 else "LÙI (Sửa lố)"
                self.get_logger().info(f"🚀 {status}... Còn: {dist_to_goal:.2f}m")
                self.publish_cmd(float(v_cmd), float(w_cmd))
                return

            # C. XOAY KHỚP HEADING CUỐI
            else:
                q   = final_pose.orientation
                target_yaw          = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                final_angle_error = normalize_angle(target_yaw - self.robot_yaw)

                if abs(final_angle_error) > 0.08:  # ~5 độ
                    self.get_logger().info(f"📐 Chỉnh Heading cuối: {math.degrees(final_angle_error):.1f}°")
                    w_final = np.clip(0.6 * final_angle_error, -0.4, 0.4)
                    self.publish_cmd(0.0, float(w_final))
                    return
                else:
                    self.get_logger().info("🎯 HOÀN THÀNH: Đã đậu xe chuẩn xác!")
                    self.global_path = []
                    self.publish_cmd(0.0, 0.0)
                    return

        # GIAI ĐOẠN 2: CHẠY AI DRL (khi ở xa > 0.8m)
        carrot_x, carrot_y = self.get_lookahead_point()

        # [SỬA HIGH]: Đọc laser_data trong lock để tránh race condition
        with self.laser_lock:
            laser_snapshot = self.laser_data.copy()

        lidar = laser_snapshot[::15][:24] / MAX_LIDAR_RANGE
        d_c   = math.hypot(carrot_x - self.robot_x, carrot_y - self.robot_y)
        a_c   = normalize_angle(
            math.atan2(carrot_y - self.robot_y, carrot_x - self.robot_x) - self.robot_yaw
        )

        state_arr    = np.concatenate([
            lidar,
            [np.clip(d_c / 5.0, 0, 1), a_c / math.pi]
        ]).astype(np.float32)
        state_tensor = torch.FloatTensor(state_arr).unsqueeze(0).to(self.device)

        with torch.no_grad():
            mean_action, _ = self.policy(state_tensor)
            action = torch.tanh(mean_action).cpu().numpy()[0]

        v_ai = (action[0] + 1.0) / 2.0 * 0.20  # ∈ [0, 0.25] — AI chỉ đi tiến
        w_ai = action[1] * 0.3

        # Phanh khẩn cấp nếu vật cản quá sát
        if np.min(laser_snapshot) < 0.25:
            v_ai = min(v_ai, 0.05)

        self.publish_cmd(v_ai, w_ai)

    # ------------------ HELPERS ------------------

    def get_lookahead_point(self):
        """Tìm điểm mục tiêu tạm thời trên đường xanh Nav2"""
        min_dist    = float('inf')
        closest_idx = 0

        for i, pose in enumerate(self.global_path):
            dist = math.hypot(
                pose.pose.position.x - self.robot_x,
                pose.pose.position.y - self.robot_y
            )
            if dist < min_dist:
                min_dist, closest_idx = dist, i

        t_x = self.global_path[-1].pose.position.x
        t_y = self.global_path[-1].pose.position.y

        for i in range(closest_idx, len(self.global_path)):
            d = math.hypot(
                self.global_path[i].pose.position.x - self.robot_x,
                self.global_path[i].pose.position.y - self.robot_y
            )
            if d >= LOOKAHEAD_DISTANCE:
                t_x = self.global_path[i].pose.position.x
                t_y = self.global_path[i].pose.position.y
                break

        return t_x, t_y

    def publish_cmd(self, v, w):
        msg           = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DRLRealController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_cmd(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()