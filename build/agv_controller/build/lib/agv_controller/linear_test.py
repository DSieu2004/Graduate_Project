#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import matplotlib.pyplot as plt
import math
import os

# --- NODE THU THẬP DỮ LIỆU ODOM ---
class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odom_recorder_final')
        self.subscription = self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.x_data = []
        self.y_data = []

    def callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

def p(nav, x, y):
    """Hàm tạo PoseStamped"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    recorder = OdomRecorder()

    print("Đang chờ Nav2 sẵn sàng...")
    nav.waitUntilNav2Active()

    # --- TỌA ĐỘ MỤC TIÊU ---
    # Ta sẽ đo sai số tại điểm cuối cùng (P1)
    goal_x, goal_y = -0.928, -0.307
    p1 = p(nav, goal_x, goal_y)
    p2 = p(nav, 1.013, -0.337)

    print("Bắt đầu thực hiện lộ trình test sai số (P1 -> P2 -> P1)...")
    
    # Gửi lệnh chạy Waypoints
    nav.followWaypoints([p1, p2, p1])

    # --- VÒNG LẶP THEO DÕI (KHÔNG CÒN LỖI distance_remaining) ---
    while not nav.isTaskComplete():
        # Cập nhật dữ liệu từ Node OdomRecorder
        rclpy.spin_once(recorder, timeout_sec=0.1)
        
        feedback = nav.getFeedback()
        if feedback:
            # Feedback của followWaypoints chỉ có current_waypoint
            print(f"Robot đang di chuyển đến Waypoint: {feedback.current_waypoint + 1}", end='\r')

    print("\n[XONG] Robot đã dừng. Đang tính toán sai số...")

    # --- TÍNH TOÁN SAI SỐ EUCLID ---
    if len(recorder.x_data) > 0:
        actual_x = recorder.x_data[-1]
        actual_y = recorder.y_data[-1]
        
        # Tính khoảng cách giữa điểm đích mong muốn và điểm thực tế robot đứng
        error = math.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
        
        print("\n" + "="*40)
        print("          BÁO CÁO SAI SỐ ĐIỂM DỪNG")
        print(f" Tọa độ mục tiêu:  x = {goal_x:.4f}, y = {goal_y:.4f}")
        print(f" Tọa độ thực tế:  x = {actual_x:.4f}, y = {actual_y:.4f}")
        print(f" SAI SỐ TUYỆT ĐỐI: {error*100:.2f} cm")
        print("="*40)

        # --- VẼ QUỸ ĐẠO VÀ LƯU ẢNH ---
        plt.figure(figsize=(10, 6))
        # Vẽ đường đi màu xanh
        plt.plot(recorder.x_data, recorder.y_data, 'b-', linewidth=1.5, label='Quỹ đạo thực tế')
        # Đánh dấu điểm đích (X đỏ)
        plt.scatter(goal_x, goal_y, color='red', marker='x', s=150, label='Đích mong muốn (Goal)')
        # Đánh dấu vị trí dừng (Chấm xanh)
        plt.scatter(actual_x, actual_y, color='green', s=100, label='Vị trí dừng thực tế', zorder=5)

        plt.title("PHÂN TÍCH SAI SỐ ĐIỀU HƯỚNG ROBOT", fontsize=14)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis('equal')
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend()
        
        # Lưu file vào thư mục người dùng
        save_path = os.path.expanduser('~/error_test_final.png')
        plt.savefig(save_path, dpi=300)
        print(f"Đã lưu biểu đồ phân tích tại: {save_path}")
    else:
        print("Lỗi: Không thu thập được dữ liệu tọa độ.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()