#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import matplotlib.pyplot as plt
import math
import time

class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odom_recorder')
        self.subscription = self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.x_data = []
        self.y_data = []

    def callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

def create_pose(navigator, x, y, theta):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    recorder = OdomRecorder()

    print("Đang khởi động hệ thống...")
    nav.waitUntilNav2Active()

    # Waypoints cũ của bạn
    goal_A = create_pose(nav, -0.546, -0.512, -math.pi/2)     
    goal_B = create_pose(nav, 0.997, -0.417, 0.0)             
    goal_C = create_pose(nav, 1.010, 0.557, math.pi/2)        
    goal_D = create_pose(nav, -0.534, 0.278, math.pi)         
    waypoints = [goal_B, goal_C, goal_D, goal_A]

    print("Bắt đầu chạy 5 vòng thực nghiệm...")

    for i in range(5):
        print(f"\n>>> VÒNG {i+1}...")
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            rclpy.spin_once(recorder, timeout_sec=0.1) # Thu thập tọa độ liên tục

    print("\n[XONG] Đang chuẩn bị xuất đồ thị...")

    # --- Vẽ đồ thị chỉ có đường thực tế ---
    if len(recorder.x_data) > 0:
        plt.figure(figsize=(8, 8))
        # Chỉ vẽ đường thực tế màu xanh dương
        plt.plot(recorder.x_data, recorder.y_data, 'b-', linewidth=1.5, label='Quỹ đạo AGV thực tế')
        
        # Đánh dấu điểm bắt đầu
        plt.scatter(recorder.x_data[0], recorder.y_data[0], color='green', s=100, label='Điểm xuất phát')
        
        plt.title("QUỸ ĐẠO DI CHUYỂN CỦA AGV (DỮ LIỆU ODOMETRY)", fontsize=14, fontweight='bold')
        plt.xlabel("Tọa độ X (m)")
        plt.ylabel("Tọa độ Y (m)")
        plt.axis('equal') # Giữ tỉ lệ chuẩn, không bị méo hình
        plt.grid(True, linestyle=':', alpha=0.7)
        plt.legend()
        
        # Lưu ảnh và hiển thị
        plt.savefig('quy_dao_agv_thuc_te.png', dpi=300)
        print("Đã lưu ảnh 'quy_dao_agv_thuc_te.png'.")
        plt.show()
    else:
        print("Không nhận được dữ liệu để vẽ.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()