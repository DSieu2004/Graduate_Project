import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import threading
import time

# Biến toàn cục để lưu danh sách các điểm đã chấm
waypoints = []

# ==============================================================
# TẠO MỘT NODE PHỤ CHỈ ĐỂ LÀM VIỆC VỚI RVIZ (TRÁNH LỖI XUNG ĐỘT)
# ==============================================================
class UIBridge(Node):
    def __init__(self):
        super().__init__('ui_bridge_node')
        # Kênh phát nét vẽ lên RViz2
        self.path_pub = self.create_publisher(Path, '/drawn_path', 10)
        # Kênh lắng nghe cú click chuột
        self.create_subscription(PointStamped, '/clicked_point', self.point_cb, 10)

    def point_cb(self, msg):
        """Hàm này tự động chạy mỗi khi bạn click 'Publish Point' trên RViz2"""
        global waypoints
        
        # Chuyển đổi PointStamped sang PoseStamped (Nav2 yêu cầu Pose)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0  # Hướng mặc định
        
        waypoints.append(pose)
        
        # Vẽ ngay lập tức đường nối các điểm lên RViz2
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = waypoints
        self.path_pub.publish(path_msg)
        
        print(f"📍 Đã chấm điểm thứ {len(waypoints)}: (x={msg.point.x:.2f}, y={msg.point.y:.2f})")

# ==============================================================
# CHƯƠNG TRÌNH CHÍNH
# ==============================================================
def main():
    global waypoints
    rclpy.init()
    
    # 1. Khởi tạo UI Node để vẽ vời
    ui_node = UIBridge()
    
    # 2. Khởi tạo Nav2 API chuyên để điều khiển xe
    nav = BasicNavigator()
    
    # CHỈ cho phép UI Node chạy ngầm để hứng sự kiện chuột, bỏ nav ra khỏi đây
    spin_thread = threading.Thread(target=rclpy.spin, args=(ui_node,), daemon=True)
    spin_thread.start()
    
    print("\n⏳ Đang kết nối với hệ thống Nav2...")
    nav.waitUntilNav2Active()
    
    # Vòng lặp chính
    while rclpy.ok():
        waypoints.clear()
        
        # Xóa đường vẽ cũ trên RViz (nếu có)
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = ui_node.get_clock().now().to_msg()
        ui_node.path_pub.publish(empty_path)
        
        print("\n" + "="*60)
        print("✍️ CHẾ ĐỘ: VẼ QUỸ ĐẠO BẰNG CHUỘT (WAYPOINT FOLLOWING)")
        print("1. Dùng công cụ 'Publish Point' trên RViz2.")
        print("2. Click liên tục nhiều điểm để tạo thành đường đi.")
        print("3. Nhấn ENTER tại Terminal này để xe bắt đầu chạy.")
        print("="*60 + "\n")
        
        # Chờ người dùng click chuột và xác nhận
        try:
            while True:
                ans = input("👉 Bấm [ENTER] để xe chạy theo quỹ đạo | Gõ [C] để xóa vẽ lại: ").strip().lower()
                
                if ans == 'c':
                    print("Đã xóa quỹ đạo. Mời bạn chấm lại từ đầu!")
                    break # Thoát vòng lặp nhỏ để reset list
                    
                elif ans == '':
                    if len(waypoints) == 0:
                        print("⚠️ Bạn chưa click điểm nào trên RViz2 cả!")
                        continue
                        
                    print(f"\n🚀 ĐÃ CHỐT! Xe bắt đầu chạy qua {len(waypoints)} điểm...")
                    
                    # Gọi API của Nav2: goThroughPoses (Bây giờ sẽ không bị văng lỗi nữa!)
                    nav.goThroughPoses(waypoints)
                    
                    # Chờ xe chạy xong
                    while not nav.isTaskComplete():
                        # Lấy feedback để hiển thị tiến độ (Tùy chọn)
                        feedback = nav.getFeedback()
                        if feedback:
                            print(f"Khoảng cách còn lại: {feedback.distance_remaining:.2f} m", end="\r")
                        time.sleep(0.5)
                        
                    # Kết quả
                    result = nav.getResult()
                    if result == TaskResult.SUCCEEDED:
                        print("\n🎉 XUẤT SẮC! Đã hoàn thành toàn bộ quỹ đạo!")
                    else:
                        print("\n⚠️ CẢNH BÁO: Bị lỗi giữa chừng (Kẹt vật cản hoặc bị hủy).")
                        
                    break # Chạy xong, thoát để bắt đầu vòng vẽ mới
                    
        except KeyboardInterrupt:
            print("\nĐang thoát chương trình...")
            break
            
    # Dọn dẹp cả 2 node khi thoát
    nav.destroy_node()
    ui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()