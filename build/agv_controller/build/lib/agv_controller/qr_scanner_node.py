import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from pyzbar.pyzbar import decode
import time

class QRCodeScanner(Node):
    def __init__(self):
        super().__init__('qr_code_scanner')
        
        # 1. Tạo Publisher để phát dữ liệu QR lên Topic '/qr_data'
        self.publisher_ = self.create_publisher(String, '/qr_data', 10)
        
        # 2. Khởi tạo Camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Không thể mở được Camera! Kiểm tra lại cổng USB.")
        else:
            self.get_logger().info("✅ Đã mở Camera thành công! Đang quét ngầm...")

        # 3. Tạo Timer để quét ảnh liên tục (10 lần/giây ~ 10Hz)
        self.timer = self.create_timer(0.1, self.scan_callback)

        # 4. Các biến chống spam (Cooldown 3 giây)
        self.last_scanned_data = ""
        self.last_scan_time = 0.0
        self.cooldown_time = 3.0

    def scan_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        # Quét mã QR
        qr_codes = decode(frame)

        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8')
            current_time = time.time()

            # Lọc spam: Báo cáo nếu là mã mới hoặc đã qua 3 giây
            if (qr_data != self.last_scanned_data) or (current_time - self.last_scan_time > self.cooldown_time):
                self.get_logger().info(f"🎯 Đã phát hiện kiện hàng: {qr_data}")
                
                # ĐÓNG GÓI VÀ BẮN LÊN ROS 2 TOPIC
                msg = String()
                msg.data = qr_data
                self.publisher_.publish(msg)

                # Cập nhật bộ nhớ tạm
                self.last_scanned_data = qr_data
                self.last_scan_time = current_time

    def destroy_node(self):
        # Tắt camera an toàn khi dập Node
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeScanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Đã nhận lệnh tắt Node (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()