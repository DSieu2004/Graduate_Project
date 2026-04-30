import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from pyzbar.pyzbar import decode
from rclpy.qos import qos_profile_sensor_data
import threading
from flask import Flask, jsonify
from flask_cors import CORS

# --- KHỞI TẠO WEB SERVER MINI ---
app = Flask(__name__)
CORS(app) # Cho phép Dashboard kết nối thoải mái

latest_qr = "" # Biến toàn cục để lưu mã vừa quét

@app.route('/get_qr')
def get_qr():
    global latest_qr
    res = latest_qr
    latest_qr = "" # Web lấy xong thì xóa đi để không in trùng
    return jsonify({"data": res})

# --- NODE ĐỌC CAMERA ---
class MyQRReader(Node):
    def __init__(self):
        super().__init__('my_qr_reader_node')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile_sensor_data)
        self.br = CvBridge()

        self.last_scanned_data = ""
        self.last_scan_time = 0
        self.COOLDOWN_TIME = 3.0 
        self.last_frame_process_time = 0
        self.PROCESS_INTERVAL = 0.5 

        self.get_logger().info("✅ Trạm API Quét QR (Cổng 5000) đã sẵn sàng!")

    def image_callback(self, data):
        global latest_qr
        current_time = time.time()
        
        if current_time - self.last_frame_process_time < self.PROCESS_INTERVAL:
            return
        self.last_frame_process_time = current_time

        frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        qr_codes = decode(frame)

        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8')
            
            if (qr_data != self.last_scanned_data) or (current_time - self.last_scan_time > self.COOLDOWN_TIME):
                self.get_logger().info(f"🎯 BÍP! Đã phát hiện: {qr_data}")
                self.last_scanned_data = qr_data
                self.last_scan_time = current_time
                
                # Lưu mã vào biến toàn cục để Web lấy
                latest_qr = qr_data

def run_ros():
    rclpy.init()
    node = MyQRReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # 1. Cho  Web Server API chạy ở luồng phụ (ẩn phía sau)
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
    flask_thread.daemon = True
    flask_thread.start()
    
    # 2. BẮT BUỘC để ROS 2 chạy ở luồng CHÍNH thì Camera mới không bị đơ
    rclpy.init()
    node = MyQRReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import time
# import threading

# from pyzbar.pyzbar import decode
# from rclpy.qos import qos_profile_sensor_data
# from flask import Flask, jsonify
# from flask_cors import CORS

# # ==========================================================
# # KHỞI TẠO WEB SERVER MINI (Flask API)
# # ==========================================================
# app = Flask(__name__)
# CORS(app)

# # Dùng threading.Lock() để bảo vệ biến latest_qr
# # Flask thread (đọc) và ROS thread (ghi) chạy đồng thời →
# # không có lock → race condition → mất dữ liệu QR
# latest_qr = ""
# qr_lock   = threading.Lock()

# @app.route('/get_qr')
# def get_qr():
#     global latest_qr
#     with qr_lock:
#         res       = latest_qr
#         latest_qr = ""  # Xóa sau khi Web lấy để không trả trùng
#     return jsonify({"data": res})

# # ==========================================================
# # NODE ROS 2 ĐỌC CAMERA & GIẢI MÃ QR
# # ==========================================================
# class MyQRReader(Node):

#     def __init__(self):
#         super().__init__('my_qr_reader_node')

#         self.subscription = self.create_subscription(
#             Image,
#             '/image_raw',
#             self.image_callback,
#             qos_profile_sensor_data
#         )

#         self.br = CvBridge()

#         self.last_scanned_data       = ""
#         self.last_scan_time          = 0
#         self.COOLDOWN_TIME           = 3.0   # Tránh quét trùng cùng 1 mã liên tục
#         self.last_frame_process_time = 0
#         self.PROCESS_INTERVAL        = 0.5   # Chỉ xử lý 2 frame/giây → giảm tải CPU Pi

#         self.get_logger().info("✅ Trạm API Quét QR (Cổng 5000) đã sẵn sàng!")

#     def image_callback(self, data):
#         global latest_qr

#         current_time = time.time()
#         if current_time - self.last_frame_process_time < self.PROCESS_INTERVAL:
#             return
#         self.last_frame_process_time = current_time

#         frame    = self.br.imgmsg_to_cv2(data, 'bgr8')
#         qr_codes = decode(frame)

#         for qr in qr_codes:
#             qr_data = qr.data.decode('utf-8')

#             is_new_data    = (qr_data != self.last_scanned_data)
#             is_cooldown_ok = (current_time - self.last_scan_time > self.COOLDOWN_TIME)

#             if is_new_data or is_cooldown_ok:
#                 self.get_logger().info(f"🎯 BÍP! Đã phát hiện: {qr_data}")
#                 self.last_scanned_data = qr_data
#                 self.last_scan_time    = current_time

#                 with qr_lock:
#                     latest_qr = qr_data

# # ==========================================================
# # MAIN
# # ==========================================================
# if __name__ == '__main__':

#     # Chỉ gọi rclpy.init() MỘT LẦN duy nhất
#     # Bản cũ có thêm 1 lần trong hàm run_ros() (dead code) →
#     # nếu vô tình gọi sẽ crash vì init 2 lần
#     rclpy.init()
#     node = MyQRReader()

#     # Flask chạy ở luồng phụ để không block ROS spin
#     flask_thread = threading.Thread(
#         target=lambda: app.run(
#             host='0.0.0.0',
#             port=5000,
#             debug=False,
#             use_reloader=False  # BẮT BUỘC False khi chạy trong thread
#         )
#     )
#     flask_thread.daemon = True
#     flask_thread.start()

#     # ROS spin chạy ở luồng CHÍNH — bắt buộc để callback camera không bị đơ
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()