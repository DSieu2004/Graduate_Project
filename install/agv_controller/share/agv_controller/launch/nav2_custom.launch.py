# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
#     # 1. Đường dẫn tới file Map của bạn
#     map_yaml_file = os.path.join(os.path.expanduser('~'), 'my_room_map.yaml')
#     my_params_file = os.path.join(
#         get_package_share_directory('agv_controller'),'config','my_nav2_params.yaml')

#     return LaunchDescription([
#         # 1. Bat LiDAR LDS-01/02
#         Node(
#             package='hls_lfcd_lds_driver',
#             executable='hlds_laser_publisher',
#             name='hlds_laser_publisher',
#             parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'laser'}]
#         ),

#         # 2. Khung toa do TF: base_link -> laser (LiDAR)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_transform_publisher_laser',
#             arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
#         ),

#         # 3. Khung toa do TF: base_link -> base_footprint
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_transform_publisher_footprint',
#             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
#         ),

#         # 4. Node dieu khien trung tam: Tinh Odom + Giao tiep STM32
#         Node(
#             package='agv_controller',
#             executable='diff_drive_controller',
#             name='diff_drive_controller',
#             output='screen'
#         ),

#         # 5. Khoi chay Nav2 
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
#             ),
#             launch_arguments={
#                 'map': map_yaml_file,
#                 'use_sim_time': 'False',
#                 'params_file': my_params_file 
#             }.items()
#         )
#     ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Khai báo các đường dẫn gói (Packages)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('agv_controller')
    
    # 2. Khai báo đường dẫn các file cấu hình (Config)
    map_yaml_file = os.path.join(os.path.expanduser('~'), 'my_room_map.yaml')
    my_params_file = os.path.join(pkg_share, 'config', 'my_nav2_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml') # Đã thêm EKF config

    return LaunchDescription([
        # --- 1. KHỞI ĐỘNG LIDAR ---
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'laser', 'use_sim_time': False}],
            output='screen'
        ),

        # --- 2. CÂY TỌA ĐỘ (TF TREE) ---
        # Khung TF: base_link -> laser (Vị trí LiDAR trên xe)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        # Khung TF: base_link -> base_footprint (Hình chiếu của xe xuống mặt đất)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # Khung TF: base_link -> imu_link (Vị trí thực tế của BNO055 trên xe)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=['0.095', '-0.07', '0.02', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # --- 3. ĐIỀU KHIỂN TRUNG TÂM (STM32 INTERFACE) ---
        # Đọc Odom từ bánh xe, đọc Yaw từ IMU và phát topic /odom + /imu/data
        Node(
            package='agv_controller',
            executable='diff_drive_controller',
            name='diff_drive_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # --- 4. BỘ LỌC EKF (SENSOR FUSION) ---
        # Lấy /odom và /imu/data trộn lại, phát ra TF: odom -> base_link cực mượt
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': False}]
        ),

        # --- 5. HỆ THỐNG ĐIỀU HƯỚNG NAV2 ---
        # Chạy AMCL, Planner, Controller (RPP), Behavior...
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': 'False',
                'params_file': my_params_file,
                'autostart': 'True'
            }.items()
        )
    ])

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


# def generate_launch_description():

#     # 1. Khai báo các đường dẫn gói (Packages)
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
#     pkg_share        = get_package_share_directory('agv_controller')

#     # 2. Khai báo đường dẫn các file cấu hình (Config)
#     map_yaml_file   = os.path.join(os.path.expanduser('~'), 'my_room_map.yaml')
#     my_params_file  = os.path.join(pkg_share, 'config', 'my_nav2_params.yaml')
#     ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

#     return LaunchDescription([

#         # -------------------------------------------------------
#         # 1. KHỞI ĐỘNG LIDAR
#         # -------------------------------------------------------
#         Node(
#             package='hls_lfcd_lds_driver',
#             executable='hlds_laser_publisher',
#             name='hlds_laser_publisher',
#             parameters=[{
#                 'port': '/dev/ttyUSB0',
#                 'frame_id': 'laser',
#                 'use_sim_time': False
#             }],
#             output='screen'
#         ),

#         # -------------------------------------------------------
#         # 2. CÂY TỌA ĐỘ (TF TREE)
#         # -------------------------------------------------------
#         # base_link → laser (vị trí LiDAR trên xe)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_tf_laser',
#             arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
#         ),

#         # base_link → base_footprint (hình chiếu xe xuống mặt đất)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_tf_footprint',
#             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
#         ),

#         # base_link → imu_link (vị trí thực tế BNO055 trên xe)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_tf_imu',
#             arguments=['0.095', '-0.07', '0.02', '0', '0', '0', 'base_link', 'imu_link']
#         ),

#         # -------------------------------------------------------
#         # 3. ĐIỀU KHIỂN TRUNG TÂM (STM32 INTERFACE)
#         # -------------------------------------------------------
#         Node(
#             package='agv_controller',
#             executable='diff_drive_controller',
#             name='diff_drive_controller',
#             output='screen',
#             parameters=[{'use_sim_time': False}]
#         ),

#         # -------------------------------------------------------
#         # 4. BỘ LỌC EKF (SENSOR FUSION)
#         # -------------------------------------------------------
#         Node(
#             package='robot_localization',
#             executable='ekf_node',
#             name='ekf_filter_node',
#             output='screen',
#             parameters=[ekf_params_file, {'use_sim_time': False}]
#         ),

#         # -------------------------------------------------------
#         # 5. HỆ THỐNG ĐIỀU HƯỚNG NAV2
#         # -------------------------------------------------------
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
#             ),
#             launch_arguments={
#                 'map':         map_yaml_file,
#                 'use_sim_time': 'False',
#                 'params_file': my_params_file,
#                 'autostart':   'True'
#             }.items()
#         ),

#         # -------------------------------------------------------
#         # [SỬA LỖI HIGH #8]: Thêm Rosbridge và Web Video Server
#         #
#         # Trước đây Web GUI (index.html) dùng roslib.js kết nối
#         # WebSocket port 9090 và stream camera port 8080, nhưng
#         # launch file không khởi động 2 node này → Web GUI luôn
#         # báo "Lỗi kết nối đến Robot!" dù mạng hoàn toàn bình thường.
#         # -------------------------------------------------------

#         # Rosbridge WebSocket: cho phép Web GUI giao tiếp với ROS2
#         # qua WebSocket (ws://pi4.local:9090)
#         Node(
#             package='rosbridge_server',
#             executable='rosbridge_websocket',
#             name='rosbridge_websocket',
#             output='screen',
#             parameters=[{
#                 'port': 9090,
#                 'use_sim_time': False
#             }]
#         ),

#         # Web Video Server: stream /image_raw ra HTTP
#         # Web GUI nhúng tại: http://pi4.local:8080/stream?topic=/image_raw
#         Node(
#             package='web_video_server',
#             executable='web_video_server',
#             name='web_video_server',
#             output='screen',
#             parameters=[{
#                 'port': 8080,
#                 'use_sim_time': False
#             }]
#         ),

#     ])