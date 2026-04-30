import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # 1. Khai báo các đường dẫn gói
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('agv_drl')
    
    # 2. Khai báo đường dẫn các file cấu hình 
    map_yaml_file = os.path.join(os.path.expanduser('~'), 'my_room_map.yaml')
    my_params_file = os.path.join(pkg_share, 'config', 'my_nav2_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        # --- 1. KHỞI ĐỘNG LIDAR  ---
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'laser', 'use_sim_time': False}],
            output='screen'
        ),

        # --- 2. CÂY TỌA ĐỘ (TF TREE) 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=['0.095', '-0.07', '0.02', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # --- 3. ĐIỀU KHIỂN TRUNG TÂM ---
        Node(
            package='agv_drl',
            executable='diff_drive_controller',
            name='diff_drive_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # --- 4. BỘ LỌC EKF ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': False}]
        ),

        # --- 5. HỆ THỐNG NAV2 ---
        GroupAction(
            actions=[
                SetRemap(src='/cmd_vel', dst='/nav_cmd_vel'),
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
                ),
            ]
        ),

        # --- 6. NÃO BỘ AI (DRL Controller) ---
        Node(
            package='agv_drl',
            executable='drl_controller',
            name='drl_controller_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])