import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_yaml_file = os.path.join(os.path.expanduser('~'), 'my_room_map.yaml')

    return LaunchDescription([
        # 1. Bat LiDAR LDS-01/02
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'laser'}]
        ),

        # 2. Khung toa do TF: base_link -> laser (LiDAR)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        # 3. MỚI THÊM: Khung toa do TF: base_link -> base_footprint (Gia lap de chieu long Nav2)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # 4. Node dieu khien trung tam: Tinh Odom + Giao tiep STM32
        Node(
            package='agv_controller',
            executable='diff_drive_controller',
            name='diff_drive_controller',
            output='screen'
        ),

        # 5. Khoi chay Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': 'False'
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

#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
#     map_yaml_file = os.path.join(os.path.expanduser('~'), 'my_room_map.yaml')

#     return LaunchDescription([

#         # LiDAR
#         Node(
#             package='hls_lfcd_lds_driver',
#             executable='hlds_laser_publisher',
#             name='hlds_laser_publisher',
#             parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'laser'}]
#         ),

#         # TF base_link -> laser
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_transform_publisher_laser',
#             arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
#         ),

#         # TF base_footprint -> base_link 
#         Node(
#              package='tf2_ros',
#              executable='static_transform_publisher',
#              name='static_transform_publisher_footprint',
#              arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
#          ),

#         # diff drive
#         Node(
#             package='agv_controller',
#             executable='diff_drive_controller',
#             name='diff_drive_controller',
#             output='screen'
#         ),

#         # Nav2
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
#             ),
#             launch_arguments={
#                 'map': map_yaml_file,
#                 'use_sim_time': 'false'
#             }.items()
#         )

#     ])