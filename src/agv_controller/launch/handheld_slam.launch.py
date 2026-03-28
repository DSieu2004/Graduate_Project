import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('agv_controller')

    slam_params_file = os.path.join(
        pkg_share,
        'config',
        'slam_handheld.yaml'
    )

    return LaunchDescription([

        # LiDAR Driver
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'frame_id': 'laser'
            }]
        ),

        # base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','base_link','laser']
        ),

        # odom -> base_link 
        Node(
            package='agv_controller',
            executable='diff_drive_controller',
            name='diff_drive_controller',
            output='screen'
        ),

        # SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file,{'use_sim_time': False}]
        )
    ])
