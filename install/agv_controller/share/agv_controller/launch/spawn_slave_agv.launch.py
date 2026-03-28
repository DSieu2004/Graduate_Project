import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Lấy mô hình Turtlebot3
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(turtlebot3_gazebo_dir, 'models')
    urdf_file = os.path.join(turtlebot3_gazebo_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')

    # 2. Mở thế giới ảo Gazebo (Dùng thế giới rỗng)
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        )
    )

    # ==========================================
    # 🤖 CHỈ THẢ ROBOT 2 (SLAVE - Namespace: /agv2)
    # ==========================================
    agv2_name = 'agv2'
    
    agv2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=agv2_name,
        output='screen',
        parameters=[{'frame_prefix': agv2_name + '/', 'use_sim_time': True}],
        arguments=[urdf_file]
    )

    # Đặt AGV 2 cách xa gốc tọa độ một chút (Trùng với tọa độ HOME trên Web)
    spawn_agv2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', agv2_name,
            '-file', urdf_file,
            '-x', '-1.30', '-y', '0.18', '-z', '0.01', # Tọa độ xấp xỉ vị trí đỗ của AGV2
            '-robot_namespace', agv2_name
        ],
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg="🚀 BẮT ĐẦU THẢ AGV 2 (SLAVE ẢO) VÀO GAZEBO..."),
        gazebo,
        agv2_state_publisher,
        spawn_agv2
    ])