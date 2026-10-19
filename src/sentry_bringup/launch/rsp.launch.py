"""Robot State Publisher 独立启动入口（base_link↔lidar_link 等静态 TF）。
LIO 必须先有 base_link↔lidar_link 静态 TF 才能反算 base_link 位姿。
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sentry_bringup')
    xacro_file = os.path.join(bringup_dir, 'urdf', 'guard.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}],
        ),
    ])
