import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('sentry_bringup')

    return LaunchDescription([
        # 1. 点云滤波器
        Node(
            package='cpp_lidar_filter',
            executable='lidar_filter_node',
            name='cpp_lidar_filter',
            output='screen',
            parameters=[{
                'input_topic': '/cloud_registered',
                'output_topic': '/cloud_filtered',
                'max_range': 16.0,
            }]
        ),

        # 2. 3D点云降维 2D
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/cloud_filtered'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 1.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 15.0,
                'use_inf': True,
            }]
        ),

        # 3. 静态 TF: map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        ),
    ])
