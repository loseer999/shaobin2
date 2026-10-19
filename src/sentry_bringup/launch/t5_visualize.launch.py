"""
T5 可视化对比工具（独立 launch）：

用途：肉眼对比先验 PCD 地图（红色）与 Point-LIO 实时点云（绿色），
       排查 GICP 不收敛是否为 Z 轴 / XY 偏移导致。

启动方式（与 core_pipeline.launch.py 并行另开终端使用）：
    ros2 launch sentry_bringup t5_visualize.launch.py

话题约定：
    /prior_map      ← PCD 地图，frame_id = odom，1Hz 发布
    /cloud_filtered ← 实时滤除车体后的点云（来自 core_pipeline）
    /cloud_registered ← Point-LIO 累积点云（默认隐藏，需手动勾选）

Fixed Frame = odom（PCD 实际就保存在 odom 系；map→odom 由 GICP 提供，
本工具不依赖该 TF 即可显示）。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sentry_bringup')
    pcd_map_path = os.path.join(bringup_dir, 'maps', 'cubot.PCD')
    rviz_config = os.path.join(bringup_dir, 'config', 't5_compare.rviz')

    return LaunchDescription([
        # 1. 把先验 PCD 当作话题发布出来，Fixed Frame 设为 odom
        #    注意: humble 下 pcl_ros pcd_to_pointcloud 节点真名为 pcd_publisher,
        #    必须用 parameters 传入 file_name/tf_frame/publishing_period_ms,
        #    不接受位置参数 (位置参数会被 rclcpp 当作未知 ROS args 报错)
        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pcd_publisher',
            output='screen',
            parameters=[{
                'file_name': pcd_map_path,
                'tf_frame': 'odom',
                'publishing_period_ms': 1000,
            }],
            remappings=[
                ('cloud_pcd', '/prior_map'),
            ],
        ),

        # 2. RViz2 对比视图
        Node(
            package='rviz2',
            executable='rviz2',
            name='t5_compare_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
