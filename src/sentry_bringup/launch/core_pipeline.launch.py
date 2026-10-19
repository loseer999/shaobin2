import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 获取 sentry_bringup 包的共享目录
    bringup_dir = get_package_share_directory('sentry_bringup')
    
    # 找到你刚写的 EKF 配置文件
    ekf_config_path = os.path.join(bringup_dir, 'config', 'ekf_fusion.yaml')
    
    # 使用 scans.pcd 作为先验地图
    pcd_map_path = os.path.join(bringup_dir, 'maps', 'scans.pcd')

    # 【T5 可视化】默认启用 RViz + PCD publisher (肉眼对比 红色先验地图 / 绿色实时点云)
    rviz_config = os.path.join(bringup_dir, 'config', 't5_compare.rviz')

    return LaunchDescription([
        
        # 1. 点云滤波器 (已禁用: GICP 和 Nav2 都直接订阅 /cloud_registered_body)
        # Node(
        #     package='cpp_lidar_filter',
        #     executable='lidar_filter_node',
        #     name='cpp_lidar_filter',
        #     output='screen',
        #     parameters=[{
        #         'input_topic': '/cloud_registered_body',
        #         'output_topic': '/cloud_filtered',
        #         'min_x': -0.33, 'max_x': 0.33,
        #         'min_y': -0.33, 'max_y': 0.33,
        #         'min_z': 0.085, 'max_z': 0.55,
        #         'negative': True,
        #         'max_range': 16.0,
        #     }]
        # ),

        # 2. 3D点云降维 2D (喂给 Nav2 局部避障)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/cloud_registered_body'), # 直接订阅 body 点云
                ('scan', '/scan')                # 输出 2D 雷达数据给 Nav2
            ],
            parameters=[{
                'target_frame': 'base_link',                # 【fake_vel 已弃用】直接投射到 base_link, 不再走虚拟稳定层
                'transform_tolerance': 0.01,
                'min_height': 0.1,  # 过滤掉地面的点
                'max_height': 1.5,  # 过滤掉太高的点
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 15.0,
                'use_inf': True,
                'scan_qos': 'RELIABLE',  # 匹配 Nav2 的 Reliable QoS
            }]
        ),

        # 3. 【已弃用】fake_vel_transform 轮椒动力变换节点
        #     原作用: 为 RM 小陀螺场景合成 (vx,vy,wz) + spin 为底盘输入。
        #     废弃原因:
        #       - 舵轮架构不需要小陀螺 (不会设 /chassis/target_angular_z)
        #       - 输出 /serial/cmd_vel 无订阅者, serial_send_node 实际订阅 /cmd_vel
        #       - velocity_smoother 已接管: cmd_vel_nav → smoother → cmd_vel → serial_send
        #     同时 pointcloud_to_laserscan 的 target_frame 从
        #         base_footprint_stabilized 改回 base_link (虚拟层 TF 源于 fake_vel)
        # Node(
        #     package='fake_vel_transform',
        #     executable='fake_vel_transform_node',
        #     name='fake_vel_transform',
        #     output='screen',
        #     parameters=[{
        #         'robot_base_frame': 'base_link',
        #         'fake_robot_base_frame': 'base_footprint_stabilized',
        #         'odom_topic': '/Odometry',
        #         'input_cmd_vel_topic': '/cmd_vel_nav',
        #         'cmd_spin_topic': '/chassis/target_angular_z',
        #         'output_cmd_vel_topic': '/serial/cmd_vel',
        #         'init_spin_speed': 0.0
        #     }]
        # ),


        # 4. 【已禁用】静态映射 (GICP 启用时由 EKF 发布 map→odom)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_map_to_odom',
        #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        # ),

        # 5. 全局匹配重定位 (small_gicp)
        Node(
            package='small_gicp_relocalization',
            executable='small_gicp_relocalization_node',
            name='small_gicp_node',
            output='screen',
            parameters=[{
                'prior_pcd_file': pcd_map_path,
                'input_cloud_topic': '/cloud_registered_body',  # 直接订阅 Point-LIO body 点云
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'robot_base_frame': 'base_link',
                'lidar_frame': 'lidar_link',
                'registered_leaf_size': 0.05,
                'global_leaf_size': 0.10,
                'num_neighbors': 15,
                'max_dist_sq': 4.0,
                'convergence_radius': 0.2,
                'max_consecutive_failures': 20,
                'init_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }],
            remappings=[('/tf', '/tf_null')]
        ),

        # 6. EKF 状态融合 (终极防线：防止被遮挡时瞬移)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_fusion_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),

        # 5.5 【已拆出】KISS-Matcher 全局重定位
        #     拆到独立终端 T5K (base_bringup.sh 阶段 5 之后启动),
        #     方便单独观察 inliers / Registration succeeded 日志。
        #     节点 executable: kiss_matcher_relocalization_node
        #     参数与原配置完全一致, 见 base_bringup.sh 中 T5K 命令。

        # 6. 【可视化】发布先验 PCD 地图为话题 /prior_map (frame_id=map, 1Hz)
        #    【重要】 PCD 是绝对地图 → 挂 map 帧 (不是 odom);
        #    这样机器人重启位置不在建图起点时, RViz 仍能看到红绿对齐 (靠 GICP 发的 map→odom)
        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pcd_publisher',
            output='screen',
            parameters=[{
                'file_name': pcd_map_path,
                'tf_frame': 'map',
                'publishing_period_ms': 1000,
            }],
            remappings=[
                ('cloud_pcd', '/prior_map'),
            ],
        ),

        # 7. 【可视化】RViz2 红绿点云对比 (Fixed Frame=odom, 不依赖 GICP 发 map→odom)
        Node(
            package='rviz2',
            executable='rviz2',
            name='t5_compare_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        )
    ])