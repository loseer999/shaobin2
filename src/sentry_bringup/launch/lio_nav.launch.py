import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('sentry_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    params_file  = LaunchConfiguration('params_file')
    use_respawn  = LaunchConfiguration('use_respawn')
    log_level    = LaunchConfiguration('log_level')

    # Nav2 lifecycle 节点
    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': xacro.process_file(
                    os.path.join(bringup_dir, 'urdf', 'guard.xacro')).toxml()
            }]
        ),

        # 2. Point-LIO
        Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='pointlio_mapping',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('point_lio'), 'config', 'mid360.yaml')],
            remappings=[('/Odometry', '/odom')]
        ),

        # 3. 点云滤波
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

        # 4. 3D→2D laserscan
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

        # 5. 动态 map→odom: 监听 /initialpose (RViz 2D Pose) 自动对齐
        Node(
            package='sentry_bringup',
            executable='initial_pose_bridge.py',
            name='initial_pose_bridge',
            output='screen',
            parameters=[{
                'default_x': 0.0,
                'default_y': 0.0,
                'default_yaw_deg': 0.0,
                'auto_pose_delay_s': 5.0,  # 等 Point-LIO 完成 IMU 初始化
            }],
        ),

        # 6. Nav2
        GroupAction(actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(package='nav2_map_server', executable='map_server',
                name='map_server', output='screen', respawn=use_respawn, respawn_delay=2.0,
                parameters=[{
                    'yaml_filename': os.path.join(bringup_dir, 'maps', 'scans.yaml'),
                    'frame_id': 'map',
                }],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(package='nav2_controller', executable='controller_server',
                output='screen', respawn=use_respawn, respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(package='nav2_smoother', executable='smoother_server',
                name='smoother_server', output='screen',
                respawn=use_respawn, respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(package='nav2_planner', executable='planner_server',
                name='planner_server', output='screen',
                respawn=use_respawn, respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(package='nav2_behaviors', executable='behavior_server',
                name='behavior_server', output='screen',
                respawn=use_respawn, respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                name='bt_navigator', output='screen',
                respawn=use_respawn, respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                name='waypoint_follower', output='screen',
                respawn=use_respawn, respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                name='velocity_smoother', output='screen',
                respawn=use_respawn, respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
            ),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_navigation', output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ]),

        # 7. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup_dir, 'config', 'lio_nav.rviz')],
            output='screen',
        ),
    ])
