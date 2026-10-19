#!/usr/bin/env python3
"""
连续航点发送脚本
用法:
  ros2 run cubot_guard_navigation send_waypoints.py --ros-args -p waypoints:="[[1.0,0.0],[2.0,1.0],[0.0,0.0]]"
  
或者在 launch 中:
  Node(
    package='cubot_guard_navigation',
    executable='send_waypoints.py',
    parameters=[{'waypoints': [[1.0, 0.0], [2.0, 1.0], [0.0, 0.0]]}]
  )
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
import json


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        # 声明参数: 航点列表 [[x1,y1], [x2,y2], ...]
        self.declare_parameter('waypoints', [[1.0, 0.0], [2.0, 1.0], [0.0, 0.0]])
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('loop', False)  # 是否循环巡逻
        self.declare_parameter('loop_delay_sec', 5.0)

        self.waypoints = self.get_parameter('waypoints').value
        self.frame_id = self.get_parameter('frame_id').value
        self.loop = self.get_parameter('loop').value
        self.loop_delay = self.get_parameter('loop_delay_sec').value

        self.action_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses')

        self.get_logger().info(
            f'航点: {self.waypoints}, 循环: {self.loop}')

    def send_waypoints(self):
        if not self.waypoints:
            self.get_logger().error('航点列表为空!')
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        for i, (x, y) in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            goal_msg.poses.append(pose)
            self.get_logger().info(f'航点 {i}: ({x}, {y})')

        self.action_client.wait_for_server(timeout_sec=5.0)
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 被拒绝')
            return

        self.get_logger().info('Goal 已接受，开始连续航点导航')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'导航完成, 结果: {result}')

        if self.loop:
            self.get_logger().info(
                f'等待 {self.loop_delay}s 后开始下一轮巡逻...')
            self.create_timer(self.loop_delay, self.send_waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    node.send_waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
