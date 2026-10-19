#!/usr/bin/env python3
"""
goal_pose_bridge.py

桥接节点：订阅 RViz '2D Goal Pose' 发布的 /goal_pose 话题，
自动调用 Nav2 的 /navigate_to_pose action server。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')
        
        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)
        
        self.action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        
        self.get_logger().info(
            'GoalPoseBridge 启动: /goal_pose -> /navigate_to_pose')

    def goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            f'收到 /goal_pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        self.action_client.wait_for_server(timeout_sec=1.0)
        self.action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
