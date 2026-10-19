#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # 1. 初始化发布者，对应 serial_send.cpp 中的订阅话题
        self.mode_pub = self.create_publisher(String, '/sentry/combat_mode', 10)
        
        # 2. 初始化 Nav2 导航器
        self.navigator = BasicNavigator()
        
        # 3. 等待 Nav2 激活 (在实机启动时非常重要)
        self.get_logger().info('正在等待 Nav2 激活...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 已就绪，开始执行任务序列。')

    def change_mode(self, mode_string):
        """发送模式切换指令"""
        msg = String()
        msg.data = mode_string
        self.mode_pub.publish(msg)
        self.get_logger().info(f'发送姿态切换: {mode_string}')
        # 等待机械结构动作时间
        time.sleep(1.5)

    def go_to_pose(self, x, y):
        """执行导航"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.w = 1.0
        
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'成功到达目标点: [{x}, {y}]')
        else:
            self.get_logger().error(f'导航至 [{x}, {y}] 失败！')

    def run(self):
        # 阶段 1：进攻模式 -> [1, 1]
        self.change_mode("attack")
        self.go_to_pose(1.0, 1.0)

        # 阶段 2：防御模式 -> [0, 0]
        # 根据 serial_send.cpp，防御模式字符串应为 "spin_defense"
        self.change_mode("spin_defense")
        self.go_to_pose(0.0, 0.0)

        # 阶段 3：移动姿态 -> [1, 1]
        # 根据 serial_send.cpp，普通/移动模式字符串应为 "normal"
        self.change_mode("normal")
        self.go_to_pose(1.0, 1.0)

        self.get_logger().info('所有预设任务已执行完毕。')

def main():
    rclpy.init()
    node = MissionManager()
    node.run()
    # 任务完成后保持节点运行，或直接 shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()