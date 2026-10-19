#!/usr/bin/env python3
"""
cmd_vel_monitor.py

实时监控 Nav2 速度链路三个阶段的输出，便于诊断速度在哪一环被吞：

  controller_server  ──▶ /cmd_vel_nav       (MPPI 原始输出)
  velocity_smoother  ──▶ /cmd_vel_smoothed  (平滑 + deadband 过滤后)
  collision_monitor  ──▶ /cmd_vel           (最终给底盘的速度，serial_send 订阅)

以 2Hz 打印三路最新值，列对齐，便于目测定位问题环。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMonitor(Node):
    def __init__(self):
        super().__init__('cmd_vel_monitor')

        # 三个阶段的最新缓存
        self.latest = {
            '/cmd_vel_nav':      None,
            '/cmd_vel_smoothed': None,
            '/cmd_vel':          None,
        }

        # 分别订阅
        for topic in self.latest.keys():
            self.create_subscription(
                Twist, topic,
                lambda msg, t=topic: self._cb(t, msg),
                10)

        # 2Hz 打印频率（0.5s 间隔，避免刷屏）
        self.timer = self.create_timer(0.5, self._print)

        self.get_logger().info(
            'CmdVelMonitor 启动: 2Hz 打印 cmd_vel_nav / cmd_vel_smoothed / cmd_vel')
        self.get_logger().info(
            '列: [vx, vy, wz]   "---" = 尚未收到该话题消息')

    def _cb(self, topic: str, msg: Twist):
        self.latest[topic] = (msg.linear.x, msg.linear.y, msg.angular.z)

    @staticmethod
    def _fmt(v):
        if v is None:
            return '[  ---,   ---,   ---]'
        return f'[{v[0]:+6.3f}, {v[1]:+6.3f}, {v[2]:+6.3f}]'

    def _print(self):
        nav      = self._fmt(self.latest['/cmd_vel_nav'])
        smoothed = self._fmt(self.latest['/cmd_vel_smoothed'])
        final    = self._fmt(self.latest['/cmd_vel'])
        # 一行输出 controller → smoother → collision_monitor 三阶段速度
        self.get_logger().info(
            f'nav {nav}  →  smooth {smoothed}  →  out {final}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
