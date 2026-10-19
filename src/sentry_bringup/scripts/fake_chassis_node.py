#!/usr/bin/env python3
"""
Fake Chassis Node - 虚拟底盘节点（闭环测试用）

订阅 /cmd_vel，根据速度指令积分更新位姿，模拟真实底盘响应。
替换 fake_odom_node 用于闭环导航测试。

底盘模型: 全向底盘 (Omni)
  - vx, vy, wz 完全独立
  - 位姿积分使用全向运动学模型
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros
import math


class FakeChassisNode(Node):
    def __init__(self):
        super().__init__('fake_chassis_node')

        # 初始位姿
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.cmd_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        self.last_time = self.get_clock().now()

        # 订阅 Nav2 输出的速度指令
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # 订阅 RViz '2D Pose Estimate' 发布的初始位置
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_cb, 10)

        # 发布模拟的 Point-LIO 输出 (Odometry，与 odom_relay 匹配)
        self.pub = self.create_publisher(
            Odometry, '/aft_mapped_to_init', 50)

        # TF 广播器: camera_init -> aft_mapped
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 50Hz 更新
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info(
            'FakeChassisNode 启动: 订阅 /cmd_vel 和 /initialpose，全向底盘模型积分更新位姿')

    def initial_pose_cb(self, msg):
        """接收 RViz '2D Pose Estimate' 设置初始位姿"""
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        # 从四元数提取 yaw
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.pose['theta'] = 2.0 * math.atan2(qz, qw)
        self.get_logger().info(
            f"初始位姿已重置: ({self.pose['x']:.2f}, {self.pose['y']:.2f}, {self.pose['theta']:.2f})")

    def cmd_vel_cb(self, msg):
        """接收 Nav2 控制器输出的速度指令"""
        # 修正自转：wz 取反（防止 controller 和 chassis 对 yaw 理解相反导致原地打转）
        # vx/vy 保持原方向
        self.cmd_vel['vx'] = msg.linear.x
        self.cmd_vel['vy'] = msg.linear.y
        self.cmd_vel['wz'] = -msg.angular.z

    def update(self):
        """根据速度指令进行位姿积分（全向底盘模型）"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        vx = self.cmd_vel['vx']
        vy = self.cmd_vel['vy']
        wz = self.cmd_vel['wz']
        theta = self.pose['theta']

        # 全向底盘运动学积分
        # 速度在世界坐标系下的分量
        self.pose['x'] += (vx * math.cos(theta) - vy * math.sin(theta)) * dt
        self.pose['y'] += (vx * math.sin(theta) + vy * math.cos(theta)) * dt
        self.pose['theta'] += wz * dt

        # 归一化角度到 [-pi, pi]
        self.pose['theta'] = math.atan2(
            math.sin(self.pose['theta']), math.cos(self.pose['theta']))

        # 构造 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'camera_init'
        odom_msg.child_frame_id = 'body'
        # yaw 取反发布（与 wz 修正对应），y 轴保持原方向
        odom_msg.pose.pose.position.x = self.pose['x']
        odom_msg.pose.pose.position.y = self.pose['y']
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = -math.sin(self.pose['theta'] / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.pose['theta'] / 2.0)
        # twist 填入当前速度（可选，odom_relay 透传时可能用到）
        odom_msg.twist.twist.linear.x = self.cmd_vel['vx']
        odom_msg.twist.twist.linear.y = self.cmd_vel['vy']
        odom_msg.twist.twist.angular.z = self.cmd_vel['wz']

        self.pub.publish(odom_msg)

        # 广播 TF: camera_init -> aft_mapped
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'camera_init'
        t.child_frame_id = 'body'
        # yaw 取反发布（与 wz 修正对应），y 轴保持原方向
        t.transform.translation.x = self.pose['x']
        t.transform.translation.y = self.pose['y']
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = -math.sin(self.pose['theta'] / 2.0)
        t.transform.rotation.w = math.cos(self.pose['theta'] / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
