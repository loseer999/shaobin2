#!/usr/bin/env python3
"""
fake_pointcloud_node.py

模拟 Point-LIO 的 /cloud_registered_body，用于无雷达本地测试：
- 在机体坐标系 (frame_id="body") 下生成一圈虚拟墙壁点云
- pointcloud_to_laserscan 会将其投影为 2D LaserScan
- Nav2 代价地图据此构建障碍物
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import math


class FakePointCloudNode(Node):
    def __init__(self):
        super().__init__('fake_pointcloud_node')

        self.cloud_pub = self.create_publisher(
            PointCloud2, '/cloud_registered_body', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # 场地墙壁参数：生成一个 8m x 15m 的矩形场地边界
        # 以机器人为中心，四周 2~5m 处有墙壁
        self.wall_distance_min = 2.0
        self.wall_distance_max = 5.0
        self.wall_height = 0.5  # 墙高，用于 pointcloud_to_laserscan 的 height filter
        self.num_points = 720   # 每圈 720 个点

        self.get_logger().info(
            'FakePointCloudNode 启动: 生成虚拟墙壁点云 (%d points)' % self.num_points)

    def timer_callback(self):
        points = []
        for i in range(self.num_points):
            angle = 2.0 * math.pi * i / self.num_points
            # 统一半径生成圆环墙壁，避免相邻角度障碍物交替导致raytrace混乱
            dist = 8.0
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            z = 0.0  # 地面高度
            points.append((x, y, z))

        # 打包为 PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'body'

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * len(points)
        cloud.is_bigendian = False
        cloud.is_dense = True

        buffer = []
        for x, y, z in points:
            buffer.append(struct.pack('fff', x, y, z))
        cloud.data = b''.join(buffer)

        self.cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = FakePointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
