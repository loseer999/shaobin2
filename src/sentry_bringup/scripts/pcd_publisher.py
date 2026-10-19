#!/usr/bin/env python3
"""简单的 PCD 地图发布节点，用于 RViz 可视化"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct


class PcdPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/map_cloud')
        self.declare_parameter('publish_rate', 0.2)  # Hz

        pcd_file = self.get_parameter('pcd_file').value
        frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        rate = self.get_parameter('publish_rate').value

        if not pcd_file:
            self.get_logger().error('pcd_file parameter is empty!')
            return

        # Load PCD
        points = self.load_pcd(pcd_file)
        if points is None or len(points) == 0:
            self.get_logger().error(f'Failed to load PCD: {pcd_file}')
            return

        self.get_logger().info(f'Loaded PCD: {len(points)} points from {pcd_file}')

        # Build PointCloud2 message
        self.msg = self.build_pointcloud2(points, frame_id)

        # Publisher
        self.pub = self.create_publisher(PointCloud2, topic, 1)
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f'Publishing to {topic} at {rate} Hz, frame_id={frame_id}')

    def load_pcd(self, filename):
        """简易 PCD 解析器，支持 ASCII 和 binary"""
        try:
            with open(filename, 'rb') as f:
                header = {}
                while True:
                    line = f.readline().decode('ascii').strip()
                    if line.startswith('DATA'):
                        header['DATA'] = line.split()[-1]
                        break
                    parts = line.split()
                    if len(parts) >= 2:
                        header[parts[0]] = parts[1:]

                width = int(header.get('WIDTH', [0])[0])
                height = int(header.get('HEIGHT', [0])[0])
                points_count = width * height

                # Parse fields
                fields = header.get('FIELDS', [])
                sizes = [int(x) for x in header.get('SIZE', [])]
                types = header.get('TYPE', [])
                counts = [int(x) for x in header.get('COUNT', [])]

                dtype = []
                for t, s in zip(types, sizes):
                    if t == 'F' and s == 4:
                        dtype.append(('x', np.float32))
                    elif t == 'F' and s == 8:
                        dtype.append(('x', np.float64))
                    else:
                        dtype.append(('x', np.float32))

                data_type = header['DATA']
                if data_type == 'ascii':
                    data = f.read().decode('ascii').strip().split('\n')
                    points = []
                    for line in data[:points_count]:
                        vals = [float(x) for x in line.split()]
                        points.append(vals[:3])
                    return np.array(points, dtype=np.float32)
                elif data_type == 'binary':
                    # Simple binary reader for x,y,z float
                    if 'FIELDS' in header and header['FIELDS'][:3] == ['x', 'y', 'z']:
                        point_size = sum(sizes)
                        buf = f.read(points_count * point_size)
                        fmt = '<' + ''.join(['f' if t == 'F' and s == 4 else 'd' if t == 'F' and s == 8 else 'f'
                                             for t, s in zip(types, sizes)])
                        points = []
                        for i in range(points_count):
                            offset = i * point_size
                            vals = struct.unpack_from(fmt, buf, offset)
                            points.append(vals[:3])
                        return np.array(points, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f'PCD load error: {e}')
            return None
        return None

    def build_pointcloud2(self, points, frame_id):
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = PcdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
