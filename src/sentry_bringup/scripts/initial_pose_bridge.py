#!/usr/bin/env python3
"""
initial_pose_bridge.py
订阅 /initialpose（RViz 2D Pose Estimate），动态发布 map→odom TF。
启动后可自动发布一次默认 initialpose，无需手动 RViz 设点。
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from builtin_interfaces.msg import Time as BuiltinTime


class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__('initial_pose_bridge')

        # 参数：默认初始位姿 (map 系，base_link 的目标位姿)
        self.declare_parameter('default_x', 0.0)
        self.declare_parameter('default_y', 0.0)
        self.declare_parameter('default_yaw_deg', 0.0)
        self.declare_parameter('auto_pose_delay_s', 3.0)  # 等 Point-LIO 初始化

        # TF 基础设施
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅 initialpose（含自己发出的）
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.pose_cb, 10)

        # /initialpose 发布者（供自动发布用）
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # 定时发布 map→odom
        self.timer = self.create_timer(0.05, self.timer_cb)

        # 当前 map→odom 变换（默认 identity）
        self.map_to_odom = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
        }
        self.pose_received = False  # 是否已处理过 initialpose

        # 延迟后自动发一次默认 pose
        delay = self.get_parameter('auto_pose_delay_s').value
        self.auto_pose_timer = self.create_timer(delay, self.auto_pose_cb)

        x = self.get_parameter('default_x').value
        y = self.get_parameter('default_y').value
        yaw = self.get_parameter('default_yaw_deg').value
        self.get_logger().info(
            f'InitialPoseBridge 已启动: 默认位姿 x={x:.2f} y={y:.2f} yaw={yaw:.1f}° 延迟={delay:.1f}s')

    def auto_pose_cb(self):
        """自动发布默认 initialpose（仅一次）"""
        self.auto_pose_timer.cancel()
        if self.pose_received:
            self.get_logger().info('已收到外部 initialpose，跳过自动发布')
            return

        x = self.get_parameter('default_x').value
        y = self.get_parameter('default_y').value
        yaw_deg = self.get_parameter('default_yaw_deg').value
        yaw = math.radians(yaw_deg)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 协方差：偏大的默认值（1m²位置, 0.5rad²朝向）
        cov = [0.0] * 36
        cov[0] = cov[7] = 1.0
        cov[14] = 0.0
        cov[21] = cov[28] = 0.0
        cov[35] = 0.5
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)
        self.get_logger().info(f'自动发布 initialpose: x={x:.2f} y={y:.2f} yaw={yaw_deg:.1f}°')

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """收到 /initialpose（RViz 或自动发布）"""
        self.pose_received = True
        try:
            # 查询 odom → base_link
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))

            odom_q = tf.transform.rotation
            odom_t = tf.transform.translation
        except Exception as e:
            self.get_logger().warn(f'无法获取 odom→base_link TF: {e}')
            return

        # 目标 map 位姿
        mp = msg.pose.pose.position
        mq = msg.pose.pose.orientation

        # 计算: T_map_odom = T_map_base * T_base_odom
        #       = T_map_base * (T_odom_base)^(-1)

        # 用四元数乘法计算
        # q_map_odom = q_map_base * q_odom_base^(-1)
        # q_odom_base 的反转 = conjugate(q_odom_base)
        q_map_base = [mq.x, mq.y, mq.z, mq.w]
        q_odom_base = [odom_q.x, odom_q.y, odom_q.z, odom_q.w]
        q_odom_base_inv = quat_inverse(q_odom_base)
        q_map_odom = quat_multiply(q_map_base, q_odom_base_inv)

        # 位置: t_map_odom = t_map_base - R_map_odom * t_odom_base
        # 先把 odom→base 的平移转到 map 系
        R_mo = quat_to_mat3(q_map_odom)
        t_odom_base_in_map = mat3_mul_vec(R_mo, [odom_t.x, odom_t.y, odom_t.z])

        t_map_odom_x = mp.x - t_odom_base_in_map[0]
        t_map_odom_y = mp.y - t_odom_base_in_map[1]
        t_map_odom_z = -t_odom_base_in_map[2]

        # 保存
        self.map_to_odom = {
            'x': t_map_odom_x, 'y': t_map_odom_y, 'z': 0.0,
            'qx': q_map_odom[0], 'qy': q_map_odom[1],
            'qz': q_map_odom[2], 'qw': q_map_odom[3],
        }
        yaw = 2.0 * math.atan2(q_map_odom[2], q_map_odom[3])
        yaw_base = 2.0 * math.atan2(mq.z, mq.w)
        self.get_logger().info(
            f'已更新 map→odom: x={t_map_odom_x:.3f} y={t_map_odom_y:.3f} yaw={math.degrees(yaw):.1f}°')
        self.get_logger().info(
            f'[推荐配置] default_x={mp.x:.3f}  default_y={mp.y:.3f}  default_yaw_deg={math.degrees(yaw_base):.1f}')

    def timer_cb(self):
        d = self.map_to_odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = d['x']
        t.transform.translation.y = d['y']
        t.transform.translation.z = d['z']
        t.transform.rotation.x = d['qx']
        t.transform.rotation.y = d['qy']
        t.transform.rotation.z = d['qz']
        t.transform.rotation.w = d['qw']
        self.tf_broadcaster.sendTransform(t)


def quat_inverse(q):
    """四元数逆（共轭）"""
    n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
    return [-q[0] / n, -q[1] / n, -q[2] / n, q[3] / n]


def quat_multiply(q1, q2):
    """四元数乘法 q1*q2"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quat_to_mat3(q):
    """四元数转 3x3 旋转矩阵"""
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ]


def mat3_mul_vec(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def main():
    rclpy.init()
    node = InitialPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
