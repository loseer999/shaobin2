#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses  # 1. 替换为途径点Action
from geometry_msgs.msg import PoseStamped

class SmoothPatrolNode(Node):
    def __init__(self):
        super().__init__('smooth_patrol_node')
        
        # 2. 连接到 navigate_through_poses 服务器
        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        self.waypoints = [
            (1.0, 1.0), (2.0, 2.0), (3.0, 3.0),
            (4.0, 4.0), (5.0, 5.0), (6.0, 6.0),
            (7.0, 7.0), (8.0, 8.0), (9.0, 9.0)
        ]
        
        self.get_logger().info('正在等待 Nav2 NavigateThroughPoses 服务器启动...')
        self.client.wait_for_server()
        self.get_logger().info('服务器已连接！开始执行全路段丝滑巡航任务。')
        
        # 直接发送整条路线
        self.send_route()

    def send_route(self):
        # 3. 构造途径点导航目标消息
        goal_msg = NavigateThroughPoses.Goal()
        
        # 把所有 (x,y) 坐标一次性打包成 PoseStamped 列表
        poses = []
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0 
            poses.append(pose)
            
        goal_msg.poses = poses
        
        self.get_logger().info(f'>>> 已下发包含 {len(self.waypoints)} 个途径点的平滑路线！')
        
        # 异步发送目标 (只需发一次)
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('路线被 Nav2 拒绝执行！请检查目标点是否在障碍物上。')
            return

        self.get_logger().info('路线已被接受，哨兵正在机动...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('✅ 漂亮！已丝滑穿过所有 9 个坐标点！')
        else:
            self.get_logger().warn(f'❌ 巡航中断，状态码: {status}')
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SmoothPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户手动中断了巡航节点。')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()