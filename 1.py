import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__('multi_goal_navigator')
        # 创建 NavigateToPose 的 Action 客户端
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 定义你的目标点列表 (填入你提供的坐标)
        self.goals = [
            {'x': 2.5415921211242676, 'y': 2.3273658752441406, 'z': 0.0, 'w': 1.0},
            {'x': 7.223820209503174,  'y': 2.540931224822998, 'z': 0.0, 'w': 1.0},
            {'x': 5.21904993057251,  'y': -2.005275249481201, 'z': 0.0, 'w': 1.0}
        ]
        self.current_goal_index = 0

    def send_next_goal(self):
        # 检查是否所有点都已执行完毕
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('✅ 所有目标点导航已完成！')
            rclpy.shutdown()
            return

        # 等待动作服务端上线
        self._action_client.wait_for_server()
        
        # 构造当前目标点消息
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        current = self.goals[self.current_goal_index]
        goal_msg.pose.pose.position.x = current['x']
        goal_msg.pose.pose.position.y = current['y']
        goal_msg.pose.pose.position.z = current['z']
        goal_msg.pose.pose.orientation.w = current['w']

        self.get_logger().info(f'🚀 正在向第 {self.current_goal_index + 1} 个目标点发送导航请求...')
        
        # 异步发送目标，并注册结果回调函数
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 目标点被导航服务器拒绝！')
            return

        self.get_logger().info('🎯 目标点已被接受，机器人正在移动...')
        
        # 获取最终执行结果（成功或失败）
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # 导航完成后，索引加1，继续发送下一个点
        self.current_goal_index += 1
        self.get_logger().info('🎉 已到达当前目标点，准备前往下一个点。')
        
        # 递归调用，发送下一个目标
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    navigator = MultiGoalNavigator()
    # 启动发送第一个目标点
    navigator.send_next_goal()
    rclpy.spin(navigator)
    navigator.destroy_node()

if __name__ == '__main__':
    main()
