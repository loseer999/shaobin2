/**
 * @file toggle_obstacle_avoidance.hpp
 * @brief 动态切换避障功能的同步动作节点
 * 
 * 功能：
 * - 通过 ROS 2 参数服务动态修改 Nav2 局部规划器参数
 * - 支持 DWB (dwb_core) 和其他常见局部规划器
 * - 可开启或关闭避障功能
 * 
 * 战术用途：
 * - 导航时开启避障，确保安全到达目标
 * - 到达控制区后关闭避障，防止被敌方机器人挤出控制区
 * 
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__TOGGLE_OBSTACLE_AVOIDANCE_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__TOGGLE_OBSTACLE_AVOIDANCE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <chrono>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

namespace cubot_guard_behavior_tree
{

/**
 * @brief ToggleObstacleAvoidance - 动态切换避障功能
 * 
 * 输入端口:
 *   - enable: 是否开启避障 (true/false)
 *   - controller_server_name: 控制器服务器名称 (默认: controller_server)
 *   - max_vel_x_enabled: 开启避障时的最大速度 (默认: 0.5)
 *   - max_vel_x_disabled: 关闭避障时的最大速度 (默认: 0.0，即不主动避障)
 * 
 * 返回:
 *   - SUCCESS: 参数设置成功
 *   - FAILURE: 参数设置失败
 */
class ToggleObstacleAvoidance : public BT::SyncActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name 节点名称
   * @param config 节点配置
   */
  ToggleObstacleAvoidance(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief 析构函数
   */
  ~ToggleObstacleAvoidance() override = default;

  /**
   * @brief 提供端口列表 (BT.CPP V4 必须实现)
   * @return 端口列表
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("enable", "是否开启避障功能"),
      BT::InputPort<std::string>("controller_server_name", "controller_server", 
        "Nav2 控制器服务器节点名称"),
      BT::InputPort<double>("critic_scale_enabled", 10.0, 
        "开启避障时的障碍物评价权重"),
      BT::InputPort<double>("critic_scale_disabled", 0.0, 
        "关闭避障时的障碍物评价权重"),
    };
  }

  /**
   * @brief 节点执行逻辑 (同步执行)
   * @return 节点状态
   */
  BT::NodeStatus tick() override;

private:
  /**
   * @brief 初始化 ROS 2 服务客户端
   */
  void initializeClient();

  /**
   * @brief 设置参数
   * @param enable 是否开启避障
   * @return 是否成功
   */
  bool setParameters(bool enable);

  // ROS 2 节点与服务客户端
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client_;

  // 配置
  std::string controller_server_name_;
  bool initialized_;

  // 超时与重试配置
  static constexpr auto kServiceTimeout = std::chrono::seconds(2);
  static constexpr int kMaxRetries = 3;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__TOGGLE_OBSTACLE_AVOIDANCE_HPP_
