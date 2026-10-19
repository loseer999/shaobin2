/**
 * @file wait_for_revive.hpp
 * @brief 等待机器人复活的条件节点
 *
 * 检测 HP 是否从 0 恢复到大于 0，用于处理击晕后的复活等待。
 * 支持递增等待策略：第1次5秒，第2次10秒，第3次15秒...
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__WAIT_FOR_REVIVE_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__WAIT_FOR_REVIVE_HPP_

#include <string>
#include <atomic>
#include <chrono>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace cubot_guard_behavior_tree
{

/**
 * @brief WaitForRevive - 等待机器人从击晕状态复活
 *
 * 输入端口:
 *   - hp: 当前血量 (从黑板读取)
 *   - check_interval: 检测间隔 (默认 1000ms)
 *   - base_wait_time: 基础等待时间 (默认 5秒)
 *   - max_wait_time: 最大等待时间 (默认 30秒)
 *
 * 返回:
 *   - SUCCESS: HP > 0，已复活
 *   - FAILURE: 超时仍未复活
 */
class WaitForRevive : public BT::StatefulActionNode
{
public:
  WaitForRevive(const std::string & name, const BT::NodeConfig & config);

  ~WaitForRevive() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hp", "当前血量"),
      BT::InputPort<int>("check_interval", 1000, "检测间隔 (ms)"),
      BT::InputPort<int>("base_wait_time", 5000, "基础等待时间 (ms)"),
      BT::InputPort<int>("max_wait_time", 30000, "最大等待时间 (ms)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  
  // 配置参数
  int check_interval_ms_;
  int base_wait_time_ms_;
  int max_wait_time_ms_;
  
  // 状态
  std::chrono::steady_clock::time_point start_time_;
  bool initialized_;
  
  // 静态死亡计数器（跨实例共享）
  static std::atomic<int> death_count_;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__WAIT_FOR_REVIVE_HPP_
