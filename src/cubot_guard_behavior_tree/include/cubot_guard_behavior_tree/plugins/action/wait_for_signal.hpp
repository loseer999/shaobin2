/**
 * @file wait_for_signal.hpp
 * @brief 等待信号变化动作节点
 *
 * 功能：
 * - 阻塞等待信号变为目标值
 * - 带超时机制（毫秒）
 * - 超时返回FAILURE，成功返回SUCCESS
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_SIGNAL_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_SIGNAL_HPP_

#include <string>
#include <chrono>
#include "behaviortree_cpp/action_node.h"

namespace cubot_guard_behavior_tree
{

class WaitForSignal : public BT::StatefulActionNode
{
public:
  WaitForSignal(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("signal", "信号值（从黑板读取）"),
      BT::InputPort<int>("target_value", "目标值，等待信号变为该值"),
      BT::InputPort<int>("timeout", 30000, "超时时间（毫秒），默认30秒")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  int timeout_ms_ = 30000;
  int target_value_ = 1;
  bool initialized_ = false;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_SIGNAL_HPP_
