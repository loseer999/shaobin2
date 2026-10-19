/**
 * @file check_signal.hpp
 * @brief 通用信号检测条件节点
 *
 * 功能：
 * - 从黑板读取信号值
 * - 检查是否等于目标值
 * - 每次tick重新评估（无记忆）
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_SIGNAL_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_SIGNAL_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace cubot_guard_behavior_tree
{

class CheckSignal : public BT::ConditionNode
{
public:
  CheckSignal(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("signal", "信号值（从黑板读取）"),
      BT::InputPort<double>("target_value", "目标值，相等时返回SUCCESS")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_SIGNAL_HPP_
