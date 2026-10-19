/**
 * @file check_signal.cpp
 * @brief 通用信号检测条件节点实现
 */

#include "cubot_guard_behavior_tree/plugins/condition/check_signal.hpp"

namespace cubot_guard_behavior_tree
{

CheckSignal::CheckSignal(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckSignal::tick()
{
  // 从黑板读取信号值
  double signal_value = 0;
  if (!getInput("signal", signal_value)) {
    // 无法读取，返回FAILURE
    return BT::NodeStatus::FAILURE;
  }

  // 读取目标值
  double target_value = 1;
  getInput("target_value", target_value);

  // 检查是否相等
  if (signal_value == target_value) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace cubot_guard_behavior_tree
