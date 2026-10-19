/**
 * @file check_time.cpp
 * @brief 时间检测条件节点实现
 *
 * 实现细节：
 * - 不订阅 ROS Topic, 直接从黑板读取 {ltime}
 * - {ltime} 由 Update 节点统一管理和刷新
 * - 简单阈值比较: ltime <= threshold -> SUCCESS
 */

#include "cubot_guard_behavior_tree/plugins/condition/check_time.hpp"

namespace cubot_guard_behavior_tree
{

CheckTime::CheckTime(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckTime::tick()
{
  double time = 420.0;
  if (!getInput("time", time)) {
    return BT::NodeStatus::FAILURE;
  }

  double threshold = 345.0;
  getInput("time_threshold", threshold);

  if (time <= threshold) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace cubot_guard_behavior_tree
