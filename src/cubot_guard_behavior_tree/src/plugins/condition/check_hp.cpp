/**
 * @file check_hp.cpp
 * @brief 血量检测条件节点实现 (简化版, 纯黑板读取)
 *
 * 实现细节：
 * - 不订阅 ROS Topic, 直接从黑板读取 {hp}
 * - {hp} 由 Update 节点统一管理和刷新
 * - 简单阈值比较: hp <= threshold -> SUCCESS (需要撤退)
 */

#include "cubot_guard_behavior_tree/plugins/condition/check_hp.hpp"

namespace cubot_guard_behavior_tree
{

CheckHP::CheckHP(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckHP::tick()
{
  // 从黑板读取当前血量
  double hp = 100.0;
  if (!getInput("hp", hp)) {
    // 无法读取血量, 假设血量充足, 继续战斗
    return BT::NodeStatus::FAILURE;
  }

  // 读取阈值
  double threshold = 40.0;
  getInput("hp_threshold", threshold);

  // 判断是否需要撤退
  if (hp <= threshold) {
    return BT::NodeStatus::SUCCESS;  // 血量过低, 需要撤退
  }

  return BT::NodeStatus::FAILURE;  // 血量充足, 继续战斗
}

}  // namespace cubot_guard_behavior_tree


