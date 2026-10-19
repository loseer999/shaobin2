/**
 * @file check_game_ended.cpp
 * @brief 比赛结束检测条件节点实现
 *
 * 实现细节：
 * - 不订阅 ROS Topic, 直接从黑板读取 {game_status}
 * - {game_status} 由 Update 节点统一管理和刷新
 * - 简单相等比较: game_status == 5 -> SUCCESS (比赛已结束)
 */

#include "cubot_guard_behavior_tree/plugins/condition/check_game_ended.hpp"

namespace cubot_guard_behavior_tree
{

CheckGameEnded::CheckGameEnded(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckGameEnded::tick()
{
  // 从黑板读取当前比赛阶段
  int game_status = 0;
  if (!getInput("game_status", game_status)) {
    // 无法读取比赛阶段, 视为未结束
    return BT::NodeStatus::FAILURE;
  }

  // 255 表示无效/未初始化状态，视为未结束
  if (game_status == 255) {
    return BT::NodeStatus::FAILURE;
  }

  // 判断比赛是否已结束 (status == 5)
  if (game_status == 5) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace cubot_guard_behavior_tree
