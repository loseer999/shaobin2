/**
 * @file check_game_started.cpp
 * @brief 比赛开始检测条件节点实现
 *
 * 实现细节：
 * - 不订阅 ROS Topic, 直接从黑板读取 {game_status}
 * - {game_status} 由 Update 节点根据 game_time>0 推导 (1=开始, 0=未开始)
 * - 带记忆：一旦比赛开始过，后续始终返回 SUCCESS
 */

#include "cubot_guard_behavior_tree/plugins/condition/check_game_started.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cubot_guard_behavior_tree
{

CheckGameStarted::CheckGameStarted(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config),
  game_started_(false)
{
}

BT::NodeStatus CheckGameStarted::tick()
{
  // 记忆逻辑：一旦比赛开始过，后续始终返回 SUCCESS
  if (game_started_) {
    return BT::NodeStatus::SUCCESS;
  }

  // 从黑板读取当前比赛状态
  int game_status = 0;
  if (!getInput("game_status", game_status)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckGameStarted"),
      "[CheckGameStarted] 无法读取 game_status");
    return BT::NodeStatus::FAILURE;
  }

  int target_status = 1;
  getInput("target_status", target_status);

  if (game_status == target_status) {
    game_started_ = true;
    RCLCPP_INFO(rclcpp::get_logger("CheckGameStarted"),
      "[CheckGameStarted] 比赛开始: game_status=%d", game_status);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace cubot_guard_behavior_tree
