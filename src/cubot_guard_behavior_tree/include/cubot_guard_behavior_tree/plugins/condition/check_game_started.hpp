/**
 * @file check_game_started.hpp
 * @brief 比赛开始检测条件节点 - 从黑板读取 game_status 判断比赛是否开始
 *
 * 功能：
 * - 从黑板读取 {game_status} (由 Update 节点根据 game_time>0 推导)
 * - 当 game_status == target_status 时返回 SUCCESS, 比赛已开始
 * - 否则返回 FAILURE, 阻止战术循环执行
 * - 带记忆：一旦比赛开始过，后续始终返回 SUCCESS
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_GAME_STARTED_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_GAME_STARTED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace cubot_guard_behavior_tree
{

/**
 * @brief CheckGameStarted - 比赛阶段门控条件节点 (纯黑板读取)
 *
 * 输入端口:
 *   - game_status: 当前比赛状态 (从黑板读取, int, 1=比赛开始, 0=未开始)
 *   - target_status: 目标比赛状态 (默认: 1, 表示比赛开始)
 *
 * 返回:
 *   - SUCCESS: game_status == target_status, 比赛已开始
 *   - FAILURE: game_status != target_status, 比赛未开始
 */
class CheckGameStarted : public BT::ConditionNode
{
public:
  CheckGameStarted(const std::string & name, const BT::NodeConfig & config);
  ~CheckGameStarted() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("game_status", "当前比赛状态 (从黑板读取)"),
      BT::InputPort<int>("target_status", 1, "目标比赛状态 (默认 1 = 比赛开始)"),
    };
  }

  BT::NodeStatus tick() override;

private:
  bool game_started_;   // 记忆标志：是否收到过比赛开始信号
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_GAME_STARTED_HPP_
