/**
 * @file check_game_ended.hpp
 * @brief 比赛结束检测条件节点 - 检测 game_status 是否为 5（比赛结束）
 *
 * 功能：
 * - 从黑板读取 {game_status}
 * - 当 game_status == 5 时返回 SUCCESS（比赛已结束）
 * - 否则返回 FAILURE
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_GAME_ENDED_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_GAME_ENDED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace cubot_guard_behavior_tree
{

/**
 * @brief CheckGameEnded - 比赛结束检测条件节点
 *
 * 输入端口:
 *   - game_status: 当前比赛阶段 (从黑板读取, int)
 *
 * 返回:
 *   - SUCCESS: game_status == 5, 比赛已结束
 *   - FAILURE: game_status != 5, 比赛未结束
 */
class CheckGameEnded : public BT::ConditionNode
{
public:
  CheckGameEnded(const std::string & name, const BT::NodeConfig & config);
  ~CheckGameEnded() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("game_status", "当前比赛阶段 (从黑板读取)"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_GAME_ENDED_HPP_
