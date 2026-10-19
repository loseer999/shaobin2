/**
 * @file check_hp.hpp
 * @brief 血量检测条件节点 - 从黑板读取血量并判断
 *
 * 功能：
 * - 从黑板读取 {hp} (由 Update 节点刷新)
 * - 当 hp <= 阈值时返回 SUCCESS, 触发保命逻辑
 * - 当 hp > 阈值时返回 FAILURE, 继续其他分支
 *
 * 注意: 此节点不直接订阅 ROS Topic, 所有数据由 Update 节点统一管理
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HP_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HP_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace cubot_guard_behavior_tree
{

/**
 * @brief CheckHP - 血量检测条件节点 (纯黑板读取)
 *
 * 输入端口:
 *   - hp: 当前血量 (从黑板读取, 0-100)
 *   - hp_threshold: 血量阈值 (默认: 40.0)
 *
 * 返回:
 *   - SUCCESS: hp <= 阈值, 需要撤退回血
 *   - FAILURE: hp > 阈值, 可以继续战斗
 */
class CheckHP : public BT::ConditionNode
{
public:
  CheckHP(const std::string & name, const BT::NodeConfig & config);
  ~CheckHP() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hp", "当前血量 (从黑板读取, 0-400)"),
      BT::InputPort<double>("hp_threshold", 40.0, "血量阈值, 低于或等于此值触发撤退"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HP_HPP_
