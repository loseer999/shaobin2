/**
 * @file check_ammo.hpp
 * @brief 剩余弹量检测条件节点 - 从黑板读取 ammo 并判断
 *
 * 功能：
 * - 从黑板读取 {ammo} (由 Update 节点刷新)
 * - 当 ammo < 阈值时返回 SUCCESS, 触发撤退补弹逻辑
 * - 当 ammo >= 阈值时返回 FAILURE
 *
 * 注意: 此节点不直接订阅 ROS Topic, 所有数据由 Update 节点统一管理
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_AMMO_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_AMMO_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace cubot_guard_behavior_tree
{

/**
 * @brief CheckAmmo - 低弹量告警条件节点 (0/1标志位, 由 MCU 下位机判断)
 *
 * 输入端口:
 *   - ammo: 低弹量告警标志 (0=弹量充足, 1=弹量<20 需撤退)
 *   - ammo_threshold: 保留兼容, 不再参与判断
 *
 * 返回:
 *   - SUCCESS: ammo >= 1.0, 弹量不足需撤退补弹
 *   - FAILURE: ammo < 1.0, 弹量充足
 */
class CheckAmmo : public BT::ConditionNode
{
public:
  CheckAmmo(const std::string & name, const BT::NodeConfig & config);
  ~CheckAmmo() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("ammo", "当前剩余弹量 (从黑板读取)"),
      BT::InputPort<double>("ammo_threshold", 20.0, "弹量阈值, 低于此值触发撤退"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_AMMO_HPP_
