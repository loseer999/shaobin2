/**
 * @file check_ammo.cpp
 * @brief 低弹量告警检测条件节点实现 (下位机已将弹量二值化)
 *
 * 新协议语义:
 *   {ammo} = 1  → 下位机判断弹量<20，需撤退补弹
 *   {ammo} = 0  → 弹量≥20，充足
 *
 * 行为:
 *   - ammo >= 1  -> SUCCESS (触发撤退)
 *   - ammo == 0  -> FAILURE (充足)
 * ammo_threshold 端口保留仅为兼容旧 XML，不再参与判断。
 */

#include "cubot_guard_behavior_tree/plugins/condition/check_ammo.hpp"

namespace cubot_guard_behavior_tree
{

CheckAmmo::CheckAmmo(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckAmmo::tick()
{
  // 从黑板读取告警位 (下位机二值化结果)
  double ammo = 0.0;
  if (!getInput("ammo", ammo)) {
    // 无法读取保守策略: 不触发撤退
    return BT::NodeStatus::FAILURE;
  }

  // 新语义: 告警位置 1 即撤退 (下位机已判断 <20)
  if (ammo >= 1.0) {
    return BT::NodeStatus::SUCCESS;  // 弹量不足，需撤退补弹
  }

  return BT::NodeStatus::FAILURE;  // 弹量充足
}

}  // namespace cubot_guard_behavior_tree
