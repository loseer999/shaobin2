/**
 * @file check_time.hpp
 * @brief 时间检测条件节点 - 从黑板读取剩余时间并判断
 *
 * 功能：
 * - 从黑板读取 {ltime} (由 Update 节点刷新)
 * - 当 ltime <= 阈值时返回 SUCCESS, 触发时间相关逻辑
 * - 当 ltime > 阈值时返回 FAILURE
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TIME_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TIME_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace cubot_guard_behavior_tree
{

class CheckTime : public BT::ConditionNode
{
public:
  CheckTime(const std::string & name, const BT::NodeConfig & config);
  ~CheckTime() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("time", "当前剩余时间 (从黑板读取)"),
      BT::InputPort<double>("time_threshold", 345.0, "时间阈值, 低于或等于此值触发"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TIME_HPP_
