/**
 * @file rate_controller.hpp
 * @brief 频率控制装饰器节点 - 限制子节点的 tick 频率
 *
 * 功能：
 * - 控制子节点的执行频率，防止高频重复调用
 * - 典型用途：包裹 NavigateToPose，防止高频重发导航目标导致 Nav2 异常
 * - 子节点 RUNNING 状态下不受频率限制，持续 tick
 * - 仅在子节点完成(SUCCESS/FAILURE)后，等待下一个周期才重新 tick
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_

#include <string>
#include <chrono>

#include "behaviortree_cpp/decorator_node.h"

namespace cubot_guard_behavior_tree
{

class RateController : public BT::DecoratorNode
{
public:
  RateController(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hz", 1.0, "子节点最大 tick 频率 (Hz)"),
    };
  }

private:
  BT::NodeStatus tick() override;

  std::chrono::high_resolution_clock::time_point start_;
  double period_;       // 周期（秒）
  bool first_time_;     // 首次 tick 标志
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
