/**
 * @file rate_controller.cpp
 * @brief 频率控制装饰器节点实现
 *
 * 逻辑：
 * - 首次 tick 立即执行子节点
 * - 子节点 RUNNING 时持续 tick，不受频率限制
 * - 子节点完成(SUCCESS/FAILURE)后，等待一个周期再重新 tick
 * - 周期内返回上一次的状态
 */

#include "cubot_guard_behavior_tree/plugins/decorator/rate_controller.hpp"

namespace cubot_guard_behavior_tree
{

RateController::RateController(const std::string & name, const BT::NodeConfig & config)
: BT::DecoratorNode(name, config),
  period_(1.0),
  first_time_(true)
{
  double hz = 1.0;
  getInput("hz", hz);
  if (hz <= 0.0) {
    hz = 1.0;
  }
  period_ = 1.0 / hz;
}

BT::NodeStatus RateController::tick()
{
  // 首次 tick：重置计时器，立即执行子节点
  if (status() == BT::NodeStatus::IDLE) {
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // 子节点正在运行：持续 tick，不受频率限制
  if (child_node_->status() == BT::NodeStatus::RUNNING) {
    const BT::NodeStatus child_state = child_node_->executeTick();
    return child_state;
  }

  // 子节点未在运行：检查是否到达下一个周期
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration<double>(now - start_).count();

  if (first_time_ || elapsed >= period_) {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::SUCCESS:
        start_ = std::chrono::high_resolution_clock::now();  // 重置计时器
        return BT::NodeStatus::SUCCESS;
      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  // 周期内：保持当前状态
  return status();
}

}  // namespace cubot_guard_behavior_tree
