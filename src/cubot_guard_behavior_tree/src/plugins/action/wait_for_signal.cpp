/**
 * @file wait_for_signal.cpp
 * @brief 等待信号变化动作节点实现
 */

#include "cubot_guard_behavior_tree/plugins/action/wait_for_signal.hpp"

namespace cubot_guard_behavior_tree
{

WaitForSignal::WaitForSignal(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus WaitForSignal::onStart()
{
  // 读取参数
  getInput("timeout", timeout_ms_);
  getInput("target_value", target_value_);
  
  // 记录开始时间
  start_time_ = std::chrono::steady_clock::now();
  initialized_ = true;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForSignal::onRunning()
{
  // 检查是否超时
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
  
  if (elapsed >= timeout_ms_) {
    // 超时
    return BT::NodeStatus::FAILURE;
  }
  
  // 读取当前信号值
  int signal_value = 0;
  if (!getInput("signal", signal_value)) {
    // 无法读取，继续等待
    return BT::NodeStatus::RUNNING;
  }
  
  // 检查是否达到目标值
  if (signal_value == target_value_) {
    return BT::NodeStatus::SUCCESS;
  }
  
  // 继续等待
  return BT::NodeStatus::RUNNING;
}

void WaitForSignal::onHalted()
{
  initialized_ = false;
}

}  // namespace cubot_guard_behavior_tree
