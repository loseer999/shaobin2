/**
 * @file wait_rfid.cpp
 * @brief RFID 等待节点实现
 *
 * 实现细节：
 * - 从黑板读取 {rfid} 状态 (由 Update 节点持续刷新)
 * - 使用 StatefulActionNode 实现阻塞等待
 * - onStart: 立即检查, rfid=1 则直接 SUCCESS
 * - onRunning: 持续检查, 带超时保护
 * - 超时后返回 FAILURE, 行为树可选择重新导航
 */

#include "cubot_guard_behavior_tree/plugins/condition/wait_rfid.hpp"

namespace cubot_guard_behavior_tree
{

WaitRFID::WaitRFID(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus WaitRFID::onStart()
{
  // 读取超时参数
  getInput("timeout_ms", timeout_ms_);

  // 记录开始时间
  start_time_ = std::chrono::steady_clock::now();

  // 立即检查 RFID 状态
  int rfid = 0;
  if (getInput("rfid", rfid) && rfid == 1) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitRFID::onRunning()
{
  // 检查 RFID 状态 (由 Update 节点在 Parallel 分支中持续刷新)
  int rfid = 0;
  if (getInput("rfid", rfid) && rfid == 1) {
    return BT::NodeStatus::SUCCESS;
  }

  // 检查超时
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time_);

  if (elapsed.count() >= timeout_ms_) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitRFID::onHalted()
{
  // 被更高优先级分支打断时无需特殊处理
}

}  // namespace cubot_guard_behavior_tree


