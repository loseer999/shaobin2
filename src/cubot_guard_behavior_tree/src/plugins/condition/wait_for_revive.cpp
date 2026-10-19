/**
 * @file wait_for_revive.cpp
 * @brief 等待机器人复活的条件节点实现
 *
 * 检测 HP 是否从 0 恢复到大于 0，用于处理击晕后的复活等待。
 */

#include "cubot_guard_behavior_tree/plugins/condition/wait_for_revive.hpp"

#include <thread>
#include <chrono>

namespace cubot_guard_behavior_tree
{

// 初始化静态死亡计数器
std::atomic<int> WaitForRevive::death_count_(0);

WaitForRevive::WaitForRevive(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  initialized_(false)
{
}

BT::NodeStatus WaitForRevive::onStart()
{
  // 获取参数
  if (!getInput("check_interval", check_interval_ms_)) {
    check_interval_ms_ = 1000;  // 默认 1 秒
  }
  if (!getInput("base_wait_time", base_wait_time_ms_)) {
    base_wait_time_ms_ = 5000;  // 默认 5 秒
  }
  if (!getInput("max_wait_time", max_wait_time_ms_)) {
    max_wait_time_ms_ = 30000;  // 默认 30 秒
  }

  // 记录开始时间
  start_time_ = std::chrono::steady_clock::now();
  
  // 创建 ROS 节点（用于日志）
  if (!node_) {
    node_ = rclcpp::Node::make_shared("wait_for_revive_bt_node");
  }

  // 获取当前 HP
  double hp = 0.0;
  if (!getInput("hp", hp)) {
    RCLCPP_ERROR(node_->get_logger(),
      "[WaitForRevive] 无法获取 hp 参数");
    return BT::NodeStatus::FAILURE;
  }

  // 如果 HP 已经 > 0，直接返回成功
  if (hp > 0.0) {
    RCLCPP_INFO(node_->get_logger(),
      "[WaitForRevive] HP=%.1f > 0，无需等待", hp);
    return BT::NodeStatus::SUCCESS;
  }

  // HP == 0，增加死亡计数
  int current_death = death_count_.fetch_add(1) + 1;
  
  // 计算本次需要等待的时间：第1次5秒，第2次10秒，第3次15秒...
  int wait_time_ms = std::min(base_wait_time_ms_ * current_death, max_wait_time_ms_);
  
  RCLCPP_INFO(node_->get_logger(),
    "[WaitForRevive] HP=0，第%d次死亡，等待%d秒后检查复活...",
    current_death, wait_time_ms / 1000);

  initialized_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForRevive::onRunning()
{
  // 获取当前 HP
  double hp = 0.0;
  if (!getInput("hp", hp)) {
    RCLCPP_ERROR(node_->get_logger(),
      "[WaitForRevive] 无法获取 hp 参数");
    return BT::NodeStatus::FAILURE;
  }

  // 检查是否已复活
  if (hp > 0.0) {
    RCLCPP_INFO(node_->get_logger(),
      "[WaitForRevive] HP 已恢复到 %.1f，复活成功！", hp);
    return BT::NodeStatus::SUCCESS;
  }

  // 检查是否超时
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - start_time_).count();
  
  int current_death = death_count_.load();
  int expected_wait = std::min(base_wait_time_ms_ * current_death, max_wait_time_ms_);
  
  if (elapsed >= expected_wait) {
    // 已经等待了足够时间，检查 HP
    if (hp > 0.0) {
      RCLCPP_INFO(node_->get_logger(),
        "[WaitForRevive] 等待%d秒后，HP 恢复到 %.1f，复活成功！",
        static_cast<int>(elapsed / 1000), hp);
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "[WaitForRevive] 等待%d秒后 HP 仍为 %.1f，可能复活失败",
        static_cast<int>(elapsed / 1000), hp);
      // 继续等待，直到 HP > 0 或达到最大等待时间
      if (elapsed >= max_wait_time_ms_) {
        RCLCPP_ERROR(node_->get_logger(),
          "[WaitForRevive] 达到最大等待时间 %d 秒，返回失败",
          max_wait_time_ms_ / 1000);
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  // 继续等待
  std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms_));
  return BT::NodeStatus::RUNNING;
}

void WaitForRevive::onHalted()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
      "[WaitForRevive] 等待复活被中断");
  }
}

}  // namespace cubot_guard_behavior_tree
