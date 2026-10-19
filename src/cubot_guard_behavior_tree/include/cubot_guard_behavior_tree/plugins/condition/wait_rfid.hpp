/**
 * @file wait_rfid.hpp
 * @brief RFID 等待节点 - 阻塞直到黑板中 rfid 为 true
 *
 * 功能：
 * - 从黑板读取 {rfid} 状态 (由 Update 节点刷新)
 * - rfid == 1 时返回 SUCCESS
 * - 超时返回 FAILURE
 * - 等待中返回 RUNNING
 *
 * BT.CPP V4 兼容 - 使用 StatefulActionNode
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__WAIT_RFID_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__WAIT_RFID_HPP_

#include <string>
#include <chrono>

#include "behaviortree_cpp/action_node.h"

namespace cubot_guard_behavior_tree
{

/**
 * @brief WaitRFID - 阻塞等待 RFID 踩点确认
 *
 * 输入端口:
 *   - rfid: RFID 状态 (从黑板读取, 0/1)
 *   - timeout_ms: 等待超时时间 (毫秒, 默认: 5000)
 *
 * 返回:
 *   - SUCCESS: rfid == 1, 已确认踩到补给区
 *   - RUNNING: 正在等待
 *   - FAILURE: 等待超时
 */
class WaitRFID : public BT::StatefulActionNode
{
public:
  WaitRFID(const std::string & name, const BT::NodeConfig & config);
  ~WaitRFID() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("rfid", "RFID 状态 (从黑板读取, 0/1)"),
      BT::InputPort<int>("timeout_ms", 5000, "等待超时时间 (毫秒)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  int timeout_ms_{5000};
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__CONDITION__WAIT_RFID_HPP_
