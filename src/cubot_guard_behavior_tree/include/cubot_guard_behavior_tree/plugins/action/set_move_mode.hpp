/**
 * @file set_move_mode.hpp
 * @brief 置位到达标志位 arrive 的同步动作节点
 *
 * 通过 ROS 2 Topic 向串口节点发布 arrive 标志位，
 * 通知下位机当前是否到达目标点（0 = 导航中 / 1 = 已到达）。
 *
 * 注意：小陀螺模式由下位机自行控制，本节点不再负责切换运动模式。
 *
 * 战术用途:
 * - 导航开始时置 arrive=0；到达目标点后置 arrive=1
 * - 下位机根据 arrive 决定小陀螺/驻守/状态灯等附带行为
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__SET_MOVE_MODE_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__SET_MOVE_MODE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace cubot_guard_behavior_tree
{

/**
 * @brief SetArrive - 发布到达标志位到 /arrive topic
 *
 * 输入端口:
 *   - arrive: 到达标志位（0 = 导航中 / 1 = 已到达目标点）
 *   - topic_name: 发布的 topic 名称 (默认: /arrive)
 *
 * 返回:
 *   - SUCCESS: 消息发布成功
 *   - FAILURE: 无法读取 arrive 参数
 */
class SetArrive : public BT::SyncActionNode
{
public:
  SetArrive(const std::string & name, const BT::NodeConfig & config);

  ~SetArrive() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("arrive", "到达标志位: 0=导航中, 1=已到达"),
      BT::InputPort<std::string>("topic_name", "/arrive",
        "发布到达标志位的 topic 名称"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void initializePublisher();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_;
  bool initialized_;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__SET_MOVE_MODE_HPP_
