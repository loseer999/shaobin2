/**
 * @file set_move_mode.cpp
 * @brief 置位到达标志位 arrive 的同步动作节点实现
 *
 * 通过 /arrive topic (std_msgs::msg::UInt8) 向串口节点发送到达标志位。
 * QoS 使用 reliable + transient_local, 确保串口节点晚启动也能收到最后一条消息。
 */

#include "cubot_guard_behavior_tree/plugins/action/set_move_mode.hpp"

namespace cubot_guard_behavior_tree
{

SetArrive::SetArrive(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config),
  initialized_(false)
{
}

void SetArrive::initializePublisher()
{
  if (initialized_) {
    return;
  }

  // 获取 topic 名称
  std::string topic_name;
  if (!getInput("topic_name", topic_name)) {
    topic_name = "/arrive";
  }

  // 创建 ROS 2 节点
  if (!node_) {
    node_ = rclcpp::Node::make_shared("set_arrive_bt_node");
  }

  // 创建 publisher, transient_local 保证 latched 语义
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  pub_ = node_->create_publisher<std_msgs::msg::UInt8>(topic_name, qos);

  RCLCPP_INFO(
    node_->get_logger(),
    "[SetArrive] 已创建 publisher: %s",
    topic_name.c_str());

  initialized_ = true;
}

BT::NodeStatus SetArrive::tick()
{
  initializePublisher();

  int arrive = 0;
  if (!getInput("arrive", arrive)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[SetArrive] 未能获取 arrive 参数");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(arrive);
  pub_->publish(msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "[SetArrive] 已发布 arrive=%d (%s)",
    arrive, arrive == 1 ? "已到达" : "导航中");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace cubot_guard_behavior_tree
