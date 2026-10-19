/**
 * @file set_stance.cpp
 * @brief 发布姿态指令到 /stance topic 实现
 */

#include "cubot_guard_behavior_tree/plugins/action/set_stance.hpp"

namespace cubot_guard_behavior_tree
{

SetStance::SetStance(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config),
  initialized_(false)
{
}

void SetStance::initializePublisher()
{
  if (initialized_) {
    return;
  }

  std::string topic_name;
  if (!getInput("topic_name", topic_name)) {
    topic_name = "/stance";
  }

  if (!node_) {
    node_ = rclcpp::Node::make_shared("set_stance_bt_node");
  }

  auto qos = rclcpp::QoS(1).reliable().transient_local();
  pub_ = node_->create_publisher<std_msgs::msg::UInt8>(topic_name, qos);

  RCLCPP_INFO(
    node_->get_logger(),
    "[SetStance] 已创建 publisher: %s",
    topic_name.c_str());

  initialized_ = true;
}

BT::NodeStatus SetStance::tick()
{
  initializePublisher();

  int stance = 1;
  if (!getInput("stance", stance)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[SetStance] 未能获取 stance 参数");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(stance);
  pub_->publish(msg);

  const char* mode = (stance == 1) ? "进攻" : (stance == 2) ? "防御" : "移动";
  RCLCPP_INFO(
    node_->get_logger(),
    "[SetStance] 已发布 stance=%d (%s)",
    stance, mode);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace cubot_guard_behavior_tree
