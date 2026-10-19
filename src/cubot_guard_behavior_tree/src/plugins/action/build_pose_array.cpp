/**
 * @file build_pose_array.cpp
 * @brief 将多个 PoseStamped 组合成数组的同步动作节点实现
 */

#include "cubot_guard_behavior_tree/plugins/action/build_pose_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace cubot_guard_behavior_tree
{

BuildPoseArray::BuildPoseArray(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

bool BuildPoseArray::addPoseIfValid(
  const std::string & port_name,
  std::vector<geometry_msgs::msg::PoseStamped> & goals)
{
  geometry_msgs::msg::PoseStamped pose;
  if (getInput(port_name, pose)) {
    // 检查 pose 是否有效（非零位置或已设置 frame_id）
    if (!pose.header.frame_id.empty()) {
      goals.push_back(pose);
      return true;
    }
  }
  return false;
}

BT::NodeStatus BuildPoseArray::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;

  // pose1 是必填项
  geometry_msgs::msg::PoseStamped pose1;
  if (!getInput("pose1", pose1)) {
    RCLCPP_ERROR(rclcpp::get_logger("BuildPoseArray"),
      "[BuildPoseArray] 无法读取 pose1");
    return BT::NodeStatus::FAILURE;
  }
  goals.push_back(pose1);

  // pose2-5 是可选的
  addPoseIfValid("pose2", goals);
  addPoseIfValid("pose3", goals);
  addPoseIfValid("pose4", goals);
  addPoseIfValid("pose5", goals);

  if (goals.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("BuildPoseArray"),
      "[BuildPoseArray] 警告: 只有 %zu 个航点，NavigateThroughPoses 至少需要 2 个",
      goals.size());
  }

  RCLCPP_INFO(rclcpp::get_logger("BuildPoseArray"),
    "[BuildPoseArray] 已构建航点数组，共 %zu 个航点", goals.size());

  // 输出到黑板
  setOutput("goals", goals);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace cubot_guard_behavior_tree
