/**
 * @file build_pose_array.hpp
 * @brief 将多个 PoseStamped 组合成数组的同步动作节点
 *
 * 用于 NavigateThroughPoses 的 goals 输入。
 * 将行为树黑板上的多个单点 pose 组合成 PoseStamped 数组。
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__BUILD_POSE_ARRAY_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__BUILD_POSE_ARRAY_HPP_

#include <vector>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cubot_guard_behavior_tree
{

/**
 * @brief BuildPoseArray - 将多个 PoseStamped 组合成数组
 *
 * 输入端口:
 *   - pose1: 第一个航点 (PoseStamped)
 *   - pose2: 第二个航点 (PoseStamped)
 *   - pose3: 第三个航点 (PoseStamped，可选)
 *   - pose4: 第四个航点 (PoseStamped，可选)
 *   - pose5: 第五个航点 (PoseStamped，可选)
 *
 * 输出端口:
 *   - goals: PoseStamped 数组 (std::vector<geometry_msgs::msg::PoseStamped>)
 *
 * 返回:
 *   - SUCCESS: 数组构建成功
 *   - FAILURE: 无法读取 pose1
 */
class BuildPoseArray : public BT::SyncActionNode
{
public:
  BuildPoseArray(const std::string & name, const BT::NodeConfig & config);

  ~BuildPoseArray() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose1", "第一个航点"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose2", "第二个航点"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose3", "第三个航点 (可选)"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose4", "第四个航点 (可选)"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose5", "第五个航点 (可选)"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals", "航点数组"),
    };
  }

  BT::NodeStatus tick() override;

private:
  bool addPoseIfValid(
    const std::string & port_name,
    std::vector<geometry_msgs::msg::PoseStamped> & goals);
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__BUILD_POSE_ARRAY_HPP_
