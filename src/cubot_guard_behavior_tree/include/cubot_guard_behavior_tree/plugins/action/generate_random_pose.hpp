/**
 * @file generate_random_pose.hpp
 * @brief 随机巡逻点生成节点 - 在控制区中心周围生成随机坐标
 *
 * 功能：
 * - 读取黑板中的 control_zone_center_pose
 * - 以其为中心, 在边长为 range 的正方形内生成随机 PoseStamped
 * - 将结果写入黑板 random_pose, 供 NavigateToPose 使用
 *
 * BT.CPP V4 兼容 - 使用 SyncActionNode
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_RANDOM_POSE_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_RANDOM_POSE_HPP_

#include <string>
#include <random>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cubot_guard_behavior_tree
{

/**
 * @brief GenerateRandomPose - 在中心点周围生成随机 PoseStamped
 *
 * 输入端口:
 *   - center: 中心坐标 (PoseStamped, 从黑板读取)
 *   - range: 随机范围, 即正方形边长 (默认: 2.0 米)
 *
 * 输出端口:
 *   - random_pose: 生成的随机坐标 (PoseStamped)
 *
 * 返回:
 *   - SUCCESS: 成功生成
 *   - FAILURE: 无法读取中心坐标
 */
class GenerateRandomPose : public BT::SyncActionNode
{
public:
  GenerateRandomPose(const std::string & name, const BT::NodeConfig & config);
  ~GenerateRandomPose() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("center", "中心坐标 (从黑板读取)"),
      BT::InputPort<double>("range", 2.0, "随机范围 - 正方形边长 (米)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("random_pose", "生成的随机坐标"),
    };
  }

  BT::NodeStatus tick() override;

private:
  std::mt19937 rng_;
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_RANDOM_POSE_HPP_
