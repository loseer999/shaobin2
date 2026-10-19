/**
 * @file generate_random_pose.cpp
 * @brief 随机巡逻点生成节点实现
 *
 * 实现细节：
 * - 从黑板读取中心坐标 (control_zone_center_pose)
 * - 使用 C++ <random> 库生成均匀分布的随机偏移
 * - 偏移范围: 以中心为原点, 边长为 range 的正方形内
 * - 保持中心坐标的 z 值和朝向不变
 */

#include "cubot_guard_behavior_tree/plugins/action/generate_random_pose.hpp"

namespace cubot_guard_behavior_tree
{

GenerateRandomPose::GenerateRandomPose(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config),
  rng_(std::random_device{}())
{
}

BT::NodeStatus GenerateRandomPose::tick()
{
  // 读取中心坐标
  geometry_msgs::msg::PoseStamped center;
  if (!getInput("center", center)) {
    return BT::NodeStatus::FAILURE;
  }

  // 读取范围参数
  double range = 2.0;
  getInput("range", range);

  // 在 [-range/2, +range/2] 范围内生成随机偏移
  double half = range / 2.0;
  std::uniform_real_distribution<double> dist(-half, half);

  // 生成随机坐标: 中心 + 偏移
  geometry_msgs::msg::PoseStamped random_pose = center;
  random_pose.pose.position.x += dist(rng_);
  random_pose.pose.position.y += dist(rng_);

  // 写入黑板
  setOutput("random_pose", random_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace cubot_guard_behavior_tree


