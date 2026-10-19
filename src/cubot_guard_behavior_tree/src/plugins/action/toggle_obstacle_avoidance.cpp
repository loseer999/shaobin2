/**
 * @file toggle_obstacle_avoidance.cpp
 * @brief 动态切换避障功能的同步动作节点实现
 * 
 * 实现细节：
 * - 通过设置 DWB 局部规划器的 ObstacleFootprint critic 权重来控制避障
 * - 权重为 0 时相当于关闭避障
 * - 包含服务调用超时和重试机制
 * 
 * 适配 Nav2 DWB 局部规划器参数：
 * - BaseObstacle.scale: 基础障碍物评价权重
 * - ObstacleFootprint.scale: 机器人足迹障碍物评价权重
 */

#include "cubot_guard_behavior_tree/plugins/action/toggle_obstacle_avoidance.hpp"

#include <thread>

namespace cubot_guard_behavior_tree
{

ToggleObstacleAvoidance::ToggleObstacleAvoidance(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config),
  initialized_(false)
{
}

void ToggleObstacleAvoidance::initializeClient()
{
  if (initialized_) {
    return;
  }

  // 获取控制器服务器名称
  if (!getInput("controller_server_name", controller_server_name_)) {
    controller_server_name_ = "controller_server";
  }

  // 创建 ROS 2 节点
  if (!node_) {
    node_ = rclcpp::Node::make_shared("toggle_obstacle_avoidance_bt_node");
  }

  // 构建参数服务名称
  std::string service_name = "/" + controller_server_name_ + "/set_parameters";

  // 创建参数服务客户端
  param_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(service_name);

  RCLCPP_INFO(
    node_->get_logger(),
    "[ToggleObstacleAvoidance] 已创建参数服务客户端: %s",
    service_name.c_str());

  initialized_ = true;
}

bool ToggleObstacleAvoidance::setParameters(bool enable)
{
  // 获取权重值
  double critic_scale_enabled = 10.0;
  double critic_scale_disabled = 0.0;
  getInput("critic_scale_enabled", critic_scale_enabled);
  getInput("critic_scale_disabled", critic_scale_disabled);

  double target_scale = enable ? critic_scale_enabled : critic_scale_disabled;

  // 等待服务可用
  for (int retry = 0; retry < kMaxRetries; ++retry) {
    if (param_client_->wait_for_service(kServiceTimeout)) {
      break;
    }
    
    RCLCPP_WARN(
      node_->get_logger(),
      "[ToggleObstacleAvoidance] 等待参数服务超时，重试 %d/%d",
      retry + 1, kMaxRetries);
    
    if (retry == kMaxRetries - 1) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[ToggleObstacleAvoidance] 参数服务不可用，放弃设置");
      return false;
    }
  }

  // 构建参数设置请求
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

  // 设置 DWB BaseObstacle critic 权重
  rcl_interfaces::msg::Parameter base_obstacle_param;
  base_obstacle_param.name = "FollowPath.BaseObstacle.scale";
  base_obstacle_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  base_obstacle_param.value.double_value = target_scale;
  request->parameters.push_back(base_obstacle_param);

  // 设置 DWB ObstacleFootprint critic 权重
  rcl_interfaces::msg::Parameter obstacle_footprint_param;
  obstacle_footprint_param.name = "FollowPath.ObstacleFootprint.scale";
  obstacle_footprint_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  obstacle_footprint_param.value.double_value = target_scale;
  request->parameters.push_back(obstacle_footprint_param);

  // 发送请求
  auto future = param_client_->async_send_request(request);

  // 等待响应
  auto status = future.wait_for(kServiceTimeout);
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[ToggleObstacleAvoidance] 参数设置请求超时");
    return false;
  }

  // 检查响应
  auto response = future.get();
  bool all_success = true;
  for (const auto & result : response->results) {
    if (!result.successful) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[ToggleObstacleAvoidance] 参数设置失败: %s",
        result.reason.c_str());
      all_success = false;
    }
  }

  if (all_success) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ToggleObstacleAvoidance] 避障功能已%s (scale=%.1f)",
      enable ? "开启" : "关闭",
      target_scale);
  }

  return all_success;
}

BT::NodeStatus ToggleObstacleAvoidance::tick()
{
  // 延迟初始化
  initializeClient();

  // 获取 enable 参数
  bool enable = true;
  if (!getInput("enable", enable)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[ToggleObstacleAvoidance] 未能获取 enable 参数");
    return BT::NodeStatus::FAILURE;
  }

  // 设置参数
  if (setParameters(enable)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    // 即使设置失败也返回 SUCCESS，避免阻塞行为树
    // 因为避障设置失败不应该阻止导航继续
    RCLCPP_WARN(
      node_->get_logger(),
      "[ToggleObstacleAvoidance] 参数设置失败，但继续执行");
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace cubot_guard_behavior_tree


