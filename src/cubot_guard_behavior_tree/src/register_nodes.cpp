/**
 * @file register_nodes.cpp
 * @brief 统一的 BT 插件注册入口
 *
 * 所有自定义行为树节点在此统一注册。
 * BT_REGISTER_NODES 宏在整个共享库中只能出现一次，
 * 否则会产生多重定义链接错误。
 */

#include "behaviortree_cpp/bt_factory.h"

// 动作节点
#include "cubot_guard_behavior_tree/plugins/action/update.hpp"
#include "cubot_guard_behavior_tree/plugins/action/toggle_obstacle_avoidance.hpp"
#include "cubot_guard_behavior_tree/plugins/action/set_move_mode.hpp"
#include "cubot_guard_behavior_tree/plugins/action/set_stance.hpp"
#include "cubot_guard_behavior_tree/plugins/action/generate_random_pose.hpp"
#include "cubot_guard_behavior_tree/plugins/action/build_pose_array.hpp"

// 条件节点
#include "cubot_guard_behavior_tree/plugins/condition/check_hp.hpp"
#include "cubot_guard_behavior_tree/plugins/condition/check_ammo.hpp"
#include "cubot_guard_behavior_tree/plugins/condition/check_time.hpp"
#include "cubot_guard_behavior_tree/plugins/condition/check_game_started.hpp"
#include "cubot_guard_behavior_tree/plugins/condition/check_game_ended.hpp"
#include "cubot_guard_behavior_tree/plugins/condition/check_signal.hpp"
#include "cubot_guard_behavior_tree/plugins/condition/wait_rfid.hpp"
#include "cubot_guard_behavior_tree/plugins/condition/wait_for_revive.hpp"

// 动作节点
#include "cubot_guard_behavior_tree/plugins/action/wait_for_signal.hpp"

// 装饰器节点
#include "cubot_guard_behavior_tree/plugins/decorator/rate_controller.hpp"

// 检查黑板条目节点 - 用于 CheckBlackboardEntry
class CheckBlackboardEntry : public BT::ConditionNode
{
public:
  CheckBlackboardEntry(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("value", "黑板变量值")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string val;
    if (!getInput("value", val)) {
      return BT::NodeStatus::FAILURE;
    }
    // 支持多种 true 表示
    if (val == "true" || val == "1" || val == "yes" ||
        val == "True" || val == "TRUE") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cubot_guard_behavior_tree::Update>("Update");
  factory.registerNodeType<cubot_guard_behavior_tree::CheckHP>("CheckHP");
  factory.registerNodeType<cubot_guard_behavior_tree::CheckAmmo>("CheckAmmo");
  factory.registerNodeType<cubot_guard_behavior_tree::CheckTime>("CheckTime");
  factory.registerNodeType<cubot_guard_behavior_tree::CheckGameStarted>("CheckGameStarted");
  factory.registerNodeType<cubot_guard_behavior_tree::CheckGameEnded>("CheckGameEnded");
  factory.registerNodeType<cubot_guard_behavior_tree::WaitRFID>("WaitRFID");
  factory.registerNodeType<cubot_guard_behavior_tree::WaitForRevive>("WaitForRevive");
  factory.registerNodeType<cubot_guard_behavior_tree::ToggleObstacleAvoidance>(
    "ToggleObstacleAvoidance");
  factory.registerNodeType<cubot_guard_behavior_tree::SetArrive>(
    "SetArrive");
  factory.registerNodeType<cubot_guard_behavior_tree::SetStance>(
    "SetStance");
  factory.registerNodeType<cubot_guard_behavior_tree::GenerateRandomPose>(
    "GenerateRandomPose");
  factory.registerNodeType<cubot_guard_behavior_tree::BuildPoseArray>(
    "BuildPoseArray");
  factory.registerNodeType<cubot_guard_behavior_tree::CheckSignal>("CheckSignal");
  factory.registerNodeType<cubot_guard_behavior_tree::WaitForSignal>("WaitForSignal");
  factory.registerNodeType<cubot_guard_behavior_tree::RateController>("CubotRateController");
  factory.registerNodeType<CheckBlackboardEntry>("CheckBlackboardEntry");
}
