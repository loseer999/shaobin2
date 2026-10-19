/**
 * @file set_stance.hpp
 * @brief 发布姿态指令到 /stance topic
 *
 * 通过 ROS 2 Topic 向串口节点发布 stance 姿态指令，
 * 下位机根据 stance 切换进攻/防御/移动模式。
 *
 * BT.CPP V4 兼容
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__SET_STANCE_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__SET_STANCE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace cubot_guard_behavior_tree
{

/**
 * @brief SetStance - 发布姿态指令到 /stance topic
 *
 * 输入端口:
 *   - stance: 姿态指令 (1:进攻,2:防御,3:移动, 4:强化进攻,5:强化防御,6:强化移动)
 *   - topic_name: 发布的 topic 名称 (默认: /stance)
 *
 * 返回:
 *   - SUCCESS: 消息发布成功
 *   - FAILURE: 无法读取 stance 参数
 */
class SetStance : public BT::SyncActionNode
{
public:
  SetStance(const std::string & name, const BT::NodeConfig & config);

  ~SetStance() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("stance", "姿态指令: 1=进攻, 2=防御, 3=移动"),
      BT::InputPort<std::string>("topic_name", "/stance",
        "发布姿态指令的 topic 名称"),
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

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__SET_STANCE_HPP_
