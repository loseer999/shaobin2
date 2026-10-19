/**
 * @file update.hpp
 * @brief 状态更新动作节点 - 持续将 ROS 2 数据刷入黑板
 *
 * 功能：
 * - 订阅裁判系统 Topic (hp, game_time, protect, ammo, defense, outpost_hp)
 * - 解析补给区/控制区坐标参数
 * - 每次 tick 将最新数据写入黑板供其他节点读取
 * - 始终返回 RUNNING, 保持 Parallel 分支活跃
 * - 比赛开始判断：game_time > 0
 *
 * BT.CPP V4 兼容 - 使用 StatefulActionNode
 */

#ifndef CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_HPP_
#define CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_HPP_

#include <string>
#include <memory>
#include <atomic>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cubot_guard_behavior_tree
{

/**
 * @brief Update - 状态更新节点, 持续将 ROS 2 数据刷入黑板
 *
 * 输入端口 (配置):
 *   - hp_topic: 血量 Topic (默认: /referee/hp, Float64)
 *   - time_topic: 比赛时间 Topic (默认: /referee/game_time, Float64)
 *   - protect_topic: 保护标志 Topic (默认: /referee/protect, UInt8)
 *   - ammo_topic: 剩余弹量 Topic (默认: /referee/ammo, Float64)
 *   - defense_topic: 防守标志 Topic (默认: /referee/defense, UInt8; 0=无, 1=保护泉水, 2=保护英雄)
 *   - outpost_hp_topic: 前哨站存活状态 Topic (默认: /referee/outpost_hp, Float64; 0=死亡,1=存活)
 *   - supply_zone_pose_str: 回复点/补给区坐标 "x;y;z;qx;qy;qz;qw"
 *   - control_zone_pose_str: 控制区坐标 "x;y;z;qx;qy;qz;qw"
 *   - station_pose_str: 驻守点坐标 "x;y;z;qx;qy;qz;qw"
 *   - fortress_pose_str: 堡垒点坐标 "x;y;z;qx;qy;qz;qw"
 *
 * 输出端口 (写入黑板):
 *   - hp: 当前血量 (0-400)
 *   - ltime: 比赛剩余时间 (0-420)
 *   - protect: 保护标志位 (0/1，初始 0)
 *   - ammo: 当前剩余弹量 (初始 0)
 *   - defense: 防守标志位 (0=无, 1=保护泉水, 2=保护英雄)
 *   - outpost_hp: 前哨站存活状态 (0=死亡, 1=存活)
 *   - supply_zone_pose: 回复点/补给区坐标 PoseStamped
 *   - relay_point1_pose: 中继点1坐标 PoseStamped
 *   - relay_point2_pose: 中继点2坐标 PoseStamped
 *   - relay_point3_pose: 中继点3坐标 PoseStamped
 *   - relay_point4_pose: 中继点4坐标 PoseStamped
 *   - station_pose: 驻守点坐标 PoseStamped
 *   - fortress_pose: 堡垒点坐标 PoseStamped
 *
 * 返回: 始终 RUNNING
 */
class Update : public BT::StatefulActionNode
{
public:
  Update(const std::string & name, const BT::NodeConfig & config);
  ~Update() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      // 输入: ROS Topic 配置
      BT::InputPort<std::string>("hp_topic", "/referee/hp", "血量 Topic"),
      BT::InputPort<std::string>("time_topic", "/referee/game_time", "比赛时间 Topic"),
      BT::InputPort<std::string>("protect_topic", "/referee/protect", "保护标志 Topic"),
      BT::InputPort<std::string>("ammo_topic", "/referee/ammo", "剩余弹量 Topic"),
      BT::InputPort<std::string>("defense_topic", "/referee/defense", "防守标志 Topic (0=无,1=保护泉水,2=保护英雄)"),
      BT::InputPort<std::string>("outpost_hp_topic", "/referee/outpost_hp", "前哨站存活状态 Topic (0=死亡,1=存活)"),
      BT::InputPort<std::string>(
        "supply_zone_pose_str", "-2.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "回复点/补给区坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "relay_point1_pose_str", "2.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "中继点1坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "relay_point2_pose_str", "1.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "中继点2坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "relay_point3_pose_str", "0.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "中继点3坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "relay_point4_pose_str", "0.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "中继点4坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "station_pose_str", "0.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "驻守点坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "guard_pose_str", "0.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "保卫点坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "rune_point_pose_str", "0.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "打符点坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "hero_guard_point_pose_str", "0.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "英雄保护点坐标 (x;y;z;qx;qy;qz;qw)"),
      BT::InputPort<std::string>(
        "outpost_point_pose_str", "0.0;0.0;0.0;0.0;0.0;0.0;1.0",
        "前哨点坐标 (x;y;z;qx;qy;qz;qw)"),

      // 输出: 写入黑板
      BT::OutputPort<double>("hp", "当前血量 (0-400)"),
      BT::OutputPort<double>("ltime", "比赛剩余时间 (0-420)"),
      BT::OutputPort<double>("protect", "保护标志位 (0/1，初始 0)"),
      BT::OutputPort<double>("ammo", "当前剩余弹量"),
      BT::OutputPort<double>("defense", "防守标志位 (0=无,1=保护泉水,2=保护英雄)"),
      BT::OutputPort<double>("outpost_hp", "前哨站存活状态 (0=死亡,1=存活)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "supply_zone_pose", "回复点/补给区坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "relay_point1_pose", "中继点1坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "relay_point2_pose", "中继点2坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "relay_point3_pose", "中继点3坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "relay_point4_pose", "中继点4坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "station_pose", "驻守点坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "guard_pose", "保卫点坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "rune_point_pose", "打符点坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "hero_guard_point_pose", "英雄保护点坐标"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "outpost_point_pose", "前哨点坐标"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void initializeNode();

  /**
   * @brief 解析坐标字符串为 PoseStamped
   * @param str 格式: "x;y;z;qx;qy;qz;qw"
   * @return 解析后的 PoseStamped (frame_id = "map")
   */
  geometry_msgs::msg::PoseStamped parsePoseString(const std::string & str) const;

  /** @brief 将最新数据写入黑板 */
  void flushToBlackboard();

  // ROS 2 回调
  void hpCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void timeCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void protectCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void ammoCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void defenseCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void outpostHpCallback(const std_msgs::msg::Float64::SharedPtr msg);

  // ROS 2 节点与订阅者
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr time_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr protect_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ammo_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr defense_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr outpost_hp_sub_;

  // 缓存数据 (原子变量保证线程安全)
  std::atomic<double> hp_{400.0};
  std::atomic<double> ltime_{0.0};
  std::atomic<uint8_t> protect_{0};  // 保护标志位，初始 0
  std::atomic<double> ammo_{0.0};  // 剩余弹量，初始 0
  std::atomic<uint8_t> defense_{0};   // 防守标志位 (0=无,1=保护泉水,2=保护英雄)
  std::atomic<double> outpost_hp_{0.0};  // 前哨站存活状态 (0=死亡,1=存活)

  // 静态坐标 (初始化后不变，设置默认值防止未定义行为)
  geometry_msgs::msg::PoseStamped supply_zone_pose_{};
  geometry_msgs::msg::PoseStamped relay_point1_pose_{};
  geometry_msgs::msg::PoseStamped relay_point2_pose_{};
  geometry_msgs::msg::PoseStamped relay_point3_pose_{};
  geometry_msgs::msg::PoseStamped relay_point4_pose_{};
  geometry_msgs::msg::PoseStamped station_pose_{};
  geometry_msgs::msg::PoseStamped guard_pose_{};
  geometry_msgs::msg::PoseStamped rune_point_pose_{};
  geometry_msgs::msg::PoseStamped hero_guard_point_pose_{};
  geometry_msgs::msg::PoseStamped outpost_point_pose_{};

  bool initialized_{false};
};

}  // namespace cubot_guard_behavior_tree

#endif  // CUBOT_GUARD_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_HPP_
