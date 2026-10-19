/**
 * @file update.cpp
 * @brief 状态更新动作节点实现
 *
 * 实现细节：
 * - 订阅 6 个 ROS 2 Topic (hp, game_time, protect, ammo, defense, outpost_hp)
 * - 解析坐标字符串 (供给区, 中继点1/2, 驻守点, 大符打击点)
 * - 每次 tick 调用 spin_some 处理回调, 然后刷新黑板
 * - 始终返回 RUNNING, 保持 Parallel 分支活跃
 * - 比赛开始判断：game_time > 0 → game_status=1
 */

#include "cubot_guard_behavior_tree/plugins/action/update.hpp"

#include <sstream>
#include <fstream>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace cubot_guard_behavior_tree
{

// 简单的 YAML 解析函数
std::string parseYamlValue(const std::string& content, const std::string& key)
{
  size_t pos = content.find(key + ":");
  if (pos == std::string::npos) return "";
  
  size_t value_start = content.find('"', pos);
  if (value_start == std::string::npos) return "";
  
  size_t value_end = content.find('"', value_start + 1);
  if (value_end == std::string::npos) return "";
  
  return content.substr(value_start + 1, value_end - value_start - 1);
}

// 从 YAML 配置文件加载坐标
bool loadPosesFromYaml(std::string& supply, std::string& relay1, std::string& relay2,
                       std::string& relay3, std::string& relay4,
                       std::string& station, std::string& guard)
{
  try {
    std::string bt_pkg_dir =
      ament_index_cpp::get_package_share_directory("cubot_guard_behavior_tree");
    std::string config_file = bt_pkg_dir + "/config/behavior_tree_params.yaml";
    
    std::ifstream file(config_file);
    if (!file.is_open()) return false;
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();
    
    supply = parseYamlValue(content, "supply_zone_pose");
    relay1 = parseYamlValue(content, "relay_point1_pose");
    relay2 = parseYamlValue(content, "relay_point2_pose");
    relay3 = parseYamlValue(content, "relay_point3_pose");
    relay4 = parseYamlValue(content, "relay_point4_pose");
    station = parseYamlValue(content, "station_pose");
    guard = parseYamlValue(content, "guard_pose");
    
    return !supply.empty() && !relay1.empty() && !relay2.empty() && !station.empty();
  } catch (...) {
    return false;
  }
}

Update::Update(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

void Update::initializeNode()
{
  if (initialized_) {
    return;
  }

  // 创建 ROS 2 节点
  node_ = rclcpp::Node::make_shared("update_bt_node");

  // 获取 Topic 名称
  std::string hp_topic = "/referee/hp";
  std::string time_topic = "/referee/game_time";
  std::string protect_topic = "/referee/protect";
  std::string ammo_topic = "/referee/ammo";
  std::string defense_topic = "/referee/defense";
  std::string outpost_hp_topic = "/referee/outpost_hp";
  getInput("hp_topic", hp_topic);
  getInput("time_topic", time_topic);
  getInput("protect_topic", protect_topic);
  getInput("ammo_topic", ammo_topic);
  getInput("defense_topic", defense_topic);
  getInput("outpost_hp_topic", outpost_hp_topic);

  // 创建订阅者
  hp_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    hp_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&Update::hpCallback, this, std::placeholders::_1));

  time_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    time_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&Update::timeCallback, this, std::placeholders::_1));

  protect_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
    protect_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&Update::protectCallback, this, std::placeholders::_1));

  ammo_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    ammo_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&Update::ammoCallback, this, std::placeholders::_1));

  defense_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
    defense_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&Update::defenseCallback, this, std::placeholders::_1));

  outpost_hp_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    outpost_hp_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&Update::outpostHpCallback, this, std::placeholders::_1));

  // 解析坐标：优先从 YAML 配置文件加载，其次使用 XML 中的输入
  std::string supply_str, relay1_str, relay2_str, relay3_str, relay4_str, station_str, guard_str, rune_str, hero_guard_str, outpost_str;
  
  // 尝试从 YAML 加载
  bool yaml_loaded = loadPosesFromYaml(supply_str, relay1_str, relay2_str, relay3_str, relay4_str, station_str, guard_str);
  
  // 如果 YAML 加载失败，使用 XML 中的默认值
  if (!yaml_loaded) {
    supply_str = "-5.7;-0.4;0.0;0.0;0.0;0.0;1.0";
    relay1_str = "-6.3;-2.69;0.0;0.0;0.0;0.0;1.0";
    relay2_str = "-5.0;-3.0;0.0;0.0;0.0;0.0;1.0";
    relay3_str = "0.0;0.0;0.0;0.0;0.0;0.0;1.0";
    relay4_str = "0.0;0.0;0.0;0.0;0.0;0.0;1.0";
    station_str = "-5.6;-4.1;0.0;0.0;0.0;0.0;1.0";
    guard_str = "0.0;0.0;0.0;0.0;0.0;0.0;1.0";
    
    // 允许 XML 覆盖默认值
    getInput("supply_zone_pose_str", supply_str);
    getInput("relay_point1_pose_str", relay1_str);
    getInput("relay_point2_pose_str", relay2_str);
    getInput("relay_point3_pose_str", relay3_str);
    getInput("relay_point4_pose_str", relay4_str);
    getInput("station_pose_str", station_str);
    getInput("guard_pose_str", guard_str);
    
    RCLCPP_INFO(node_->get_logger(), "[Update] 使用 XML/默认值坐标");
  } else {
    // YAML 加载成功时，允许 XML 覆盖可选坐标（relay3/4、guard）
    std::string xml_val;
    if (getInput("guard_pose_str", xml_val) && !xml_val.empty()) {
      guard_str = xml_val;
    }
    if (getInput("relay_point3_pose_str", xml_val) && !xml_val.empty()) {
      relay3_str = xml_val;
    }
    if (getInput("relay_point4_pose_str", xml_val) && !xml_val.empty()) {
      relay4_str = xml_val;
    }
    // 保卫点在 YAML 中缺失时使用安全默认
    if (guard_str.empty()) {
      guard_str = "0.0;0.0;0.0;0.0;0.0;0.0;1.0";
    }
    RCLCPP_INFO(node_->get_logger(), "[Update] 使用 YAML 配置文件坐标");
  }
  
  // 打符点和英雄保护点始终从 XML 读取
  rune_str = "0.0;0.0;0.0;0.0;0.0;0.0;1.0";
  hero_guard_str = "0.0;0.0;0.0;0.0;0.0;0.0;1.0";
  getInput("rune_point_pose_str", rune_str);
  getInput("hero_guard_point_pose_str", hero_guard_str);
  getInput("outpost_point_pose_str", outpost_str);

  supply_zone_pose_ = parsePoseString(supply_str);
  relay_point1_pose_ = parsePoseString(relay1_str);
  relay_point2_pose_ = parsePoseString(relay2_str);
  relay_point3_pose_ = parsePoseString(relay3_str);
  relay_point4_pose_ = parsePoseString(relay4_str);
  station_pose_ = parsePoseString(station_str);
  guard_pose_ = parsePoseString(guard_str);
  rune_point_pose_ = parsePoseString(rune_str);
  hero_guard_point_pose_ = parsePoseString(hero_guard_str);
  outpost_point_pose_ = parsePoseString(outpost_str);

  RCLCPP_INFO(
    node_->get_logger(),
    "[Update] 已初始化: hp=%s, time=%s, protect=%s, ammo=%s, defense=%s, outpost_hp=%s",
    hp_topic.c_str(), time_topic.c_str(),
    protect_topic.c_str(), ammo_topic.c_str(),
    defense_topic.c_str(), outpost_hp_topic.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "[Update] 回复点: (%.2f, %.2f), 中继点1: (%.2f, %.2f), 中继点2: (%.2f, %.2f), 中继点3: (%.2f, %.2f), 中继点4: (%.2f, %.2f), 驻守点: (%.2f, %.2f), 保卫点: (%.2f, %.2f), 打符点: (%.2f, %.2f), 英雄保护点: (%.2f, %.2f)",
    supply_zone_pose_.pose.position.x, supply_zone_pose_.pose.position.y,
    relay_point1_pose_.pose.position.x, relay_point1_pose_.pose.position.y,
    relay_point2_pose_.pose.position.x, relay_point2_pose_.pose.position.y,
    relay_point3_pose_.pose.position.x, relay_point3_pose_.pose.position.y,
    relay_point4_pose_.pose.position.x, relay_point4_pose_.pose.position.y,
    station_pose_.pose.position.x, station_pose_.pose.position.y,
    guard_pose_.pose.position.x, guard_pose_.pose.position.y,
    rune_point_pose_.pose.position.x, rune_point_pose_.pose.position.y,
    hero_guard_point_pose_.pose.position.x, hero_guard_point_pose_.pose.position.y);

  initialized_ = true;
}

geometry_msgs::msg::PoseStamped Update::parsePoseString(const std::string & str) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";

  // 默认朝向
  pose.pose.orientation.w = 1.0;

  std::istringstream ss(str);
  char delim;
  ss >> pose.pose.position.x >> delim
     >> pose.pose.position.y >> delim
     >> pose.pose.position.z >> delim
     >> pose.pose.orientation.x >> delim
     >> pose.pose.orientation.y >> delim
     >> pose.pose.orientation.z >> delim
     >> pose.pose.orientation.w;

  return pose;
}

void Update::hpCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  double new_hp = msg->data;
  double current_hp = hp_.load();
  
  // HP 有效性校验：只接受 0-400 范围内的值
  if (new_hp < 0.0 || new_hp > 400.0) {
    RCLCPP_DEBUG(node_->get_logger(),
      "[Update] 忽略无效 HP 值: %.1f (超出 0-400 范围)", new_hp);
    return;
  }
  
  // 跳变过滤：如果 HP 变化超过 150 且不是死亡/满血场景，忽略
  double delta = std::abs(new_hp - current_hp);
  if (delta > 150.0) {
    // 允许的大跳变：死亡(HP->0) 或 满血(HP->400)
    bool is_death = (new_hp == 0.0);
    bool is_full_heal = (new_hp == 400.0);
    bool is_revive = (current_hp == 0.0 && new_hp > 0.0);
    
    if (!is_death && !is_full_heal && !is_revive) {
      RCLCPP_DEBUG(node_->get_logger(),
        "[Update] 忽略 HP 异常跳变: %.1f -> %.1f (变化 %.1f)",
        current_hp, new_hp, delta);
      return;
    }
  }
  
  hp_.store(new_hp);
}

void Update::timeCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  ltime_.store(msg->data);
}

void Update::protectCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  uint8_t old_val = protect_.exchange(msg->data);
  if (old_val != msg->data) {
    RCLCPP_INFO(node_->get_logger(),
      "[Update] 保护标志变更: %u -> %u", old_val, msg->data);
  }
}

void Update::ammoCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  ammo_.store(msg->data);
}

void Update::defenseCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  uint8_t old_val = defense_.exchange(msg->data);
  if (old_val != msg->data) {
    RCLCPP_INFO(node_->get_logger(),
      "[Update] 防守标志变更: %u -> %u (0=无,1=保护泉水,2=保护英雄)", old_val, msg->data);
  }
}

void Update::outpostHpCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  outpost_hp_.store(msg->data);
}

void Update::flushToBlackboard()
{
  setOutput("hp", hp_.load());
  setOutput("ltime", ltime_.load());
  setOutput("protect", static_cast<double>(protect_.load()));
  setOutput("ammo", ammo_.load());
  setOutput("defense", static_cast<double>(defense_.load()));
  setOutput("outpost_hp", outpost_hp_.load());
  setOutput("supply_zone_pose", supply_zone_pose_);
  setOutput("relay_point1_pose", relay_point1_pose_);
  setOutput("relay_point2_pose", relay_point2_pose_);
  setOutput("relay_point3_pose", relay_point3_pose_);
  setOutput("relay_point4_pose", relay_point4_pose_);
  setOutput("station_pose", station_pose_);
  setOutput("guard_pose", guard_pose_);
  setOutput("rune_point_pose", rune_point_pose_);
  setOutput("hero_guard_point_pose", hero_guard_point_pose_);
  setOutput("outpost_point_pose", outpost_point_pose_);

  // 定期输出调试信息（每100次tick约10秒输出一次）
  static int flush_count = 0;
  if (++flush_count % 100 == 0) {
    RCLCPP_INFO(node_->get_logger(),
      "[Update] 黑板状态: hp=%.1f, ammo=%.1f, game_time=%.1f, protect=%d, defense=%d, outpost_hp=%.0f",
      hp_.load(), ammo_.load(), ltime_.load(), protect_.load(), defense_.load(), outpost_hp_.load());
  }
}

BT::NodeStatus Update::onStart()
{
  initializeNode();
  flushToBlackboard();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Update::onRunning()
{
  // 多次 spin_some 确保队列中的回调全部处理完
  if (node_) {
    for (int i = 0; i < 5; ++i) {
      rclcpp::spin_some(node_);
    }
  }

  // 刷新黑板
  flushToBlackboard();

  // 始终返回 RUNNING, 保持全天候更新
  return BT::NodeStatus::RUNNING;
}

void Update::onHalted()
{
  if (node_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[Update] 状态更新已停止");
  }
}

}  // namespace cubot_guard_behavior_tree


