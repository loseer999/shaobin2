/**
 * @file mission_executor.cpp
 * @brief 任务级行为树执行器 - 加载并运行 sentinel_main_tree.xml
 *
 * 独立 ROS 2 节点, 运行任务级行为树 (外层决策):
 * - 加载自定义 BT 插件 (cubot_guard_bt_plugins) 和 Nav2 BT 插件
 * - 配置黑板 (NavigateToPose BT 节点所需的 node/timeout 等条目)
 * - 以固定频率 tick 行为树主循环
 *
 * 架构层级:
 *   mission_executor (本节点, 外层)
 *     └── NavigateToPose BT 节点 ──action──> bt_navigator (Nav2, 内层)
 *           └── ComputePath + FollowPath + Recovery
 */

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <fstream>
#include <vector>
#include <cstdio>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// 简单的 YAML 解析函数，提取坐标值
std::string parse_yaml_value(const std::string& content, const std::string& key) {
  size_t pos = content.find(key + ":");
  if (pos == std::string::npos) return "";
  
  size_t value_start = content.find('"', pos);
  if (value_start == std::string::npos) return "";
  
  size_t value_end = content.find('"', value_start + 1);
  if (value_end == std::string::npos) return "";
  
  return content.substr(value_start + 1, value_end - value_start - 1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("mission_executor");

  // game_status 记忆状态（与 CheckGameStarted 节点保持一致）
  bool game_started_memory = false;

  // ---- 加载 YAML 配置文件 ----
  std::string bt_config_file;
  std::string supply_pose_from_yaml, control_pose_from_yaml, relay1_pose_from_yaml, relay2_pose_from_yaml;
  try {
    std::string bt_pkg_dir =
      ament_index_cpp::get_package_share_directory("cubot_guard_behavior_tree");
    bt_config_file = bt_pkg_dir + "/config/behavior_tree_params.yaml";
    
    // 读取 YAML 文件内容
    std::ifstream file(bt_config_file);
    if (file.is_open()) {
      std::string content((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());
      file.close();
      
      // 解析坐标
      supply_pose_from_yaml = parse_yaml_value(content, "supply_zone_pose");
      control_pose_from_yaml = parse_yaml_value(content, "control_zone_pose");
      relay1_pose_from_yaml = parse_yaml_value(content, "relay_point1_pose");
      relay2_pose_from_yaml = parse_yaml_value(content, "relay_point2_pose");
      
      RCLCPP_INFO(node->get_logger(),
        "[MissionExecutor] 已加载配置文件: %s", bt_config_file.c_str());
      RCLCPP_INFO(node->get_logger(),
        "[MissionExecutor] 坐标: 补给%s 控制%s 中继1%s 中继2%s",
        supply_pose_from_yaml.c_str(),
        control_pose_from_yaml.c_str(),
        relay1_pose_from_yaml.c_str(),
        relay2_pose_from_yaml.c_str());
    } else {
      RCLCPP_WARN(node->get_logger(),
        "[MissionExecutor] 无法打开配置文件: %s, 将使用 XML 中的默认值", bt_config_file.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(),
      "[MissionExecutor] 加载配置文件失败: %s, 将使用 XML 中的默认值", e.what());
  }

  // ---- 参数声明 ----
  std::string default_bt_xml;
  try {
    std::string bt_pkg_dir =
      ament_index_cpp::get_package_share_directory("cubot_guard_behavior_tree");
    default_bt_xml = bt_pkg_dir + "/behavior_trees/sentinel_main_tree.xml";
  } catch (const std::exception &) {
    default_bt_xml = "";
  }

  auto bt_xml_filename = node->declare_parameter<std::string>(
    "bt_xml_filename", default_bt_xml);
  auto bt_loop_duration = node->declare_parameter<int>(
    "bt_loop_duration", 50);  // 20Hz tick，提高响应速度
  auto server_timeout = node->declare_parameter<int>(
    "server_timeout", 20000);
  auto wait_for_service_timeout = node->declare_parameter<int>(
    "wait_for_service_timeout", 10000);
  auto max_tree_creation_retries = node->declare_parameter<int>(
    "max_tree_creation_retries", 10);
  auto tree_creation_retry_interval = node->declare_parameter<int>(
    "tree_creation_retry_interval", 2000);
  auto enable_bt_logger = node->declare_parameter<bool>(
    "enable_bt_logger", false);  // 默认禁用行为树详细日志

  if (bt_xml_filename.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "[MissionExecutor] bt_xml_filename 为空, 无法加载行为树");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
    "[MissionExecutor] 行为树 XML: %s", bt_xml_filename.c_str());
  RCLCPP_INFO(node->get_logger(),
    "[MissionExecutor] tick 周期: %ld ms, server 超时: %ld ms",
    bt_loop_duration, server_timeout);

  // ---- 初始化: 置位到达标志位 arrive=0 (导航中) ----
  // 使用 transient_local QoS 确保串口节点能收到初始化指令
  auto arrive_pub = node->create_publisher<std_msgs::msg::UInt8>(
    "/arrive", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::UInt8 init_msg;
  init_msg.data = 0;  // 0 = 导航中 (初始状态视为未到达)
  arrive_pub->publish(init_msg);
  RCLCPP_INFO(node->get_logger(),
    "[MissionExecutor] 已发送初始化到达标志位: arrive=0 (导航中)");

  // ---- 创建 BT 工厂并加载插件 ----
  BT::BehaviorTreeFactory factory;

  // 加载自定义 BT 插件
  try {
    factory.registerFromPlugin(
      BT::SharedLibrary::getOSName("cubot_guard_bt_plugins"));
    RCLCPP_INFO(node->get_logger(),
      "[MissionExecutor] 已加载自定义插件: cubot_guard_bt_plugins");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "[MissionExecutor] 加载 cubot_guard_bt_plugins 失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // 加载 Nav2 BT 节点插件
  std::vector<std::string> nav2_plugins = {
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_back_up_action_bt_node"
  };
  
  for (const auto & plugin : nav2_plugins) {
    try {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
      RCLCPP_INFO(node->get_logger(),
        "[MissionExecutor] 已加载 Nav2 插件: %s", plugin.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(),
        "[MissionExecutor] 加载 %s 失败: %s", plugin.c_str(), e.what());
      rclcpp::shutdown();
      return 1;
    }
  }

  // ---- 配置黑板 ----
  // NavigateToPose BT 节点 (nav2_behavior_tree::BtActionNode) 构造时需要以下条目
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::chrono::milliseconds>(
    "server_timeout", std::chrono::milliseconds(server_timeout));
  blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration", std::chrono::milliseconds(bt_loop_duration));
  blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(wait_for_service_timeout));

  // ---- 创建行为树 (带重试, 等待 Nav2 action server 就绪) ----
  BT::Tree tree;
  std::unique_ptr<BT::StdCoutLogger> console_logger;
  bool tree_created = false;
  for (int attempt = 1; attempt <= max_tree_creation_retries && rclcpp::ok(); ++attempt) {
    try {
      tree = factory.createTreeFromFile(bt_xml_filename, blackboard);
      RCLCPP_INFO(node->get_logger(),
        "[MissionExecutor] 行为树创建成功");
      // 根据参数决定是否启用行为树详细日志
      if (enable_bt_logger) {
        console_logger = std::make_unique<BT::StdCoutLogger>(tree);
        RCLCPP_INFO(node->get_logger(),
          "[MissionExecutor] 行为树详细日志已启用");
      }
      tree_created = true;
      break;
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(),
        "[MissionExecutor] 创建行为树失败 (第 %d/%ld 次): %s",
        attempt, max_tree_creation_retries, e.what());
      if (attempt < max_tree_creation_retries) {
        RCLCPP_INFO(node->get_logger(),
          "[MissionExecutor] %ld ms 后重试...", tree_creation_retry_interval);
        std::this_thread::sleep_for(
          std::chrono::milliseconds(tree_creation_retry_interval));
      }
    }
  }

  if (!tree_created) {
    RCLCPP_ERROR(node->get_logger(),
      "[MissionExecutor] 达到最大重试次数 (%ld), 退出", max_tree_creation_retries);
    rclcpp::shutdown();
    return 1;
  }

  // ---- 主循环: tick 行为树 ----
  rclcpp::WallRate rate{std::chrono::milliseconds(bt_loop_duration)};

  // 短暂休眠确保初始化消息被发送
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(node->get_logger(),
    "[MissionExecutor] 开始 tick 主循环 (频率: %.1f Hz)",
    1000.0 / bt_loop_duration);
  RCLCPP_INFO(node->get_logger(),
    "[MissionExecutor] 提示: 如需查看详细行为树日志，请设置环境变量: export RCUTILS_LOGGING_SEVERITY=DEBUG");

  // 调试计数器
  int tick_count = 0;
  const int log_interval = 50;  // 每50个tick强制输出一次 (约2.5秒@20Hz)
  
  // 当前执行的行为描述（用于日志）
  std::string current_action = "初始化";
  std::string last_logged_action = "";
  int action_unchanged_count = 0;  // 行为未变化计数

  while (rclcpp::ok()) {
    // 处理 ROS 2 回调 (NavigateToPose action client 需要)
    rclcpp::spin_some(node);

    // tick 行为树
    BT::NodeStatus status = tree.tickOnce();
    tick_count++;
    
    // 根据黑板状态推断当前执行的行为
    auto blackboard = tree.rootBlackboard();
    double hp = 0.0;
    double ltime = 0.0;
    bool has_hp = blackboard->get("hp", hp);
    bool has_ltime = blackboard->get("ltime", ltime);

    // 调试：检查黑板读取是否成功
    if (!has_hp || !has_ltime) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
        "[MissionExecutor] 黑板读取失败: has_hp=%d, has_ltime=%d",
        has_hp, has_ltime);
    }

    // 比赛开始判定: ltime > 0.5 (由串口接收 game_time>0 推导)
    if (has_ltime && ltime > 0.5 && !game_started_memory) {
      game_started_memory = true;
    }
    
    // 从黑板读取坐标（PoseStamped 类型）
    geometry_msgs::msg::PoseStamped supply_pose, control_pose, relay1_pose, relay2_pose;
    bool has_supply = blackboard->get("supply_zone_pose", supply_pose);
    bool has_control = blackboard->get("station_pose", control_pose);
    bool has_relay1 = blackboard->get("relay_point1_pose", relay1_pose);
    bool has_relay2 = blackboard->get("relay_point2_pose", relay2_pose);
    
    // 格式化坐标为字符串 (x,y)
    auto format_xy = [](const geometry_msgs::msg::PoseStamped& pose) -> std::string {
      char buf[64];
      snprintf(buf, sizeof(buf), "(%.2f,%.2f)", 
               pose.pose.position.x, pose.pose.position.y);
      return std::string(buf);
    };
    
    std::string supply_xy = has_supply ? format_xy(supply_pose) : "(?,?)";
    std::string control_xy = has_control ? format_xy(control_pose) : "(?,?)";
    std::string relay1_xy = has_relay1 ? format_xy(relay1_pose) : "(?,?)";
    std::string relay2_xy = has_relay2 ? format_xy(relay2_pose) : "(?,?)";
    
    // 从黑板读取当前阶段（如果存在）
    std::string current_stage;
    bool has_stage = blackboard->get("current_stage", current_stage);
    
    // 判断当前执行的行为（完全基于 current_stage，不依赖 HP 值）
    std::string new_action;
    if (!game_started_memory) {
      new_action = "【等待】等待比赛开始信号";
    } else if (has_stage && current_stage.rfind("retreat_", 0) == 0) {
      // 撤退阶段 - 根据 stage 前缀判断
      if (current_stage == "retreat_relay2") {
        new_action = "【撤退】移动小陀螺-正在前往中继点2" + relay2_xy;
      } else if (current_stage == "retreat_relay1") {
        new_action = "【撤退】移动小陀螺-正在前往中继点1" + relay1_xy;
      } else if (current_stage == "retreat_supply") {
        new_action = "【撤退】正常模式-正在前往补给区" + supply_xy;
      } else {
        new_action = "【撤退】正在撤退";
      }
    } else if (has_stage && current_stage.rfind("attack_", 0) == 0) {
      // 进攻阶段 - 根据 stage 前缀判断
      if (current_stage == "attack_relay1") {
        new_action = "【进攻】正常模式-正在前往中继点1" + relay1_xy;
      } else if (current_stage == "attack_relay2") {
        new_action = "【进攻】移动小陀螺-正在前往中继点2" + relay2_xy;
      } else if (current_stage == "attack_control") {
        new_action = "【进攻】移动小陀螺-正在前往控制区" + control_xy;
      } else {
        new_action = "【进攻】正在进攻";
      }
    } else {
      // 未知阶段，根据 HP 推断
      if (hp <= 150) {
        new_action = "【撤退】血量低-正在撤退";
      } else {
        new_action = "【进攻】正在执行任务";
      }
    }
    
    // 检测行为是否变化
    bool action_changed = (new_action != current_action);
    if (action_changed) {
      current_action = new_action;
      action_unchanged_count = 0;
    } else {
      action_unchanged_count++;
    }

    // 输出日志条件：1)行为变化 2)达到强制输出间隔
    bool should_log = action_changed || (tick_count % log_interval == 0);
    
    if (should_log) {
      RCLCPP_INFO(node->get_logger(),
        "[MissionExecutor] %s | HP:%.0f | ltime:%.0f | stage:%s",
        current_action.c_str(), hp, ltime,
        has_stage ? current_stage.c_str() : "none");
      last_logged_action = current_action;
    }

    // 安全防护: sentinel_main_tree 有 Repeat(-1), 正常情况下始终 RUNNING
    if (status == BT::NodeStatus::SUCCESS ||
        status == BT::NodeStatus::FAILURE)
    {
      RCLCPP_WARN(node->get_logger(),
        "[MissionExecutor] 行为树返回 %s, 重置并继续",
        (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
      tree.haltTree();
    }

    rate.sleep();
  }

  // ---- 退出清理 ----
  tree.haltTree();
  RCLCPP_INFO(node->get_logger(), "[MissionExecutor] 已退出");
  rclcpp::shutdown();
  return 0;
}
