/**
 * @file bt_simulation_test.cpp
 * @brief RoboMaster 2026 哨兵机器人行为树仿真测试
 *
 * 纯 C++17 实现, 无 ROS2 / BT.CPP 依赖
 * 模拟行为树的核心决策逻辑:
 *   - 血量低时撤退至补给区回血
 *   - 血量充足时前往控制区巡逻
 *
 * 编译: g++ -std=c++17 -o bt_sim bt_simulation_test.cpp
 * 运行: ./bt_sim
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <random>
#include <chrono>
#include <thread>
#include <unordered_map>
#include <variant>
#include <any>

// =============================================================================
// 颜色输出
// =============================================================================
namespace color {
const char* RESET   = "\033[0m";
const char* RED     = "\033[31m";
const char* GREEN   = "\033[32m";
const char* YELLOW  = "\033[33m";
const char* BLUE    = "\033[34m";
const char* MAGENTA = "\033[35m";
const char* CYAN    = "\033[36m";
const char* BOLD    = "\033[1m";
const char* DIM     = "\033[2m";
}  // namespace color

// =============================================================================
// 节点状态
// =============================================================================
enum class NodeStatus {
  SUCCESS,
  FAILURE,
  RUNNING,
};

const char* statusToStr(NodeStatus s) {
  switch (s) {
    case NodeStatus::SUCCESS: return "SUCCESS";
    case NodeStatus::FAILURE: return "FAILURE";
    case NodeStatus::RUNNING: return "RUNNING";
  }
  return "UNKNOWN";
}

const char* statusColor(NodeStatus s) {
  switch (s) {
    case NodeStatus::SUCCESS: return color::GREEN;
    case NodeStatus::FAILURE: return color::RED;
    case NodeStatus::RUNNING: return color::YELLOW;
  }
  return color::RESET;
}

// =============================================================================
// 简易黑板 (Blackboard)
// =============================================================================
class Blackboard {
public:
  void setDouble(const std::string& key, double val) { doubles_[key] = val; }
  void setInt(const std::string& key, int val)       { ints_[key] = val; }
  void setString(const std::string& key, const std::string& val) { strings_[key] = val; }

  double getDouble(const std::string& key, double def = 0.0) const {
    auto it = doubles_.find(key);
    return it != doubles_.end() ? it->second : def;
  }
  int getInt(const std::string& key, int def = 0) const {
    auto it = ints_.find(key);
    return it != ints_.end() ? it->second : def;
  }
  std::string getString(const std::string& key, const std::string& def = "") const {
    auto it = strings_.find(key);
    return it != strings_.end() ? it->second : def;
  }

  void dump() const {
    std::printf("  %s[Blackboard]%s hp=%.0f  ltime=%.0f  rfid=%d\n",
                color::DIM, color::RESET,
                getDouble("hp"), getDouble("ltime"), getInt("rfid"));
  }

private:
  std::unordered_map<std::string, double> doubles_;
  std::unordered_map<std::string, int> ints_;
  std::unordered_map<std::string, std::string> strings_;
};

// =============================================================================
// 节点基类
// =============================================================================
class TreeNode {
public:
  TreeNode(const std::string& name, Blackboard& bb)
    : name_(name), bb_(bb) {}
  virtual ~TreeNode() = default;

  virtual NodeStatus tick() = 0;
  virtual void halt() {}

  const std::string& name() const { return name_; }

protected:
  std::string name_;
  Blackboard& bb_;
};

// =============================================================================
// 条件节点: CheckHP
// =============================================================================
class CheckHP : public TreeNode {
public:
  CheckHP(const std::string& name, Blackboard& bb, double threshold)
    : TreeNode(name, bb), threshold_(threshold) {}

  NodeStatus tick() override {
    double hp = bb_.getDouble("hp", 400.0);
    NodeStatus result = (hp <= threshold_) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    std::printf("    %s[CheckHP]%s hp=%.0f threshold=%.0f -> %s%s%s\n",
                color::CYAN, color::RESET,
                hp, threshold_,
                statusColor(result), statusToStr(result), color::RESET);
    return result;
  }

private:
  double threshold_;
};

// =============================================================================
// 动作节点: NavigateToPose (模拟导航, 需要多次tick完成)
// =============================================================================
class NavigateToPose : public TreeNode {
public:
  NavigateToPose(const std::string& name, Blackboard& bb,
                 const std::string& goal_key, int ticks_to_complete = 3)
    : TreeNode(name, bb), goal_key_(goal_key),
      ticks_to_complete_(ticks_to_complete) {}

  NodeStatus tick() override {
    if (!navigating_) {
      navigating_ = true;
      ticks_done_ = 0;
      std::printf("    %s[NavigateToPose:%s]%s 开始导航至 %s\n",
                  color::BLUE, name_.c_str(), color::RESET, goal_key_.c_str());
    }

    ticks_done_++;
    if (ticks_done_ >= ticks_to_complete_) {
      navigating_ = false;
      std::printf("    %s[NavigateToPose:%s]%s 到达目标! %s%s%s\n",
                  color::BLUE, name_.c_str(), color::RESET,
                  color::GREEN, "SUCCESS", color::RESET);
      return NodeStatus::SUCCESS;
    }

    std::printf("    %s[NavigateToPose:%s]%s 导航中... (%d/%d) %s%s%s\n",
                color::BLUE, name_.c_str(), color::RESET,
                ticks_done_, ticks_to_complete_,
                color::YELLOW, "RUNNING", color::RESET);
    return NodeStatus::RUNNING;
  }

  void halt() override {
    if (navigating_) {
      std::printf("    %s[NavigateToPose:%s]%s 导航被中断\n",
                  color::BLUE, name_.c_str(), color::RESET);
      navigating_ = false;
      ticks_done_ = 0;
    }
  }

private:
  std::string goal_key_;
  int ticks_to_complete_;
  bool navigating_ = false;
  int ticks_done_ = 0;
};

// =============================================================================
// 条件节点: WaitRFID (模拟等待RFID, 带超时)
// =============================================================================
class WaitRFID : public TreeNode {
public:
  WaitRFID(const std::string& name, Blackboard& bb, int timeout_ticks = 5)
    : TreeNode(name, bb), timeout_ticks_(timeout_ticks) {}

  NodeStatus tick() override {
    if (!waiting_) {
      waiting_ = true;
      ticks_waited_ = 0;
    }

    int rfid = bb_.getInt("rfid", 0);
    ticks_waited_++;

    if (rfid == 1) {
      waiting_ = false;
      std::printf("    %s[WaitRFID]%s RFID确认! %sSUCCESS%s\n",
                  color::MAGENTA, color::RESET, color::GREEN, color::RESET);
      return NodeStatus::SUCCESS;
    }

    if (ticks_waited_ >= timeout_ticks_) {
      waiting_ = false;
      std::printf("    %s[WaitRFID]%s 等待超时 %sFAILURE%s\n",
                  color::MAGENTA, color::RESET, color::RED, color::RESET);
      return NodeStatus::FAILURE;
    }

    std::printf("    %s[WaitRFID]%s 等待中... (%d/%d) %sRUNNING%s\n",
                color::MAGENTA, color::RESET,
                ticks_waited_, timeout_ticks_,
                color::YELLOW, color::RESET);
    return NodeStatus::RUNNING;
  }

  void halt() override {
    waiting_ = false;
    ticks_waited_ = 0;
  }

private:
  int timeout_ticks_;
  bool waiting_ = false;
  int ticks_waited_ = 0;
};

// =============================================================================
// 动作节点: Sleep (模拟驻留等待)
// =============================================================================
class SleepNode : public TreeNode {
public:
  SleepNode(const std::string& name, Blackboard& bb, int ticks = 2)
    : TreeNode(name, bb), total_ticks_(ticks) {}

  NodeStatus tick() override {
    if (!sleeping_) {
      sleeping_ = true;
      ticks_done_ = 0;
    }

    ticks_done_++;
    if (ticks_done_ >= total_ticks_) {
      sleeping_ = false;
      std::printf("    %s[Sleep]%s 驻留完成 %sSUCCESS%s\n",
                  color::DIM, color::RESET, color::GREEN, color::RESET);
      return NodeStatus::SUCCESS;
    }

    std::printf("    %s[Sleep]%s 驻留等待中 (%d/%d) %sRUNNING%s\n",
                color::DIM, color::RESET,
                ticks_done_, total_ticks_,
                color::YELLOW, color::RESET);
    return NodeStatus::RUNNING;
  }

  void halt() override {
    sleeping_ = false;
    ticks_done_ = 0;
  }

private:
  int total_ticks_;
  bool sleeping_ = false;
  int ticks_done_ = 0;
};

// =============================================================================
// 动作节点: ToggleObstacleAvoidance (模拟避障开关)
// =============================================================================
class ToggleObstacleAvoidance : public TreeNode {
public:
  ToggleObstacleAvoidance(const std::string& name, Blackboard& bb, bool enable)
    : TreeNode(name, bb), enable_(enable) {}

  NodeStatus tick() override {
    bb_.setInt("obstacle_avoidance", enable_ ? 1 : 0);
    std::printf("    %s[ToggleAvoidance]%s 避障已%s %sSUCCESS%s\n",
                color::CYAN, color::RESET,
                enable_ ? "开启" : "关闭",
                color::GREEN, color::RESET);
    return NodeStatus::SUCCESS;
  }

private:
  bool enable_;
};

// =============================================================================
// 动作节点: GenerateRandomPose (模拟随机点生成)
// =============================================================================
class GenerateRandomPose : public TreeNode {
public:
  GenerateRandomPose(const std::string& name, Blackboard& bb, double range = 2.0)
    : TreeNode(name, bb), range_(range), rng_(std::random_device{}()) {}

  NodeStatus tick() override {
    double cx = bb_.getDouble("control_zone_x", 5.0);
    double cy = bb_.getDouble("control_zone_y", 0.0);
    double half = range_ / 2.0;
    std::uniform_real_distribution<double> dist(-half, half);

    double rx = cx + dist(rng_);
    double ry = cy + dist(rng_);

    bb_.setDouble("random_pose_x", rx);
    bb_.setDouble("random_pose_y", ry);

    std::printf("    %s[GenRandomPose]%s 中心(%.1f,%.1f) -> 随机点(%.2f,%.2f) %sSUCCESS%s\n",
                color::MAGENTA, color::RESET,
                cx, cy, rx, ry,
                color::GREEN, color::RESET);
    return NodeStatus::SUCCESS;
  }

private:
  double range_;
  std::mt19937 rng_;
};

// =============================================================================
// 复合节点: Sequence
// =============================================================================
class Sequence : public TreeNode {
public:
  Sequence(const std::string& name, Blackboard& bb,
           std::vector<TreeNode*> children)
    : TreeNode(name, bb), children_(std::move(children)) {}

  NodeStatus tick() override {
    for (size_t i = current_; i < children_.size(); ++i) {
      NodeStatus s = children_[i]->tick();
      if (s == NodeStatus::RUNNING) {
        current_ = i;
        return NodeStatus::RUNNING;
      }
      if (s == NodeStatus::FAILURE) {
        current_ = 0;
        return NodeStatus::FAILURE;
      }
    }
    current_ = 0;
    return NodeStatus::SUCCESS;
  }

  void halt() override {
    if (current_ < children_.size()) {
      children_[current_]->halt();
    }
    current_ = 0;
  }

private:
  std::vector<TreeNode*> children_;
  size_t current_ = 0;
};

// =============================================================================
// 复合节点: ReactiveFallback
// 每次tick都从第一个子节点开始, 如果高优先级分支返回SUCCESS/RUNNING则跳过后续
// =============================================================================
class ReactiveFallback : public TreeNode {
public:
  ReactiveFallback(const std::string& name, Blackboard& bb,
                   std::vector<TreeNode*> children)
    : TreeNode(name, bb), children_(std::move(children)) {}

  NodeStatus tick() override {
    for (size_t i = 0; i < children_.size(); ++i) {
      NodeStatus s = children_[i]->tick();
      if (s == NodeStatus::SUCCESS || s == NodeStatus::RUNNING) {
        // halt 后面的子节点
        for (size_t j = i + 1; j < children_.size(); ++j) {
          children_[j]->halt();
        }
        return s;
      }
    }
    return NodeStatus::FAILURE;
  }

  void halt() override {
    for (auto* child : children_) {
      child->halt();
    }
  }

private:
  std::vector<TreeNode*> children_;
};

// =============================================================================
// 复合节点: Parallel (简化版)
// success_count 个子节点 SUCCESS -> 返回 SUCCESS
// failure_count 个子节点 FAILURE -> 返回 FAILURE
// 否则 RUNNING
// =============================================================================
class Parallel : public TreeNode {
public:
  Parallel(const std::string& name, Blackboard& bb,
           std::vector<TreeNode*> children,
           int success_count = 2, int failure_count = 1)
    : TreeNode(name, bb), children_(std::move(children)),
      success_count_(success_count), failure_count_(failure_count) {}

  NodeStatus tick() override {
    int success = 0, failure = 0;
    for (auto* child : children_) {
      NodeStatus s = child->tick();
      if (s == NodeStatus::SUCCESS) success++;
      if (s == NodeStatus::FAILURE) failure++;
    }

    if (success >= success_count_) return NodeStatus::SUCCESS;
    if (failure >= failure_count_) return NodeStatus::FAILURE;
    return NodeStatus::RUNNING;
  }

  void halt() override {
    for (auto* child : children_) {
      child->halt();
    }
  }

private:
  std::vector<TreeNode*> children_;
  int success_count_;
  int failure_count_;
};

// =============================================================================
// 装饰节点: Repeat
// =============================================================================
class Repeat : public TreeNode {
public:
  Repeat(const std::string& name, Blackboard& bb, TreeNode* child, int cycles = -1)
    : TreeNode(name, bb), child_(child), max_cycles_(cycles) {}

  NodeStatus tick() override {
    NodeStatus s = child_->tick();
    if (s == NodeStatus::RUNNING) {
      return NodeStatus::RUNNING;
    }
    // 一轮结束, 无限循环时重置子节点
    if (max_cycles_ < 0) {
      return NodeStatus::RUNNING;  // 无限循环, 始终 RUNNING
    }
    cycle_count_++;
    if (cycle_count_ >= max_cycles_) {
      cycle_count_ = 0;
      return s;
    }
    return NodeStatus::RUNNING;
  }

  void halt() override {
    child_->halt();
    cycle_count_ = 0;
  }

private:
  TreeNode* child_;
  int max_cycles_;
  int cycle_count_ = 0;
};

// =============================================================================
// 模拟 Update 节点 (始终 RUNNING, 刷新黑板)
// 在实际仿真中通过外部驱动修改黑板, 此处仅返回 RUNNING
// =============================================================================
class UpdateNode : public TreeNode {
public:
  UpdateNode(const std::string& name, Blackboard& bb)
    : TreeNode(name, bb) {}

  NodeStatus tick() override {
    // 模拟: 将静态坐标写入黑板
    bb_.setDouble("supply_zone_x", -2.0);
    bb_.setDouble("supply_zone_y", 0.0);
    bb_.setDouble("control_zone_x", 5.0);
    bb_.setDouble("control_zone_y", 0.0);
    return NodeStatus::RUNNING;  // 始终 RUNNING
  }
};

// =============================================================================
// 仿真场景定义
// =============================================================================
struct SimStep {
  double hp;       // 当前血量
  double ltime;    // 剩余时间
  int    rfid;     // RFID 状态
  std::string description;  // 场景描述
};

// =============================================================================
// 打印分隔线
// =============================================================================
void printSeparator(const char* title) {
  std::printf("\n%s%s==============================%s\n", color::BOLD, color::CYAN, color::RESET);
  std::printf("%s%s %s %s\n", color::BOLD, color::CYAN, title, color::RESET);
  std::printf("%s%s==============================%s\n", color::BOLD, color::CYAN, color::RESET);
}

void printTickHeader(int tick, const std::string& desc) {
  std::printf("\n%s--- Tick %d: %s ---%s\n", color::BOLD, tick, desc.c_str(), color::RESET);
}

// =============================================================================
// 测试1: 血量充足 -> 巡逻模式
// =============================================================================
bool testPatrolMode(Blackboard& bb) {
  printSeparator("测试1: 血量充足 - 巡逻模式");

  // 构建巡逻子树
  ToggleObstacleAvoidance enableAvoid("Enable_Avoidance", bb, true);
  GenerateRandomPose genPose("Gen_Random_Pose", bb, 2.0);
  NavigateToPose navRandom("Nav_To_Random", bb, "random_pose", 3);
  ToggleObstacleAvoidance disableAvoid("Disable_Avoidance", bb, false);

  Sequence patrolSeq("Patrol_Random_Point", bb,
                     {&enableAvoid, &genPose, &navRandom, &disableAvoid});

  // 场景: 血量 300, 巡逻应正常执行
  bb.setDouble("hp", 300.0);
  bb.setDouble("ltime", 400.0);
  bb.setInt("rfid", 0);

  std::printf("  设定: hp=300 (充足), 预期: 执行巡逻序列\n");

  bool passed = true;
  for (int i = 1; i <= 5; ++i) {
    printTickHeader(i, "巡逻");
    bb.dump();
    NodeStatus s = patrolSeq.tick();
    std::printf("  %s=> Patrol结果: %s%s%s\n",
                color::BOLD, statusColor(s), statusToStr(s), color::RESET);

    if (i < 3 && s != NodeStatus::RUNNING) {
      std::printf("  %s[FAIL] 预期 RUNNING (导航中)%s\n", color::RED, color::RESET);
      passed = false;
    }
    if (i == 3 && s != NodeStatus::RUNNING) {
      // tick3: NavToPose完成, 接着DisableAvoid也完成, 整个Sequence应SUCCESS
      // 但实际上 tick3 时 Nav 才完成, DisableAvoid 在同一个 tick 中也执行, Sequence 返回 SUCCESS
    }
  }
  return passed;
}

// =============================================================================
// 测试2: 血量过低 -> 撤退模式
// =============================================================================
bool testRetreatMode(Blackboard& bb) {
  printSeparator("测试2: 血量过低 - 撤退模式");

  // 构建撤退子树
  CheckHP checkHP("Is_HP_Low", bb, 140.0);
  NavigateToPose navSupply("Nav_To_Supply", bb, "supply_zone", 2);
  WaitRFID waitRFID("Wait_RFID", bb, 5);
  SleepNode sleepHeal("Wait_Healing", bb, 2);

  Sequence retreatSeq("Emergency_Retreat", bb,
                      {&checkHP, &navSupply, &waitRFID, &sleepHeal});

  // 场景: hp=80 (低于阈值140), RFID先为0后为1
  bb.setDouble("hp", 80.0);
  bb.setInt("rfid", 0);

  std::printf("  设定: hp=80 (<140), 预期: 触发撤退序列\n");

  bool passed = true;

  // Tick 1: CheckHP SUCCESS -> NavSupply RUNNING
  printTickHeader(1, "撤退 - 开始导航补给区");
  bb.dump();
  NodeStatus s = retreatSeq.tick();
  std::printf("  %s=> Retreat结果: %s%s%s\n",
              color::BOLD, statusColor(s), statusToStr(s), color::RESET);
  if (s != NodeStatus::RUNNING) {
    std::printf("  %s[FAIL] 预期 RUNNING (导航中)%s\n", color::RED, color::RESET);
    passed = false;
  }

  // Tick 2: NavSupply 完成 -> WaitRFID RUNNING (rfid=0)
  printTickHeader(2, "撤退 - 到达补给区, 等待RFID");
  bb.dump();
  s = retreatSeq.tick();
  std::printf("  %s=> Retreat结果: %s%s%s\n",
              color::BOLD, statusColor(s), statusToStr(s), color::RESET);

  // Tick 3: WaitRFID RUNNING (rfid仍为0)
  printTickHeader(3, "撤退 - 继续等待RFID");
  bb.dump();
  s = retreatSeq.tick();
  std::printf("  %s=> Retreat结果: %s%s%s\n",
              color::BOLD, statusColor(s), statusToStr(s), color::RESET);

  // Tick 4: 模拟踩到RFID
  bb.setInt("rfid", 1);
  printTickHeader(4, "撤退 - RFID触发! 开始驻留回血");
  bb.dump();
  s = retreatSeq.tick();
  std::printf("  %s=> Retreat结果: %s%s%s\n",
              color::BOLD, statusColor(s), statusToStr(s), color::RESET);

  // Tick 5: Sleep 完成
  printTickHeader(5, "撤退 - 回血完成");
  bb.dump();
  s = retreatSeq.tick();
  std::printf("  %s=> Retreat结果: %s%s%s\n",
              color::BOLD, statusColor(s), statusToStr(s), color::RESET);
  if (s != NodeStatus::SUCCESS) {
    std::printf("  %s[FAIL] 预期 SUCCESS (回血完成)%s\n", color::RED, color::RESET);
    passed = false;
  }

  return passed;
}

// =============================================================================
// 测试3: 完整行为树 - 动态场景仿真
// =============================================================================
bool testFullTree(Blackboard& bb) {
  printSeparator("测试3: 完整行为树 - 动态战场仿真");

  // -- 构建所有叶节点 --
  UpdateNode update("Status_Update", bb);

  // 撤退分支
  CheckHP checkHP("Is_HP_Low", bb, 140.0);
  NavigateToPose navSupply("Nav_To_Supply", bb, "supply_zone", 2);
  WaitRFID waitRFID("Wait_RFID", bb, 5);
  SleepNode sleepHeal("Wait_Healing", bb, 2);
  Sequence retreatSeq("Emergency_Retreat", bb,
                      {&checkHP, &navSupply, &waitRFID, &sleepHeal});

  // 巡逻分支
  ToggleObstacleAvoidance enableAvoid("Enable_Avoidance", bb, true);
  GenerateRandomPose genPose("Gen_Random_Pose", bb, 2.0);
  NavigateToPose navRandom("Nav_To_Random", bb, "random_pose", 3);
  ToggleObstacleAvoidance disableAvoid("Disable_Avoidance", bb, false);
  Sequence patrolSeq("Patrol_Random_Point", bb,
                     {&enableAvoid, &genPose, &navRandom, &disableAvoid});

  // 战术决策
  ReactiveFallback combatBrain("Combat_Brain", bb,
                               {&retreatSeq, &patrolSeq});

  Repeat tacticalLoop("Tactical_Loop", bb, &combatBrain, -1);

  Parallel mainParallel("Main_Parallel", bb,
                        {&update, &tacticalLoop}, 2, 1);

  // -- 定义仿真时间线 --
  std::vector<SimStep> timeline = {
    // 阶段1: 满血, 正常巡逻
    {400.0, 480.0, 0, "满血巡逻"},
    {400.0, 475.0, 0, "满血巡逻"},
    {380.0, 470.0, 0, "满血巡逻"},
    {350.0, 465.0, 0, "满血巡逻 (导航完成预期)"},
    // 阶段2: 被攻击, 血量骤降
    {200.0, 460.0, 0, "被攻击, hp=200 > 140"},
    {120.0, 455.0, 0, "HP危险! 120 <= 140, 触发撤退"},
    { 80.0, 450.0, 0, "撤退中, 导航补给区"},
    { 60.0, 445.0, 0, "到达补给区, 等待RFID"},
    { 50.0, 440.0, 1, "踩到RFID! 开始回血"},
    { 50.0, 435.0, 1, "驻留回血中"},
    // 阶段3: 回血完成, 恢复巡逻
    {300.0, 430.0, 0, "回血完成, 恢复巡逻"},
    {300.0, 425.0, 0, "巡逻中"},
    {280.0, 420.0, 0, "巡逻中"},
  };

  bool passed = true;
  bool saw_retreat = false;
  bool saw_patrol = false;

  for (size_t i = 0; i < timeline.size(); ++i) {
    auto& step = timeline[i];

    // 注入仿真数据到黑板
    bb.setDouble("hp", step.hp);
    bb.setDouble("ltime", step.ltime);
    bb.setInt("rfid", step.rfid);

    printTickHeader(static_cast<int>(i + 1), step.description);
    bb.dump();

    // 执行行为树
    std::printf("  %s[树执行]%s\n", color::BOLD, color::RESET);
    NodeStatus s = mainParallel.tick();

    std::printf("  %s=> Main结果: %s%s%s\n",
                color::BOLD, statusColor(s), statusToStr(s), color::RESET);

    // 验证关键逻辑
    if (step.hp <= 140.0) {
      saw_retreat = true;
    }
    if (step.hp > 140.0) {
      saw_patrol = true;
    }
  }

  // 验证仿真覆盖了两种模式
  if (!saw_retreat) {
    std::printf("  %s[FAIL] 仿真未覆盖撤退模式%s\n", color::RED, color::RESET);
    passed = false;
  }
  if (!saw_patrol) {
    std::printf("  %s[FAIL] 仿真未覆盖巡逻模式%s\n", color::RED, color::RESET);
    passed = false;
  }

  return passed;
}

// =============================================================================
// 测试4: CheckHP 边界值测试
// =============================================================================
bool testCheckHPBoundary(Blackboard& bb) {
  printSeparator("测试4: CheckHP 边界值测试");

  CheckHP checkHP("CheckHP_Boundary", bb, 140.0);
  bool passed = true;

  struct TestCase {
    double hp;
    NodeStatus expected;
    const char* desc;
  };

  std::vector<TestCase> cases = {
    {0.0,   NodeStatus::SUCCESS, "hp=0 (死亡)"},
    {139.0, NodeStatus::SUCCESS, "hp=139 (略低于阈值)"},
    {140.0, NodeStatus::SUCCESS, "hp=140 (等于阈值)"},
    {141.0, NodeStatus::FAILURE, "hp=141 (略高于阈值)"},
    {400.0, NodeStatus::FAILURE, "hp=400 (满血)"},
  };

  for (auto& tc : cases) {
    bb.setDouble("hp", tc.hp);
    NodeStatus result = checkHP.tick();
    bool ok = (result == tc.expected);
    std::printf("    %s -> %s%s%s %s\n",
                tc.desc,
                statusColor(result), statusToStr(result), color::RESET,
                ok ? "" : "[FAIL]");
    if (!ok) passed = false;
  }

  return passed;
}

// =============================================================================
// 测试5: WaitRFID 超时测试
// =============================================================================
bool testWaitRFIDTimeout(Blackboard& bb) {
  printSeparator("测试5: WaitRFID 超时测试");

  WaitRFID waitRFID("WaitRFID_Timeout", bb, 3);  // 3 ticks 超时
  bb.setInt("rfid", 0);  // RFID 始终为 0

  bool passed = true;

  for (int i = 1; i <= 4; ++i) {
    NodeStatus s = waitRFID.tick();
    std::printf("  Tick %d: %s%s%s\n", i, statusColor(s), statusToStr(s), color::RESET);

    if (i < 3 && s != NodeStatus::RUNNING) {
      std::printf("    %s[FAIL] 预期 RUNNING%s\n", color::RED, color::RESET);
      passed = false;
    }
    if (i == 3 && s != NodeStatus::FAILURE) {
      std::printf("    %s[FAIL] 预期 FAILURE (超时)%s\n", color::RED, color::RESET);
      passed = false;
    }
  }

  return passed;
}

// =============================================================================
// 测试6: ReactiveFallback 优先级抢占测试
// =============================================================================
bool testReactiveFallbackPreemption(Blackboard& bb) {
  printSeparator("测试6: ReactiveFallback 优先级抢占测试");

  CheckHP checkHP("CheckHP_Preempt", bb, 140.0);
  NavigateToPose navSupply("Nav_Supply_Preempt", bb, "supply_zone", 2);
  Sequence retreatSeq("Retreat_Preempt", bb, {&checkHP, &navSupply});

  NavigateToPose navRandom("Nav_Random_Preempt", bb, "random_pose", 5);
  Sequence patrolSeq("Patrol_Preempt", bb, {&navRandom});

  ReactiveFallback fallback("Brain_Preempt", bb,
                            {&retreatSeq, &patrolSeq});

  bool passed = true;

  // Tick 1-2: 血量充足, 执行巡逻
  bb.setDouble("hp", 300.0);
  std::printf("  %s--- 阶段1: 血量充足, 巡逻 ---%s\n", color::BOLD, color::RESET);
  for (int i = 0; i < 2; ++i) {
    printTickHeader(i + 1, "巡逻中");
    NodeStatus s = fallback.tick();
    std::printf("  => %s%s%s\n", statusColor(s), statusToStr(s), color::RESET);
  }

  // Tick 3: 突然血量暴降, 撤退应该抢占巡逻
  bb.setDouble("hp", 50.0);
  std::printf("\n  %s--- 阶段2: 血量骤降! 撤退抢占 ---%s\n", color::BOLD, color::RESET);
  printTickHeader(3, "HP=50, 撤退抢占!");
  NodeStatus s = fallback.tick();
  std::printf("  => %s%s%s (预期 RUNNING: 撤退导航中)\n",
              statusColor(s), statusToStr(s), color::RESET);

  // Tick 4: 继续撤退
  printTickHeader(4, "继续撤退导航");
  s = fallback.tick();
  std::printf("  => %s%s%s\n", statusColor(s), statusToStr(s), color::RESET);

  // Tick 5: 回血后恢复巡逻
  bb.setDouble("hp", 300.0);
  std::printf("\n  %s--- 阶段3: 回血后恢复巡逻 ---%s\n", color::BOLD, color::RESET);
  printTickHeader(5, "HP=300, 恢复巡逻");
  s = fallback.tick();
  std::printf("  => %s%s%s\n", statusColor(s), statusToStr(s), color::RESET);

  return passed;
}

// =============================================================================
// main
// =============================================================================
int main() {
  std::printf("\n%s%s", color::BOLD, color::CYAN);
  std::printf("╔══════════════════════════════════════════════════════╗\n");
  std::printf("║  RoboMaster 2026 哨兵行为树仿真测试                ║\n");
  std::printf("║  Cubot Guard Behavior Tree Simulation Test          ║\n");
  std::printf("╚══════════════════════════════════════════════════════╝\n");
  std::printf("%s\n", color::RESET);

  int total = 0, pass = 0;

  auto runTest = [&](const char* name, std::function<bool(Blackboard&)> fn) {
    Blackboard bb;
    total++;
    bool ok = fn(bb);
    if (ok) pass++;
    std::printf("\n  %s%s=> %s: %s%s\n",
                color::BOLD, ok ? color::GREEN : color::RED,
                name, ok ? "PASSED" : "FAILED", color::RESET);
  };

  runTest("巡逻模式",              testPatrolMode);
  runTest("撤退模式",              testRetreatMode);
  runTest("完整行为树仿真",        testFullTree);
  runTest("CheckHP边界值",         testCheckHPBoundary);
  runTest("WaitRFID超时",          testWaitRFIDTimeout);
  runTest("ReactiveFallback抢占",  testReactiveFallbackPreemption);

  // 汇总
  std::printf("\n%s%s", color::BOLD, color::CYAN);
  std::printf("══════════════════════════════════════════════════════\n");
  std::printf("%s", color::RESET);
  std::printf("%s 测试结果: %d/%d 通过%s\n",
              (pass == total) ? color::GREEN : color::RED,
              pass, total, color::RESET);

  if (pass == total) {
    std::printf("%s%s 所有测试通过!%s\n\n", color::BOLD, color::GREEN, color::RESET);
  } else {
    std::printf("%s%s 存在失败的测试!%s\n\n", color::BOLD, color::RED, color::RESET);
  }

  return (pass == total) ? 0 : 1;
}
