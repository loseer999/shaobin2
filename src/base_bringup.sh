#!/bin/bash
# =====================================================================
# NAV 机器人启动脚本 (sentry_bringup 新栈版) —— 10 阶段顺序布局
# ---------------------------------------------------------------------
#   阶段 1  : 清理残留进程                                  [主脚本]
#   阶段 2  : Livox Driver                                 [终端]  T2. Livox
#   阶段 3  : Robot State Publisher (URDF→TF)              [终端]  T3. RSP
#   阶段 4  : Point-LIO  (/Odometry, /cloud_registered_body)[终端]  T4. Point-LIO
#   阶段 5  : Core Pipeline (滤波+P2L+GICP+EKF)        [终端]  T5. Core
#   阶段 5K : KISS-Matcher 全局粗定位 (独立窗口看 inliers)  [终端]  T5K. KISS
#   阶段 6  : Nav2 RViz                                    [终端]  T6. RViz
#   阶段 7  : Serial Send + TX 监控                        [终端]  T7. Serial · T7B. TX
#   阶段 8  : LIO + EKF map→odom 就绪验证                  [主脚本]
#   阶段 9  : Nav2 导航栈 (nav2_bringup + new.yaml)        [终端]  T9. Nav2
#   阶段 10 : 感知就绪 + Nav2 激活 + 辅助脚本              [主脚本+T8.RX+T10.Bridge]
# ---------------------------------------------------------------------
# 链路状态（方案二完全对齐版）：
#   - Point-LIO 源码级发布转正后的局部机体点云 /cloud_registered_body (frame_id: base_link)
#   - GICP 与 pointcloud_to_laserscan 直接订阅 /cloud_registered_body (base_link)
#   - small_gicp 关闭内部 TF 倒装翻转，精准输出 /pose，阻绝 2 米跳变
#   - pointcloud_to_laserscan 完美吃进 Reliable 点云，输出高质量 /scan，对接 Nav2
# =====================================================================

# ---------- 路径配置 ----------
SETUP_NAV="/home/cubot/NAV/install/setup.bash"

# ---------- 主脚本 source ROS 环境 ----------
# 【关键】主脚本会直接调用 ros2 (topic info / tf2_echo / lifecycle get) 验证节点状态,
# 必须显式 source. 子终端通过 launch_term 各自 source ~/.bashrc, 不影响主脚本.
source /opt/ros/jazzy/setup.bash
source "$SETUP_NAV"

# 串口设备 by-id (CP2102)
SERIAL_DEV="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
SERIAL_SCALE="1.0"   # 上位机不再乘系数，MCU 同步取消 ÷15 还原

# ---------- 计时 ----------
SCRIPT_START=$SECONDS
echo ">>> ⏱  计时开始: $(date '+%H:%M:%S')"
stage_time() {
    local ELAPSED=$((SECONDS - SCRIPT_START))
    printf "    ⏱  [+%3ds] %s\n" "$ELAPSED" "$1"
}

# ---------- 通用终端启动 ----------
# 参数: 1.标题 2.窗口几何 3.环境 source 指令 4.运行命令
launch_term() {
    local TITLE=$1 GEO=$2 SETUP_CMD=$3 RUN_CMD=$4
    local INNER="source ~/.bashrc; $SETUP_CMD; history -s '$RUN_CMD'; $RUN_CMD; exec bash"
    gnome-terminal --window --title="$TITLE" --geometry=$GEO -- bash -c "$INNER"
}

# ---------- 后台启动 ----------
BG_LOG_DIR="/tmp/nav_logs"
mkdir -p "$BG_LOG_DIR"
BG_LOGS=()
launch_bg() {
    local TITLE=$1 SETUP_CMD=$2 RUN_CMD=$3
    local LOG_FILE="$BG_LOG_DIR/${TITLE}.log"
    echo "    → [后台] $TITLE  日志: $LOG_FILE"
    nohup bash -c "source ~/.bashrc; $SETUP_CMD; exec $RUN_CMD" >"$LOG_FILE" 2>&1 &
    disown
    BG_LOGS+=("$TITLE|$LOG_FILE")
}

# ---------- Nav2 lifecycle 检查工具 ----------
check_lifecycle_active() {
    local NODE_NAME=$1 MAX_RETRY=${2:-30}
    for i in $(seq 1 $MAX_RETRY); do
        local STATE
        STATE=$(ros2 lifecycle get "$NODE_NAME" 2>/dev/null | grep -o 'active \[3\]' || true)
        if [ "$STATE" = "active [3]" ]; then
            echo "  ✅ $NODE_NAME active"
            return 0
        fi
        sleep 1
    done
    local FINAL
    FINAL=$(ros2 lifecycle get "$NODE_NAME" 2>/dev/null || echo "无法获取")
    echo "  ❌ $NODE_NAME 未激活 (当前: $FINAL)"
    return 1
}

check_nav2_servers() {
    local TIMEOUT=${1:-10}
    echo ""
    echo "========== Nav2 导航服务器激活状态 (并行检测, 超时 ${TIMEOUT}s) =========="
    local NODES=(
        /controller_server /smoother_server /planner_server
        /behavior_server /velocity_smoother
        /bt_navigator /waypoint_follower
    )
    local TMPDIR; TMPDIR=$(mktemp -d -t nav2_lc.XXXX)
    local PIDS=()
    for n in "${NODES[@]}"; do
        local SAFE=${n//\//_}
        (
            if check_lifecycle_active "$n" "$TIMEOUT" >"$TMPDIR/${SAFE}.log" 2>&1; then
                echo OK >"$TMPDIR/${SAFE}.st"
            else
                echo FAIL >"$TMPDIR/${SAFE}.st"
            fi
        ) &
        PIDS+=($!)
    done
    for pid in "${PIDS[@]}"; do wait "$pid"; done
    local OK=true
    for n in "${NODES[@]}"; do
        local SAFE=${n//\//_}
        cat "$TMPDIR/${SAFE}.log" 2>/dev/null
        [ "$(cat "$TMPDIR/${SAFE}.st" 2>/dev/null)" = "OK" ] || OK=false
    done
    rm -rf "$TMPDIR"
    $OK && echo "🟢 Nav2 全部服务器已激活" || echo "🟡 警告: 部分 Nav2 未激活"
}

check_map_server() {
    echo ""
    echo "========== map_server 激活状态 =========="
    check_lifecycle_active "/map_server" 30 \
        && echo "🟢 /map 可订阅" \
        || echo "🟡 map_server 未激活"
}

# =====================================================================
# 阶段 1/10 : 清理残留进程
# =====================================================================
echo ""
echo ">>> [1/10] 清理上一次残留进程..."

CLEANUP_TARGETS=(
    livox_ros_driver2
    point_lio
    point_lio
    small_gicp_node small_gicp_relocalization
    kiss_matcher_reloc kiss_matcher_relocalization
    ekf_node ekf_fusion_node
    serial_send_node
    goal_pose_bridge.py rx_monitor.py cmd_vel_monitor.py
    pointcloud_to_laserscan
    nav2_controller nav2_planner nav2_bt_navigator nav2_behaviors
    nav2_waypoint_follower nav2_collision_monitor nav2_map_server
    nav2_lifecycle_manager nav2_velocity_smoother nav2_smoother_server
    component_container
    rviz2
    robot_state_publisher joint_state_publisher static_transform_publisher
    "ros2 launch sentry_bringup"
    "ros2 launch nav2_bringup"
    "ros2 launch point_lio"
    "ros2 launch livox_ros_driver2"
    mission_executor
)
for PAT in "${CLEANUP_TARGETS[@]}"; do
    PIDS=$(pgrep -f "$PAT" 2>/dev/null || true)
    if [ -n "$PIDS" ]; then
        echo "    → kill $PAT : $PIDS"
        pkill -9 -f "$PAT" 2>/dev/null || true
    fi
done

# 关掉旧弹窗 (标题前缀统一 T<N>.)
for PFX in "T2. " "T3. " "T4. " "T4M. " "T5. " "T5K. " "T6. " "T7. " "T7B. " "T8. " "T9. " "T10. "; do
    wmctrl -l 2>/dev/null | grep -F "$PFX" | awk '{print $1}' | while read -r WID; do
        wmctrl -ic "$WID" 2>/dev/null || true
    done
done

sleep 1
REMAIN=$(pgrep -fa 'point_lio|small_gicp|kiss_matcher|livox_ros_driver2|robot_state_publisher|ekf_node' 2>/dev/null || true)
if [ -n "$REMAIN" ]; then
    echo "⚠️  仍有残留，再清一次:"
    echo "$REMAIN"
    pkill -9 -f 'point_lio|small_gicp|kiss_matcher|livox_ros_driver2|robot_state_publisher|ekf_node' 2>/dev/null || true
    sleep 1
fi
stage_time "阶段 1 完成 — 残留清理"

# =====================================================================
# 阶段 2/10 : Livox Driver
# =====================================================================
echo ""
echo ">>> [2/10] 启动 Livox Mid360 (终端 T2)..."
launch_term "T2. Livox" "80x15+0+0" \
    "source $SETUP_NAV" \
    "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
stage_time "阶段 2 完成 — Livox Driver"

# =====================================================================
# 阶段 3/10 : Robot State Publisher (URDF → TF)
# ---------------------------------------------------------------------
# 必须在 Point-LIO 之前启动！
# LIO 依赖 base_link↔lidar_link 的静态 TF 来反算 base_link 位姿。
# =====================================================================
echo ""
echo ">>> [3/10] 启动 robot_state_publisher (终端 T3)..."
launch_term "T3. RSP" "80x12+0+700" \
    "source $SETUP_NAV" \
    "ros2 launch sentry_bringup rsp.launch.py"

for i in {1..10}; do
    if timeout 0.3 bash -c 'ros2 run tf2_ros tf2_echo base_link lidar_link 2>&1 | grep -m1 -q "Translation"'; then
        echo "    ✅ base_link→lidar_link TF 就绪"
        break
    fi
    sleep 0.2
done
stage_time "阶段 3 完成 — robot_state_publisher"

# =====================================================================
# 阶段 4/10 : Point-LIO  [终端 T4]
# ---------------------------------------------------------------------
# 输出核心：/cloud_registered_body (完全对齐 base_link 局部正向系)
# =====================================================================
echo ""
echo ">>> [4/10] 启动 Point-LIO (终端 T4)..."
launch_term "T4. Point-LIO" "80x20+0+200" \
    "source $SETUP_NAV" \
    "ros2 launch point_lio point_lio.launch.py rviz:=False"
stage_time "阶段 4 完成 — Point-LIO 已投递"

# =====================================================================
# 阶段 4M/10 : PCD 地图发布  [终端 T4M]
# =====================================================================
echo ""
echo ">>> [4M/10] 启动 PCD 地图发布 (终端 T4M)..."
MAP_PCD="/home/cubot/NAV/install/sentry_bringup/share/sentry_bringup/maps/scans.pcd"
launch_term "T4M. MapCloud" "60x10+650+600" \
    "source $SETUP_NAV" \
    "ros2 run sentry_bringup pcd_publisher.py --ros-args -p pcd_file:=$MAP_PCD -p frame_id:=map -p topic:=/map_cloud"
stage_time "阶段 4M 完成 — PCD 地图发布已投递"

# =====================================================================
# 阶段 5/10 : Core Pipeline  [终端 T5]
# ---------------------------------------------------------------------
# 包含：pointcloud_to_laserscan (直接订阅 /cloud_registered_body，输出稳定 /scan)
#       small_gicp_relocalization (精配准，base 与 lidar 均指定为 base_link，消除2米高度跳变)
#       ekf_fusion (发布核心 map→odom 动态 TF 树)
# =====================================================================
echo ""
echo ">>> [5/10] 启动 Core Pipeline (终端 T5)..."
CORE_LOG="$BG_LOG_DIR/T5_core.log"
: > "$CORE_LOG"
launch_term "T5. Core" "80x18+950+0" \
    "source $SETUP_NAV" \
    "ros2 launch sentry_bringup core_pipeline.launch.py 2>&1 | tee $CORE_LOG"
stage_time "阶段 5 完成 — Core Pipeline 已投递"

# =====================================================================
# 阶段 5K/10 : KISS-Matcher 全局粗定位  [终端 T5K]  【已禁用】
# =====================================================================
echo ""
echo ">>> [5K/10] KISS-Matcher 已禁用，跳过启动"

# =====================================================================
# 阶段 6/10 : Nav2 RViz  [终端 T6]
# =====================================================================
echo ""
echo ">>> [6/10] 启动 Nav2 监控 RViz (终端 T6)..."
launch_term "T6. RViz" "80x18+950+350" \
    "source $SETUP_NAV" \
    "rviz2 -d /home/cubot/NAV/src/sentry_bringup/config/rviz_nav2.rviz"

# =====================================================================
# 阶段 7/10 : Serial Send (底盘串口)  [终端 T7. Serial · T7B. TX]
# =====================================================================
echo ""
echo ">>> [7/10] 启动 Serial Send (T7. Serial) + cmd_vel 速度链路监控 (T7B. CmdVel)..."

SERIAL_RUN="ros2 run serial_send serial_send_node $SERIAL_DEV $SERIAL_SCALE --ros-args --log-level info"
launch_term "T7. Serial" "80x15+950+700" \
    "source $SETUP_NAV" \
    "$SERIAL_RUN"

launch_term "T7B. CmdVel" "100x12+650+450" \
    "source $SETUP_NAV" \
    "ros2 run sentry_bringup cmd_vel_monitor.py"

stage_time "阶段 7 完成 — Serial + CmdVel 两阶段监控已投递"

# =====================================================================
# 阶段 8/10 : LIO + GICP 就绪验证
# ---------------------------------------------------------------------
# 注意：map→odom 现由 EKF 发布 (GICP 的 /tf 已被 remap 到 /tf_null 避开冲突)
# =====================================================================
echo ""
echo ">>> [8/10] 验证 LIO 与 map→odom 状态..."

# 8.a /Odometry
echo "  [8.a] /Odometry..."
for i in {1..10}; do
    ros2 topic info /Odometry >/dev/null 2>&1 && { echo "    ✅ /Odometry 就绪"; break; }
    sleep 1
done

# 8.b odom→base_link TF
echo "  [8.b] odom→base_link TF..."
if timeout 30 bash -c 'ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -m1 -q "Translation"'; then
    echo "    ✅ odom→base_link TF 就绪"
else
    echo "    ❌ odom→base_link TF 超时(30s)，查看 T4 终端"
fi

# 8.e map→odom TF  (由 EKF 输出)
echo "  [8.e] map→odom TF (EKF 输出)..."
if timeout 30 bash -c 'ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m1 -q "Translation"'; then
    echo "    ✅ map→odom TF 就绪"
else
    echo "    🟡 map→odom TF 超时(30s)，GICP 可能尚未收敛"
    echo "    排查: tail -f $CORE_LOG  (实时观察 GICP 终端输出)"
    echo "          手动: RViz 用 '2D Pose Estimate' 发 /initialpose 强制辅助初始化"
fi

stage_time "阶段 8 完成 — LIO + map→odom 验证"

# =====================================================================
# 阶段 9/10 : Nav2 导航栈  [终端 T9]
# ---------------------------------------------------------------------
# 精简配置集，由 GICP 统一进行全局定位，防数据跳变
# =====================================================================
echo ""
echo ">>> [9/10] 启动 Nav2 导航栈 (终端 T9)..."
NAV2_MAP="/home/cubot/NAV/install/sentry_bringup/share/sentry_bringup/maps/scans.yaml"
NAV2_PARAMS="/home/cubot/NAV/install/sentry_bringup/share/sentry_bringup/config/nav2_params.yaml"
launch_term "T9. Nav2" "80x18+0+900" \
    "source $SETUP_NAV" \
    "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$NAV2_MAP -p use_sim_time:=False & \
     sleep 1 && ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_map -p autostart:=true -p use_sim_time:=False -p 'node_names:=[map_server]' & \
     sleep 2 && ros2 launch sentry_bringup nav2_minimal.launch.py params_file:=$NAV2_PARAMS"
stage_time "阶段 9 完成 — Nav2 已投递 (对齐 Best-Effort QoS，动态避障管道已畅通)"

# =====================================================================
# 阶段 10/10 : 感知就绪 + Nav2 激活 + 辅助脚本
# =====================================================================
echo ""
echo ">>> [10/10] 感知就绪 / Nav2 激活 / 辅助后台..."

# 10.b 等 Nav2 lifecycle
echo "  [10.b] 等待 Nav2 lifecycle..."
sleep 3
check_map_server
check_nav2_servers 10

# 10.c goal_pose → navigate_to_pose 桥接 (手动导航时可用)
launch_term "T10. Bridge" "80x12+0+550" \
    "source $SETUP_NAV" \
    "ros2 run sentry_bringup goal_pose_bridge.py"

stage_time "阶段 10 完成 — 辅助脚本已投递"

# =====================================================================
# 阶段 11/10 : 行为树加载 (sentinel_main_tree.xml)
# =====================================================================
echo ""
echo "=============================================="
echo "基础节点已就绪，等待 5s 后自动加载行为树..."
sleep 5

BT_XML="/home/cubot/NAV/src/cubot_guard_behavior_tree/behavior_trees/sentinel_main_tree.xml"
launch_term "T11. BT" "80x18+0+450" \
    "source $SETUP_NAV" \
    "ros2 run cubot_guard_behavior_tree mission_executor --ros-args -p bt_xml_filename:=\"$BT_XML\""

echo ""
echo "=============================================="
echo "✅ 行为树已加载"
echo ""
echo "  优先级 (ReactiveFallback):"
echo "    ① 撤退  HP<250 or ammo不足 → 双路线(补给→中继→驻守/保卫)"
echo "    ② 保卫  protect_active锁存 → 中继2→1→保卫点"
echo "    ③ 进攻  attack_done=0 (比赛开始) → 进攻路线"
echo "    ④ 驻守  默认 → 驻守点待命"
echo ""
echo "  串口监控: ros2 topic echo /referee/hp"
echo "=============================================="

# =====================================================================
# 最终汇总 + 停止
# =====================================================================
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║            NAV 系统启动完成报告 (精简栈)                      ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  感知层  Livox + Point-LIO + 滤波 + P2L → /scan              ║"
echo "║  定位层  GICP 精配准 + EKF map→odom                           ║"
echo "║  控制层  serial_send ←→ MCU (HP/ltime/ammo/jdhp)             ║"
echo "║  导航层  Nav2 (PGM 栅格地图 + 动态避障)                       ║"
echo "║  行为树  sentinel_main_tree.xml                                ║"
echo "║         ①撤退(HP<250) ②保卫(protect_active) ③进攻 ④驻守      ║"
echo "╚══════════════════════════════════════════════════════════════╝"

TOTAL=$((SECONDS - SCRIPT_START))
echo ""
echo "⏱  启动总耗时: ${TOTAL}s  (≈ $((TOTAL/60))分$((TOTAL%60))秒)   结束: $(date '+%H:%M:%S')"
echo ""

if [ ${#BG_LOGS[@]} -gt 0 ]; then
    echo "─── 后台日志 ───"
    for entry in "${BG_LOGS[@]}"; do
        printf "  %-20s  tail -f %s\n" "${entry%%|*}" "${entry##*|}"
    done
    echo "───────────────────────────────────────"
fi

echo ""
echo ">>> [STOP] 停止所有节点..."

# 关闭终端窗口
for PFX in "T2. " "T3. " "T4. " "T4M. " "T5. " "T5K. " "T6. " "T7. " "T7B. " "T8. " "T9. " "T10. " "T11. "; do
    wmctrl -l 2>/dev/null | grep -F "$PFX" | awk '{print $1}' | while read -r WID; do
        wmctrl -ic "$WID" 2>/dev/null || true
    done
done

# 杀死所有进程
for PAT in "${CLEANUP_TARGETS[@]}"; do
    PIDS=$(pgrep -f "$PAT" 2>/dev/null || true)
    if [ -n "$PIDS" ]; then
        pkill -9 -f "$PAT" 2>/dev/null || true
    fi
done

echo "✅ 所有节点已停止"
echo "=============================================="