#!/bin/bash

# ================================================================
# mission_bringup.sh - 行为树启动脚本
# 
# 用途: 在 base_bringup.sh 启动全栈导航后，手动加载行为树
# 特点: 自动检测 Nav2 bt_navigator 就绪状态，就绪后才启动行为树
#
# 使用方法:
#   1. 先运行 ./base_bringup.sh 启动全栈导航
#   2. 确认 RViz 中地图加载完成、机器人位姿正确
#   3. 运行 ./mission_bringup.sh 启动行为树
# ================================================================

SETUP_NAV="/home/cubot/NAV/install/setup.bash"
BT_PKG_DIR="/home/cubot/NAV/install/cubot_guard_behavior_tree/share/cubot_guard_behavior_tree"

# ---- 颜色输出 ----
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}>>> [MissionBringup] 行为树启动脚本${NC}"
echo ""

# ---- 1. 等待 bt_navigator 的 action server 就绪 ----
echo -e "${YELLOW}[1/3] 检测 Nav2 bt_navigator 就绪状态...${NC}"
echo "    (每 2 秒检测一次，最多等待 60 秒)"

check_count=0
max_checks=30

while [ $check_count -lt $max_checks ]; do
    # 检测 navigate_to_pose action server 是否可用
    if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
        echo -e "${GREEN}    ✓ Nav2 bt_navigator 已就绪!${NC}"
        break
    fi
    
    check_count=$((check_count + 1))
    echo "    等待中... ($check_count/$max_checks)"
    sleep 2
done

if [ $check_count -eq $max_checks ]; then
    echo -e "${RED}    ✗ 超时: Nav2 bt_navigator 未就绪，请检查 base_bringup.sh 是否正常运行${NC}"
    echo "    提示: 可能需要先在 RViz 中设置 2D Pose Estimate 完成初始定位"
    exit 1
fi

# ---- 2. 确认机器人初始位姿已设置 ----
echo ""
echo -e "${YELLOW}[2/3] 确认机器人初始位姿...${NC}"

# 检查 map→base_link TF 是否存在
if ros2 tf2_tools view_frames >/dev/null 2>&1; then
    if ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | head -1 | grep -q "Translation"; then
        echo -e "${GREEN}    ✓ TF 树正常 (map → base_link)${NC}"
    else
        echo -e "${YELLOW}    ⚠ 警告: 未检测到 map→base_link TF${NC}"
        echo "    提示: 请在 RViz 中点击 '2D Pose Estimate' 设置初始位姿"
        read -p "    是否继续? [Y/n]: " confirm
        if [ "$confirm" = "n" ] || [ "$confirm" = "N" ]; then
            exit 0
        fi
    fi
else
    echo -e "${YELLOW}    ⚠ 跳过 TF 检查${NC}"
fi

# ---- 3. 启动行为树 ----
echo ""
echo -e "${GREEN}[3/3] 启动 mission_executor...${NC}"
echo "    行为树文件: ${BT_PKG_DIR}/behavior_trees/sentinel_main_tree.xml"
echo "    配置文件:   ${BT_PKG_DIR}/config/behavior_tree_params.yaml"
echo ""

source $SETUP_NAV

# 使用 gnome-terminal 启动，便于查看日志
gnome-terminal --window --title="5. Mission Executor (Behavior Tree)" \
    --geometry="100x24+0+700" \
    -- bash -c "
        source ~/.bashrc
        source ${SETUP_NAV}
        echo '>>> 正在启动行为树...'
        ros2 run cubot_guard_behavior_tree mission_executor \
            --ros-args \
            -p bt_xml_filename:='${BT_PKG_DIR}/behavior_trees/sentinel_main_tree.xml' \
            -p bt_loop_duration:=50 \
            -p server_timeout:=20000 \
            -p wait_for_service_timeout:=30000 \
            -p max_tree_creation_retries:=15 \
            -p tree_creation_retry_interval:=5000
        exec bash
    "

echo -e "${GREEN}>>> 行为树已启动!${NC}"
echo ""
echo "  优先级 (ReactiveFallback):"
echo "    ① 撤退  HP<250 or ammo不足 → 双路线"
echo "    ② 保卫  protect_active锁存 → 中继2→1→保卫点"
echo "    ③ 进攻  attack_done=0 → 进攻路线"
echo "    ④ 驻守  默认 → 驻守点待命"
echo ""
echo "  串口数据: ros2 topic echo /referee/hp"
echo "  行为树状态: ros2 topic echo /mission_executor/status"
echo ""
echo "  停止: 在 BT 终端中按 Ctrl+C"
