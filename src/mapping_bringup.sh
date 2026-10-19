#!/bin/bash
# ============================================================
#   NAV 建图脚本 (适配当前架构)
#   方案: Livox MID360 -> Point-LIO 自带 PCD 保存
#   输出: /home/cubot/NAV/src/sentry_bringup/maps/map_<TS>.pcd
# ============================================================
# 注意: 不能 set -u, ROS setup.bash 内有未绑定变量 (AMENT_TRACE_SETUP_FILES)

set -e

WORKSPACE_ROOT="/home/cubot/NAV"
TIMESTAMP=$(date +%m%d%H%M)   # 月日时分, 如 09082211 = 9月8日22:11
MAPS_DIR="${WORKSPACE_ROOT}/src/sentry_bringup/maps"
PCD_OUTPUT="${MAPS_DIR}/map_${TIMESTAMP}.pcd"
LIO_PCD_DIR="${WORKSPACE_ROOT}/src/pcd/PCD"
YAML="${WORKSPACE_ROOT}/src/point_lio/config/mid360.yaml"
YAML_BAK="${YAML}.mapping_bak"
RVIZ_CFG="${WORKSPACE_ROOT}/src/sentry_bringup/config/mapping.rviz"

mkdir -p "$MAPS_DIR" "$LIO_PCD_DIR"

# ---------- 启动前强制清理残留进程 ----------
# 注意: pkill 找不到匹配进程返回 1, set -e 会让脚本退出, 必须加 || true
echo "[CLEAN] 清理旧进程..."
pkill -9 -f 'pointlio_mapping'         2>/dev/null || true
pkill -9 -f 'livox_ros_driver2_node'   2>/dev/null || true
pkill -9 -f 'robot_state_publisher'    2>/dev/null || true
pkill -9 -f 'rviz2'                    2>/dev/null || true
pkill -9 -f 'small_gicp_relocalization' 2>/dev/null || true
pkill -9 -f 'ekf_node'                 2>/dev/null || true
pkill -9 -f 'cpp_lidar_filter'         2>/dev/null || true
pkill -9 -f 'fake_vel_transform'       2>/dev/null || true
pkill -9 -f 'pointcloud_to_laserscan'  2>/dev/null || true
sleep 2
# 顺手清上次未恢复的 yaml 备份 (防止遺留)
if [ -f "${YAML_BAK}" ]; then
    cp "${YAML_BAK}" "${YAML}"
    rm -f "${YAML_BAK}"
    echo "[CLEAN] 恢复上次遗留的 mid360.yaml"
fi
echo "[CLEAN] 完成"

# ---------- 临时开启 Point-LIO 的 PCD 保存 ----------
# 只在 yaml 中定位出 pcd_save_en 这行 (12+ 个空格缩进), 避免误伤 cut_frame_time_interval
cp "$YAML" "$YAML_BAK"
sed -i -E 's/^([[:space:]]+)pcd_save_en:[[:space:]]*(False|True|false|true)/\1pcd_save_en: True/' "$YAML"
# interval 默认就是 -1 (全部帧存 1 个 pcd), 不需修改

cleanup() {
    echo ""
    echo "[CLEAN] 恢复 mid360.yaml 并停止节点..."
    [ -f "$YAML_BAK" ] && cp "$YAML_BAK" "$YAML" && rm -f "$YAML_BAK"
    pkill -2  -f "pointlio_mapping"        2>/dev/null
    pkill -2  -f "livox_ros_driver2_node"  2>/dev/null
    pkill -2  -f "robot_state_publisher"   2>/dev/null
    pkill -2  -f "rviz2"                   2>/dev/null
    sleep 2
    pkill -9  -f "pointlio_mapping"        2>/dev/null
    pkill -9  -f "livox_ros_driver2_node"  2>/dev/null
    pkill -9  -f "robot_state_publisher"   2>/dev/null
    pkill -9  -f "rviz2"                   2>/dev/null
    echo "[CLEAN] 完成"
}
trap cleanup EXIT INT TERM

source /opt/ros/jazzy/setup.bash
source ${WORKSPACE_ROOT}/install/setup.bash

clear
echo "=============================================="
echo "  NAV 建图启动 (Livox + Point-LIO + RViz)"
echo "  目标 PCD: $PCD_OUTPUT"
echo "=============================================="

launch_term(){
    # $1=title  $2=command
    gnome-terminal --title="$1" -- bash -c \
        "source /opt/ros/jazzy/setup.bash; \
         source ${WORKSPACE_ROOT}/install/setup.bash; \
         echo '>>> $1'; \
         $2; \
         echo '--- exited, press ENTER to close ---'; read" &
    sleep 0.3
}

echo ""
echo "[1/4] 启动 Livox MID360 驱动..."
launch_term "MAP-Livox" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 4

echo "[2/4] 启动 RSP (URDF 静态 TF)..."
launch_term "MAP-RSP" "ros2 launch sentry_bringup rsp.launch.py"
sleep 2

echo "[3/4] 启动 Point-LIO (PCD 保存已开启, 禁用自带 RViz)..."
launch_term "MAP-Point-LIO" "ros2 launch point_lio point_lio.launch.py rviz:=False"
sleep 5

echo "[4/4] 启动 RViz..."
launch_term "MAP-RViz" "rviz2 -d $RVIZ_CFG 2>/dev/null"

echo ""
echo "=============================================="
echo "✅ 建图运行中 (共 ${SECONDS}s)"
echo "   • Livox MID360 驱动      [MAP-Livox 终端]"
echo "   • Robot State Publisher  [MAP-RSP 终端]"
echo "   • Point-LIO + PCD 保存   [MAP-Point-LIO 终端]"
echo "   • RViz 可视化            [MAP-RViz 终端]"
echo ""
echo "   🟡 推着机器人慢速走一圈 (覆盖目标区域)"
echo "   🟡 注意走出环路, 帮助 LIO 闭环"
echo "   🟡 完成后回到本终端按 [Enter] 保存并退出"
echo "=============================================="
read -r

echo ""
echo "[Save 1/3] 给 Point-LIO 发 SIGINT 触发 PCD 写盘..."
pkill -INT -f "pointlio_mapping" 2>/dev/null
echo "          等待写盘 (10s)..."
sleep 10

echo "[Save 2/3] 查找输出 PCD..."
LATEST=$(ls -t "${LIO_PCD_DIR}"/*.pcd 2>/dev/null | head -1)
if [ -z "$LATEST" ] || [ ! -s "$LATEST" ]; then
    echo "[ERROR] 未找到 PCD 输出于 ${LIO_PCD_DIR}"
    echo "        ↳ 请检查 Point-LIO 是否正常退出 (终端 MAP-Point-LIO)"
    exit 1
fi

echo "[Save 3/3] 拷贝到 sentry_bringup/maps/..."
cp "$LATEST" "$PCD_OUTPUT"
SIZE=$(du -h "$PCD_OUTPUT" | cut -f1)
PTS=$(head -20 "$PCD_OUTPUT" | grep -E "^POINTS" | awk '{print $2}')

echo ""
echo "=============================================="
echo "✅ 建图完成"
echo "   • PCD : $PCD_OUTPUT"
echo "   • 大小: $SIZE  (约 $PTS 点)"
echo ""
echo "🟡 投产步骤 (二选一):"
echo ""
echo "  方式 A — 覆盖默认地图 new.pcd (T5 立即生效):"
echo "    cp $PCD_OUTPUT ${MAPS_DIR}/new.pcd"
echo "    cd ${WORKSPACE_ROOT} && colcon build --packages-select sentry_bringup --symlink-install"
echo ""
echo "  方式 B — 修改 launch 引用新文件:"
echo "    编辑 src/sentry_bringup/launch/core_pipeline.launch.py L15"
echo "    pcd_map_path = ... 'maps', 'map_${TIMESTAMP}.pcd'"
echo "=============================================="
