#!/bin/bash
# ============================================================
#   LIO + Nav2 精简启动脚本 (无重定位, PGM 栅格地图)
#   坐标链路:
#     odom → base_link : Point-LIO (实时里程计)
#     map  → odom     : static_transform_publisher (固定原点)
#     base_link → laser: URDF / robot_state_publisher
# ============================================================

set -e

WORKSPACE_ROOT="/home/cubot/NAV"
source /opt/ros/jazzy/setup.bash
source ${WORKSPACE_ROOT}/install/setup.bash

# 串口设备 by-id (CP2102)
SERIAL_DEV="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
SERIAL_SCALE="1.0"

echo "[CLEAN] 清理旧进程..."
pkill -9 -f 'pointlio_mapping|livox_ros_driver2|robot_state_publisher|rviz2|cpp_lidar_filter|pointcloud_to_laserscan|nav2|static_transform_publisher|serial_send_node|cmd_vel_monitor' 2>/dev/null || true
sleep 2

launch_term(){
    gnome-terminal --title="$1" -- bash -c \
        "source /opt/ros/jazzy/setup.bash; \
         source ${WORKSPACE_ROOT}/install/setup.bash; \
         echo '>>> $1'; \
         $2; \
         echo '--- exited, press ENTER to close ---'; read" &
    sleep 0.3
}

echo "=============================================="
echo "  LIO + Nav2 启动 (无重定位, PGM 地图)"
echo "=============================================="

echo "[1/2] 启动 Livox MID360 驱动..."
launch_term "LIVOX" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 4

echo "[2/3] 启动 LIO + Nav2 (RSP / LIO / filter / laserscan / static TF / Nav2 / RViz)..."
launch_term "LIO_NAV" "ros2 launch sentry_bringup lio_nav.launch.py"
sleep 3

echo "[3/4] 启动 Serial Send (底盘串口)..."
launch_term "SERIAL" "ros2 run serial_send serial_send_node $SERIAL_DEV $SERIAL_SCALE --ros-args --log-level info"
sleep 2

echo "=============================================="
echo "✅ 基础节点启动完成"
echo "   • /scan      ← Point-LIO 实时点云 (局部避障)"
echo "   • /map       ← PGM 栅格地图 (全局规划)"
echo "   • map→odom   ← 静态 TF (identity)"
echo "   • serial_send ← 底盘串口 (cmd_vel → MCU)"
echo "   • 无 AMCL/GICP/KISS 重定位，odom 会漂移"
echo "=============================================="
echo "基础节点已就绪，等待 5s 后自动加载行为树..."
sleep 5

echo "[4/4] 加载行为树..."
BT_XML="/home/cubot/NAV/src/cubot_guard_behavior_tree/behavior_trees/sentinel_main_tree.xml"
launch_term "BEHAVIOR_TREE" "ros2 run cubot_guard_behavior_tree mission_executor --ros-args -p bt_xml_filename:=\"$BT_XML\""

echo ""
echo "=============================================="
echo "✅ 全部节点就绪"
echo ""
echo "  ┌─ 感知层"
echo "  │  /scan       ← lidar_filter → pointcloud_to_laserscan"
echo "  │  /map        ← HALL PGM 栅格地图"
echo "  │  map→odom    ← 静态 TF (identity)"
echo "  ├─ 控制层"
echo "  │  serial_send ← 底盘串口 ${SERIAL_DEV}"
echo "  │  HP/lthp/ammo ← MCU → /referee/* → 行为树黑板"
echo "  ├─ 行为树优先级 (ReactiveFallback)"
echo "  │  ① 撤退  HP<250 或 ammo不足 → 补给→中继→驻守/保卫"
echo "  │  ② 保卫  protect_active锁存 → 中继2→1→保卫点"
echo "  │  ③ 进攻  attack_done=0 (比赛开始) → 进攻路线"
echo "  │  ④ 驻守  默认 → 驻守点待命"
echo "  └─ 坐标"
echo "     补给(-1.46,-5.59) 中继1(2.04,0.96) 中继2(4.41,0.82)"
echo "     中继3(-0.30,-2.79) 中继4(-1.45,-4.42)"
echo "     驻守(7.10,4.47) 保卫(1.76,0.98)"
echo ""
echo "  串口数据: ros2 topic echo /referee/hp"
echo "  行为树日志: ros2 run rqt_console (DEBUG级别)"
echo "=============================================="
echo "按 Enter 停止所有节点..."
read -r

echo "[CLEAN] 停止节点..."
pkill -9 -f 'pointlio_mapping|livox_ros_driver2|robot_state_publisher|rviz2|cpp_lidar_filter|pointcloud_to_laserscan|nav2|static_transform_publisher|serial_send_node|cmd_vel_monitor|mission_executor' 2>/dev/null || true
echo "完成"
