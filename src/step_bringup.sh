#!/bin/bash
# ============================================================================
# 分步启动 + 交互验证脚本
#   每个阶段：弹独立 gnome-terminal 启动节点 → 主脚本自动跑验证命令
#   → 用户判断输出是否正常 → [Enter]=继续 | r=重启该阶段 | q=中止全部
# 用法:  bash /home/cubot/NAV/src/step_bringup.sh
# ============================================================================

# 注意: 不能 set -u, ROS setup.bash 内部使用了未绑定变量 AMENT_TRACE_SETUP_FILES
SETUP_NAV="/home/cubot/NAV/install/setup.bash"
SERIAL_DEV="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
SERIAL_SCALE="1.0"
NAV2_MAP="/home/cubot/NAV/install/sentry_bringup/share/sentry_bringup/maps/cubot.yaml"
NAV2_PARAMS="/home/cubot/NAV/install/sentry_bringup/share/sentry_bringup/config/nav2_params.yaml"
RVIZ_CFG="/home/cubot/NAV/src/sentry_bringup/config/mapping.rviz"
# T9 Nav2 起来后, 在 RViz 里手动将 Fixed Frame 改为 map, 并 Add Map(/map, Transient Local)/Path(/plan)。

# ------------------- 颜色 -------------------
G='\033[1;32m'; Y='\033[1;33m'; R='\033[1;31m'; B='\033[1;34m'; N='\033[0m'
hdr(){ echo -e "\n${B}==================== $1 ====================${N}"; }
ok(){  echo -e "${G}[ OK ] $*${N}"; }
warn(){ echo -e "${Y}[WARN] $*${N}"; }
err(){ echo -e "${R}[FAIL] $*${N}"; }

# ------------------- 弹终端启动 -------------------
# launch_term <title> <cmd>
launch_term(){
    local title="$1"; shift
    local cmd="$*"
    gnome-terminal --title="$title" -- bash -c \
        "source /opt/ros/jazzy/setup.bash; source $SETUP_NAV; echo '>>> $title'; $cmd; echo; echo '[退出] 回车关闭'; read" &
    sleep 1
}

# ------------------- 验证封装 -------------------
# 在主脚本环境内运行 ros2 命令需要 source
source /opt/ros/jazzy/setup.bash
source $SETUP_NAV 2>/dev/null || { err "请先编译并 source $SETUP_NAV"; exit 1; }

check_topic_hz(){     # check_topic_hz <topic> <max_wait_sec>
    local topic="$1"; local sec="${2:-8}"
    echo "→ ros2 topic hz $topic  (持续 ${sec}s)"
    timeout ${sec}s ros2 topic hz "$topic" 2>&1 | tail -5
}
check_tf(){           # check_tf <parent> <child>
    local p="$1"; local c="$2"
    echo "→ tf2_echo $p $c  (持续 3s)"
    timeout 3s ros2 run tf2_ros tf2_echo "$p" "$c" 2>&1 | tail -10
}
check_node_alive(){   # check_node_alive <node_name_substr>
    local kw="$1"
    echo "→ ros2 node list | grep $kw"
    ros2 node list 2>/dev/null | grep -E "$kw" || echo "(未发现)"
}

# ------------------- 交互确认 -------------------
# confirm <stage_name>   返回: 0=继续  1=重启  2=中止
confirm(){
    local stage="$1"
    while true; do
        echo
        echo -en "${Y}[$stage] 输出正常? [Enter=继续 / r=重启该阶段 / q=中止]: ${N}"
        read -r ans
        case "$ans" in
            "")  return 0 ;;
            r|R) return 1 ;;
            q|Q) return 2 ;;
            *)   warn "无效输入" ;;
        esac
    done
}

# ------------------- 阶段定义 -------------------
# run_stage <name> <launch_cmd> <verify_func>
run_stage(){
    local name="$1"; local launch="$2"; local verify="$3"
    while true; do
        hdr "$name"
        echo "弹终端: $launch"
        launch_term "$name" "$launch"
        echo "等待节点就绪..."; sleep 5
        eval "$verify"
        confirm "$name"
        case $? in
            0) ok "$name 通过"; return 0 ;;
            1) warn "重启 $name (会再弹一个新终端,请手动关旧的)" ;;
            2) err "用户中止"; exit 1 ;;
        esac
    done
}

# ------------------- T1 清理 -------------------
hdr "T1 清理旧进程"
pkill -9 -f 'livox_ros_driver2|point_lio|small_gicp|ekf_node|cpp_lidar_filter|robot_state_publisher|serial_send_node|goal_pose_bridge|cmd_vel_monitor|rx_monitor|nav2_|rviz2' 2>/dev/null
sleep 1
ok "已 pkill"; confirm "T1 清理" || true

# ------------------- T2 Livox -------------------
run_stage "T2 Livox 驱动" \
    "ros2 launch livox_ros_driver2 msg_MID360_launch.py" \
    'check_topic_hz /livox/lidar 6; check_topic_hz /livox/imu 4'

# ------------------- T3 RSP -------------------
run_stage "T3 RSP (URDF→TF)" \
    "ros2 launch sentry_bringup rsp.launch.py" \
    'check_tf base_link lidar_link; check_node_alive robot_state_publisher'

# ------------------- T4 Point-LIO -------------------
# rviz:=False  禁用 point_lio launch 自带的 ROS1 旧格式 loam_livox.rviz (显示为空)
run_stage "T4 Point-LIO" \
    "ros2 launch point_lio point_lio.launch.py rviz:=False" \
    'check_topic_hz /Odometry 6; check_tf odom base_link'

# ------------------- T5 Core Pipeline -------------------
run_stage "T5 Core (filter+P2L+GICP+EKF)" \
    "ros2 launch sentry_bringup core_pipeline.launch.py" \
    'check_topic_hz /cloud_filtered 6; check_topic_hz /scan 4; check_tf map odom'

# ------------------- T6 RViz (无验证) -------------------
hdr "T6 RViz"
launch_term "T6 RViz" "rviz2 -d $RVIZ_CFG"
echo "请在 RViz 中确认 /cloud_registered 点云、 /Odometry 箭头、 RobotModel 显示正常 (Fixed Frame=odom)"
echo "提示: T9 Nav2 启动后可在 RViz 手动 Add: Map(/map TransientLocal)、LaserScan(/scan)、Path(/plan)、Costmap 等"
confirm "T6 RViz" || true

# ------------------- T7 Serial -------------------
run_stage "T7 Serial 下发" \
    "ros2 run serial_send serial_send_node $SERIAL_DEV $SERIAL_SCALE --ros-args --log-level info" \
    'check_node_alive serial_send'

# ------------------- T7B cmd_vel TX 监控 -------------------
run_stage "T7B cmd_vel 监控" \
    "ros2 run sentry_bringup cmd_vel_monitor.py" \
    'check_node_alive cmd_vel_monitor'

# ------------------- T9 Nav2 -------------------
run_stage "T9 Nav2 bringup" \
    "ros2 launch nav2_bringup bringup_launch.py use_rviz:=False map:=$NAV2_MAP params_file:=$NAV2_PARAMS" \
    'sleep 8; echo "→ ros2 lifecycle get /bt_navigator"; ros2 lifecycle get /bt_navigator 2>&1 | tail -3; echo "→ ros2 lifecycle get /controller_server"; ros2 lifecycle get /controller_server 2>&1 | tail -3'

# ------------------- T10 goal_pose 桥接 -------------------
run_stage "T10 goal_pose 桥接" \
    "ros2 run sentry_bringup goal_pose_bridge.py" \
    'check_node_alive goal_pose_bridge'

# ------------------- T8 RX 监控 -------------------
run_stage "T8 RX 监控" \
    "ros2 run sentry_bringup rx_monitor.py" \
    'check_node_alive rx_monitor'

hdr "全部阶段通过"
ok "全栈联调完成。在 RViz 用 '2D Goal Pose' 测导航。"
ok "行为树: ros2 run cubot_guard_behavior_tree mission_executor --ros-args -p bt_xml_filename:=/home/cubot/NAV/src/cubot_guard_behavior_tree/behavior_trees/sentinel_main_tree.xml"
ok "串口监控: ros2 topic echo /referee/hp"
