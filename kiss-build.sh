#!/bin/bash
# 防爆内存编译入口：串行包 + 每包 4 线程
set -e
source /opt/ros/jazzy/setup.bash
cd /home/cubot/NAV
export MAKEFLAGS="-j4"
colcon build \
    --base-paths src \
    --symlink-install \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DHUMBLE_ROS=humble
