cmake_minimum_required(VERSION 3.8)
project(sentry_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 资源目录安装（含 launch / config / maps / urdf）
install(DIRECTORY launch config maps urdf
  DESTINATION share/${PROJECT_NAME}
)

# Python 脚本安装为可执行（供 ros2 run sentry_bringup xxx.py 调用）
install(PROGRAMS
  scripts/cmd_vel_monitor.py
  scripts/goal_pose_bridge.py
  scripts/rx_monitor.py
  scripts/mission_manager.py
  scripts/waypoint_patrol.py
  scripts/send_waypoints.py
  scripts/stl_to_pcd.py
  scripts/fake_chassis_node.py
  scripts/fake_pointcloud_node.py
  scripts/pcd_publisher.py
  scripts/initial_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
