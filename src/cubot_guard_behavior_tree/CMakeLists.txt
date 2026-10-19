cmake_minimum_required(VERSION 3.8)
project(cubot_guard_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_behavior_tree
  nav2_msgs
  std_msgs
  geometry_msgs
  rcl_interfaces
  ament_index_cpp
)

# mission_executor 额外需要 ament_index_cpp
set(mission_executor_dependencies
  ${dependencies}
  ament_index_cpp
)

# =============================================================================
# BT Plugin Library - 所有自定义行为树节点
# =============================================================================
add_library(cubot_guard_bt_plugins SHARED
  # 动作节点
  src/plugins/action/update.cpp
  src/plugins/action/toggle_obstacle_avoidance.cpp
  src/plugins/action/set_move_mode.cpp
  src/plugins/action/set_stance.cpp
  src/plugins/action/generate_random_pose.cpp
  src/plugins/action/build_pose_array.cpp
  # 条件节点
  src/plugins/condition/check_hp.cpp
  src/plugins/condition/check_ammo.cpp
  src/plugins/condition/check_time.cpp
  src/plugins/condition/check_game_started.cpp
  src/plugins/condition/check_game_ended.cpp
  src/plugins/condition/check_signal.cpp
  src/plugins/condition/wait_rfid.cpp
  src/plugins/condition/wait_for_revive.cpp
  # 动作节点
  src/plugins/action/wait_for_signal.cpp
  # 装饰器节点
  src/plugins/decorator/rate_controller.cpp
  # 统一注册入口 (BT_REGISTER_NODES 只能出现一次)
  src/register_nodes.cpp
)

target_include_directories(cubot_guard_bt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cubot_guard_bt_plugins ${dependencies})

# 必须定义 BT_PLUGIN_EXPORT, 否则 BT_REGISTER_NODES 宏会将注册函数声明为 static,
# 导致 registerFromPlugin() 在运行时找不到 BT_RegisterNodesFromPlugin 符号
target_compile_definitions(cubot_guard_bt_plugins PRIVATE BT_PLUGIN_EXPORT)

# =============================================================================
# Mission Executor - 任务级行为树执行器节点
# =============================================================================
add_executable(mission_executor
  src/mission_executor.cpp
)

target_include_directories(mission_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mission_executor ${mission_executor_dependencies})

# =============================================================================
# Install
# =============================================================================
install(TARGETS cubot_guard_bt_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# 安装行为树 XML 文件
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export for Nav2 to find the plugins
ament_export_include_directories(include)
ament_export_libraries(cubot_guard_bt_plugins)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
