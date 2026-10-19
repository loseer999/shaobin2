cmake_minimum_required(VERSION 3.8)
project(serial_send)

# 默认使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED) 

include_directories(include)

# 添加可执行文件
add_executable(serial_send_node src/serial_send.cpp)

# 链接 ROS 2 依赖
ament_target_dependencies(serial_send_node
  rclcpp
  geometry_msgs
  std_msgs
  serial
)

# 链接 serial 库 (通常 serial 库不是 ament 包，需要单独链接)
target_link_libraries(serial_send_node ${serial_LIBRARIES})

# 安装规则
install(TARGETS
  serial_send_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()