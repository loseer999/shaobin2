cmake_minimum_required(VERSION 3.16)
project(kiss_matcher_relocalization VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ========================================================================
# 关键：强制所有子目录编译 -fPIC，包括 KISS-Matcher 和 ROBIN
# ========================================================================
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ========================================================================
# KISS-Matcher core
# ========================================================================
set(KM_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../KISS-Matcher-main/cpp/kiss_matcher")
if(EXISTS ${KM_CORE_DIR}/CMakeLists.txt)
  add_subdirectory(${KM_CORE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/kiss_matcher_core)
  message(STATUS "KISS-Matcher core from sibling: ${KM_CORE_DIR}")
else()
  find_package(kiss_matcher REQUIRED)
  message(STATUS "KISS-Matcher core via find_package")
endif()

# ========================================================================
# ROS2 dependencies
# ========================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(MPI REQUIRED COMPONENTS C CXX)

if(MPI_FOUND AND NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
  set_target_properties(MPI::MPI_C PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MPI_C_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${MPI_C_LIBRARIES}"
    INTERFACE_COMPILE_OPTIONS "${MPI_C_COMPILE_OPTIONS}"
  )
endif()

if(MPI_FOUND AND NOT TARGET MPI::MPI_CXX)
  add_library(MPI::MPI_CXX INTERFACE IMPORTED)
  set_target_properties(MPI::MPI_CXX PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MPI_CXX_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${MPI_CXX_LIBRARIES}"
    INTERFACE_COMPILE_OPTIONS "${MPI_CXX_COMPILE_OPTIONS}"
  )
endif()

find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ========================================================================
# Node library
# ========================================================================
set(LIB_NAME "${PROJECT_NAME}_lib")
add_library(${LIB_NAME} SHARED
  src/kiss_matcher_relocalization.cpp
)

target_link_libraries(${LIB_NAME}
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${LIB_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  PCL
)

rclcpp_components_register_node(${LIB_NAME}
  PLUGIN "kiss_matcher_relocalization::KissMatcherRelocalizationNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ========================================================================
# Install
# ========================================================================
install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()