cmake_minimum_required(VERSION 3.5)
project(robot_localization_ranges)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_localization REQUIRED)
find_package(anchor_msgs REQUIRED)

add_library(ekf_node SHARED src/ekf_node.cpp)
target_include_directories(ekf_node PRIVATE include)
ament_target_dependencies(ekf_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros urdf anchor_msgs robot_localization)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

# auto-executable from node
rclcpp_components_register_node(ekf_node
  PLUGIN "robot_localization_ranges::EKFNode"
  EXECUTABLE ekf)

install(TARGETS ekf_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(ekf_node)
ament_package()
