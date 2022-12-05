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
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(anchor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(robot_localization REQUIRED)
find_package(angles REQUIRED)

find_package(Eigen3 REQUIRED)

# sync with robot_localization source
set(RL_DIR robot_localization)
set(RL_DIR_ABS "${PROJECT_SOURCE_DIR}/${RL_DIR}")
execute_process(COMMAND lsb_release -sc OUTPUT_VARIABLE UBUNTU_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)

# was using release repo but Humble is at another place
#set(RL_BRANCH "debian/$ENV{ROS_DISTRO}/${UBUNTU_VERSION}/robot_localization")
#set(RL_URL https://github.com/cra-ros-pkg/robot_localization-release)

# usr devel repo, works on Foxy -> Humble
set(RL_BRANCH "$ENV{ROS_DISTRO}-devel")
set(RL_URL https://github.com/cra-ros-pkg/robot_localization)

if(NOT EXISTS ${RL_DIR_ABS})
    execute_process(COMMAND
    git clone ${RL_URL} -b ${RL_BRANCH} ${RL_DIR} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
else()
    execute_process(COMMAND git checkout -b ${RL_BRANCH} WORKING_DIRECTORY ${RL_DIR_ABS} OUTPUT_QUIET ERROR_QUIET)
endif()

# build custom RL lib
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED COMPONENTS STATIC)

add_library(rl_range SHARED
  ${RL_DIR}/src/ekf.cpp
  ${RL_DIR}/src/ukf.cpp
  ${RL_DIR}/src/filter_base.cpp
  ${RL_DIR}/src/filter_utilities.cpp
  ${RL_DIR}/src/navsat_transform.cpp
  ${RL_DIR}/src/robot_localization_estimator.cpp
  ${RL_DIR}/src/ros_filter.cpp
  ${RL_DIR}/src/ros_filter_utilities.cpp
  ${RL_DIR}/src/ros_robot_localization_listener.cpp
  #src/ros_filter_local.cpp
  #src/ekf.cpp
  )
target_include_directories(rl_range PRIVATE include ${RL_DIR}/include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(rl_range ${GeographicLib_LIBRARIES})
ament_target_dependencies(rl_range
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor
  robot_localization
  angles)


add_executable(ekf
    src/ekf_node.cpp
    src/ros_filter_ranges.cpp
    src/ekf.cpp
    )
target_include_directories(ekf PRIVATE include ${RL_DIR}/include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ekf rl_range)
ament_target_dependencies(ekf rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros urdf anchor_msgs std_srvs angles)


if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

install(TARGETS rl_range #efk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY tests DESTINATION share/${PROJECT_NAME})

ament_package()
