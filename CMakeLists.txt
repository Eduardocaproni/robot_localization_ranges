cmake_minimum_required(VERSION 3.5)
project(robot_localization_ranges)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

# sync with robot_localization source
# can be removed when robot_localization comes with headers
set(RL_DIR robot_localization)
set(RL_DIR_ABS "${PROJECT_SOURCE_DIR}/${RL_DIR}")
execute_process(COMMAND lsb_release -sc OUTPUT_VARIABLE UBUNTU_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)

## robot_localization devel repo, works on Foxy -> Humble
## needed to get the headers
set(RL_BRANCH "$ENV{ROS_DISTRO}-devel")
set(RL_URL https://github.com/cra-ros-pkg/robot_localization)

if(NOT EXISTS ${RL_DIR_ABS})
    execute_process(COMMAND
    git clone ${RL_URL} -b ${RL_BRANCH} ${RL_DIR} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
else()
    execute_process(COMMAND git checkout -b ${RL_BRANCH} WORKING_DIRECTORY ${RL_DIR_ABS} OUTPUT_QUIET ERROR_QUIET)
endif()

ament_auto_add_executable(ekf
    src/ekf_node.cpp
    src/ros_filter_ranges.cpp)
target_include_directories(ekf PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${RL_DIR}/include)
target_link_libraries(ekf ${robot_localization_LIBRARIES})

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

install(PROGRAMS examples/3d/sim3d DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY examples DESTINATION share/${PROJECT_NAME})

ament_auto_package()
