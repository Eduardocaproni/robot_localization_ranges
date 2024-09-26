cmake_minimum_required(VERSION 3.5)
project(robot_localization_ranges)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_executable(ekf
    src/ekf_node.cpp
    src/ros_filter_ranges.cpp)
target_include_directories(ekf PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ekf ${robot_localization_LIBRARIES})

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

install(PROGRAMS examples/3d/sim3d DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY examples DESTINATION share/${PROJECT_NAME})

ament_auto_package()
