# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(fanuc_benchmark)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(fanuc_msgs REQUIRED)

add_executable(motion_task_test test/motion_task_test.cpp)
ament_target_dependencies(motion_task_test
  rclcpp
  std_msgs
  tf2_ros
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  fanuc_msgs
)

install(TARGETS motion_task_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, pose definitions and the plotting helper.
install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

# Make the analysis / comparison helpers runnable via `ros2 run fanuc_benchmark <script>`.
install(PROGRAMS
  test/analyze_benchmark.py
  test/compare_benchmarks.py
  test/run_pipeline_comparison.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
