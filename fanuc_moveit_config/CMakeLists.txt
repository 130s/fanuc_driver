# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(fanuc_moveit_config)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_control_interface REQUIRED)
find_package(moveit_simple_controller_manager REQUIRED)
find_package(pluginlib REQUIRED)

add_library(fanuc_controllers_moveit_trajectory_plugin SHARED
    src/fanuc_controllers_moveit_trajectory_plugin.cpp
)
target_include_directories(fanuc_controllers_moveit_trajectory_plugin PUBLIC
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fanuc_controllers_moveit_trajectory_plugin PRIVATE
    moveit_ros_control_interface::moveit_ros_control_interface_trajectory_plugin)
ament_target_dependencies(fanuc_controllers_moveit_trajectory_plugin
  moveit_ros_control_interface
  moveit_simple_controller_manager
  pluginlib
)
ament_export_targets(export_fanuc_controllers_moveit_trajectory_plugin)
install(TARGETS fanuc_controllers_moveit_trajectory_plugin
        EXPORT export_fanuc_controllers_moveit_trajectory_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(moveit_ros_control_interface fanuc_controllers_moveit_trajectory_plugin.xml)

install(FILES
    fanuc_controllers_moveit_trajectory_plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz srdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
