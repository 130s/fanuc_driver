# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0
#
# Minimal Python helper: builds the MoveIt configuration and starts the
# motion_task_test node. This is the ONLY part of the launch that cannot be
# expressed in launch-XML, because it relies on MoveItConfigsBuilder (the same
# reason fanuc_moveit.launch.py is Python). Everything else -- argument
# defaults, bringing up move_group + driver and rosbag recording -- lives in
# benchmark.launch.xml, which includes this file.
#
# The node receives:
#   * the full MoveIt config (so MTC, which plans in-process, has the robot
#     model, kinematics, joint limits and planning pipeline), and
#   * the run-time options + waypoint poses, forwarded from the XML entry point.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot_model = arg("robot_model")

    # Mirror the xacro mappings used by fanuc_moveit.launch.py so the kinematic
    # model is identical to the one move_group loads. Real hardware only.
    description_arguments = {
        "robot_ip": arg("robot_ip"),
        "use_mock": "false",
        "gpio_configuration": PathJoinSubstitution(
            [FindPackageShare(arg("gpio_config_package")), arg("gpio_config_path")]
        ),
    }
    urdf_full_path = os.path.join(
        get_package_share_directory("fanuc_hardware_interface"),
        "robot",
        f"{robot_model}.urdf.xacro",
    )

    planning_pipeline = arg("planning_pipeline")
    builder = (
        MoveItConfigsBuilder(robot_model, package_name="fanuc_moveit_config")
        .robot_description(file_path=urdf_full_path, mappings=description_arguments)
        .robot_description_semantic(file_path=f"srdf/{robot_model}.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path=arg("joint_limits_file"))
        # This node is a MoveIt client; move_group owns publishing the robot
        # description, so don't re-publish it here.
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=False,
        )
        .planning_pipelines(pipelines=[planning_pipeline])
    )
    if planning_pipeline == "pilz_industrial_motion_planner":
        builder = builder.pilz_cartesian_limits()
    moveit_config = builder.to_moveit_configs()

    # Run-time options forwarded from the XML entry point. Cast to native types
    # so the node sees correctly-typed parameters (substitutions are strings).
    run_params = {
        "planner": arg("planner"),
        "group": arg("group"),
        "planner_id": arg("planner_id"),
        "iterations": int(arg("iterations")),
        "velocity_scaling": float(arg("velocity_scaling")),
        "acceleration_scaling": float(arg("acceleration_scaling")),
        "planning_time": float(arg("planning_time")),
        "planning_attempts": int(arg("planning_attempts")),
        "state_wait_timeout": float(arg("state_wait_timeout")),
        "mtc_solver": arg("mtc_solver"),
        "mtc_pipeline": arg("mtc_pipeline"),
    }

    motion_task_node = Node(
        package="fanuc_benchmark",
        executable="motion_task_test",
        name="motion_task_test",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # robot description, kinematics, limits, pipelines
            arg("poses_file"),  # joint_names / waypoint_names / poses.*
            run_params,  # planner selection and run options (override)
        ],
    )
    return [motion_task_node]


def generate_launch_description():
    # Defaults here are placeholders; benchmark.launch.xml passes real values.
    names = [
        ("robot_model", "crx30ia"),
        ("robot_ip", "192.168.120.101"),
        ("planner", "ompl"),
        ("group", "manipulator"),
        ("planner_id", ""),
        ("planning_pipeline", "ompl"),
        ("gpio_config_package", "fanuc_hardware_interface"),
        ("gpio_config_path", "config/example_gpio_config.yaml"),
        ("iterations", "3"),
        ("velocity_scaling", "0.1"),
        ("acceleration_scaling", "0.1"),
        ("planning_time", "5.0"),
        ("planning_attempts", "10"),
        ("state_wait_timeout", "30.0"),
        ("mtc_solver", "pipeline"),
        ("mtc_pipeline", "ompl"),
    ]
    declared = [DeclareLaunchArgument(n, default_value=d) for n, d in names]
    declared.append(
        DeclareLaunchArgument(
            "joint_limits_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("fanuc_moveit_config"), "config", "joint_limits.yaml"]
            ),
        )
    )
    declared.append(
        DeclareLaunchArgument(
            "poses_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("fanuc_benchmark"), "test", "benchmark_poses.yaml"]
            ),
        )
    )
    return LaunchDescription(declared + [OpaqueFunction(function=launch_setup)])
