# SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    use_mock = LaunchConfiguration("use_mock")
    gpio_config_package = LaunchConfiguration("gpio_config_package")
    gpio_config_path = LaunchConfiguration("gpio_config_path")
    motion_control = LaunchConfiguration("motion_control")
    joint_limits_file = LaunchConfiguration("joint_limits_file")
    planning_pipeline = LaunchConfiguration("planning_pipeline")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_output = LaunchConfiguration("rviz_output")

    nodes_to_launch = []

    # Conditionally include the appropriate control launch file
    include_fanuc_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_physical_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": "crx",
            "gpio_config_package": gpio_config_package,
            "gpio_config_path": gpio_config_path,
            "robot_ip": robot_ip,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",  # RViz should be launched from within this launch file, not the launch in hardware_control pkg.
            "use_mock": use_mock,
            "motion_control": motion_control,
        }.items(),
        condition=UnlessCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_control)

    include_fanuc_mock_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_mock_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": "crx",
            "gpio_config_package": gpio_config_package,
            "gpio_config_path": gpio_config_path,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",  # RViz should be launched from within this launch file, not the launch in hardware_control pkg.
        }.items(),
        condition=IfCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_mock_control)

    description_arguments = {
        "robot_ip": robot_ip.perform(context),
        "use_mock": use_mock.perform(context),
        "gpio_configuration": PathJoinSubstitution(
            [FindPackageShare(gpio_config_package), gpio_config_path]
        ),
        "motion_control": motion_control.perform(context),
    }

    urdf_full_path = os.path.join(
        get_package_share_directory("fanuc_hardware_interface"),
        "robot",
        f"{robot_model.perform(context)}.urdf.xacro",
    )

    selected_planning_pipeline = planning_pipeline.perform(context)

    moveit_builder = (
        MoveItConfigsBuilder(
            robot_model.perform(context), package_name="fanuc_moveit_config"
        )
        .robot_description(file_path=urdf_full_path, mappings=description_arguments)
        .robot_description_semantic(
            file_path=f"srdf/{robot_model.perform(context)}.srdf"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path=joint_limits_file.perform(context))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=[selected_planning_pipeline])
    )

    if selected_planning_pipeline == "pilz_industrial_motion_planner":
        moveit_builder = moveit_builder.pilz_cartesian_limits()

    moveit_config = moveit_builder.to_moveit_configs()

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict()],
    )
    nodes_to_launch.append(move_group_node)

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("fanuc_moveit_config"), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # "both" -> RViz stdout/stderr go to the console AND the log file, so a
        # startup failure (e.g. "could not connect to display") is visible live
        # instead of only ending up in ~/.ros/log. Configurable via rviz_output.
        output=rviz_output.perform(context),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        arguments=["--display-config", rviz_file],
        condition=IfCondition(launch_rviz),
    )
    nodes_to_launch.append(rviz_node)

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            description="The robot model (required).",
            choices=[
                "crx3ia",
                "crx5ia",
                "crx10ia",
                "crx10ia_l",
                "crx20ia_l",
                "crx30ia",
            ],
        ),
        DeclareLaunchArgument(
            "robot_ip",
#            default_value="192.168.1.100",
            default_value="192.168.120.101",
            description="The robot's IP address.",
        ),
        DeclareLaunchArgument(
            "ros2_control_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "ros2_controllers.yaml",
                ]
            ),
            description="ROS 2 control configuration file the controllers.",
        ),
        DeclareLaunchArgument(
            "gpio_config_package",
            default_value="fanuc_hardware_interface",
            description="The package name where gpio_configuration file exists",
        ),
        DeclareLaunchArgument(
            "gpio_config_path",
            default_value="config/example_gpio_config.yaml",
            description="The gpio_configuration file path in gpio_config_package",
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Whether to use a mock hardware interface.",
        ),
        DeclareLaunchArgument(
            "motion_control",
            default_value="1",
            description="Initial motion control state.",
        ),
        DeclareLaunchArgument(
            "joint_limits_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_moveit_config"),
                    "config",
                    "joint_limits.yaml",
                ]
            ),
            description="Joint limits YAML file passed to MoveIt.",
        ),
        DeclareLaunchArgument(
            "planning_pipeline",
            default_value="ompl",
            description="MoveIt planning pipeline to use.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Whether to launch RViz for visualization.",
        ),
        DeclareLaunchArgument(
            "rviz_output",
            default_value="both",
            choices=["screen", "log", "both"],
            description=(
                "RViz logging destination: 'log' (file only), 'screen' "
                "(console only), or 'both' (console + file). Use 'both' to make "
                "RViz startup failures visible on the console."
            ),
        ),
        DeclareLaunchArgument(
            "motion_control",
            default_value="1",
            description="Initial motion control state.",
        ),
        DeclareLaunchArgument(
            "joint_limits_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_moveit_config"),
                    "config",
                    "joint_limits.yaml",
                ]
            ),
            description="Joint limits YAML file passed to MoveIt.",
        ),
        DeclareLaunchArgument(
            "planning_pipeline",
            default_value="ompl",
            description="MoveIt planning pipeline to use.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Whether to launch RViz for visualization.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
