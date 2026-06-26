# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0
#
# Bag operation + end-of-run orchestration for the FANUC benchmark.
#
# This is the one piece of the benchmark whose control flow cannot be expressed
# in launch-XML (XML has no event handlers). It owns everything that happens
# around the recorded bag:
#
#   1. records the benchmark topics to <output_dir>/benchmark_<timestamp>,
#   2. when the motion-task runner exits (run complete OR aborted), stops the
#      recorder cleanly (SIGINT) so the bag is finalized,
#   3. runs analyze_benchmark.py on the finalized bag so the per-MOTR results /
#      stats are printed to the terminal automatically, and
#   4. shuts the whole launch down so no processes are left alive.
#
# It watches the runner by process name (it lives in a sibling include, so its
# action handle is not directly available here) -- a launch_ros Node named
# "motion_task_test" appears as a process named "motion_task_test-<n>".

import os
import signal

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration

# Topics recorded for the benchmark: planned (controller_state.reference) vs.
# actual (controller_state.feedback / joint_states) joints, the collaborative
# speed scaling, the per-MOTR timing messages and the displayed planned paths.
BENCHMARK_TOPICS = [
    "/motion_task_test/motr_result",
    "/joint_states",
    "/joint_trajectory_controller/controller_state",
    "/fanuc_gpio_controller/collaborative_speed_scaling",
    "/display_planned_path",
]

# A launch_ros Node named "motion_task_test" runs as process "motion_task_test-<n>".
RUNNER_PROCESS_PREFIX = "motion_task_test"


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    output_dir = arg("output_dir")
    timestamp = arg("timestamp")
    record_bag = _as_bool(arg("record_bag"))
    run_analyze = _as_bool(arg("analyze"))

    bag_path = os.path.join(output_dir, f"benchmark_{timestamp}")

    # No bag requested: the only job left is to end the launch when the runner
    # finishes, so processes are not left alive.
    if not record_bag:
        def _shutdown_on_runner_exit(event, _context):
            if str(event.process_name).startswith(RUNNER_PROCESS_PREFIX):
                return [
                    LogInfo(msg="[benchmark] runner finished (no bag); shutting down."),
                    Shutdown(reason="runner finished"),
                ]
            return []

        return [RegisterEventHandler(OnProcessExit(on_exit=_shutdown_on_runner_exit))]

    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_path, *BENCHMARK_TOPICS],
        output="screen",
        name="benchmark_rosbag",
    )

    # Step 2: when the runner exits, SIGINT the recorder so the bag is finalized
    # before anything reads it. SIGINT (not SIGTERM) lets `ros2 bag record` close
    # the bag cleanly.
    def _stop_recorder_on_runner_exit(event, _context):
        if not str(event.process_name).startswith(RUNNER_PROCESS_PREFIX):
            return []
        return [
            LogInfo(msg=f"[benchmark] runner finished; finalizing bag at {bag_path} ..."),
            EmitEvent(
                event=SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda action: action is recorder,
                )
            ),
        ]

    # Step 3+4: once the bag is finalized, optionally analyze it, then shut down.
    if run_analyze:
        analyze = ExecuteProcess(
            cmd=["ros2", "run", "fanuc_benchmark", "analyze_benchmark.py", bag_path, "--no-show"],
            output="screen",
            name="analyze_benchmark",
        )
        on_recorder_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=recorder,
                on_exit=[
                    LogInfo(msg="[benchmark] bag finalized; running analyze_benchmark.py ..."),
                    analyze,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=analyze,
                            on_exit=[Shutdown(reason="analysis complete; ending benchmark")],
                        )
                    ),
                ],
            )
        )
    else:
        on_recorder_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=recorder,
                on_exit=[Shutdown(reason="bag finalized; ending benchmark")],
            )
        )

    return [
        recorder,
        RegisterEventHandler(OnProcessExit(on_exit=_stop_recorder_on_runner_exit)),
        on_recorder_exit,
    ]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "output_dir",
            default_value=os.environ.get("PWD", os.getcwd()),
            description="Directory for the recorded bag.",
        ),
        DeclareLaunchArgument(
            "timestamp",
            default_value="run",
            description="Bag name suffix (benchmark_<timestamp>). Passed in by benchmark.launch.xml.",
        ),
        DeclareLaunchArgument(
            "record_bag",
            default_value="true",
            description="Record a rosbag of the run.",
        ),
        DeclareLaunchArgument(
            "analyze",
            default_value="true",
            description="After the run, automatically run analyze_benchmark.py and print results to the terminal.",
        ),
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=launch_setup)])
